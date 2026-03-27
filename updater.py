"""Self-updater: check GitHub releases, download exe to staging, apply on restart."""

import json
import logging
import os
import subprocess
import sys
import tempfile
import threading
import urllib.request

from version import __version__ as APP_VERSION, DEV_MODE

log = logging.getLogger("updater")

GITHUB_REPO = "Toon-Red/plc-learning-env"
RELEASES_URL = f"https://api.github.com/repos/{GITHUB_REPO}/releases"
LATEST_URL = f"{RELEASES_URL}/latest"

APP_NAME = "PLCLearningEnv"
_NO_WINDOW = subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0


def _headers() -> dict:
    """Request headers with GitHub token for private repo access."""
    h = {"User-Agent": APP_NAME, "Accept": "application/vnd.github+json"}
    # Read token from gh CLI
    try:
        result = subprocess.run(
            ["gh", "auth", "token"],
            capture_output=True, text=True, timeout=5,
            creationflags=_NO_WINDOW if sys.platform == "win32" else 0,
        )
        token = result.stdout.strip()
        if token:
            h["Authorization"] = f"Bearer {token}"
    except Exception:
        pass
    return h


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

def _staging_dir():
    d = os.path.join(
        os.environ.get("LOCALAPPDATA", tempfile.gettempdir()),
        APP_NAME, "updates",
    )
    os.makedirs(d, exist_ok=True)
    return d


def _staging_exe():
    return os.path.join(_staging_dir(), f"{APP_NAME}_new.exe")


def _staging_version_file():
    return os.path.join(_staging_dir(), "staged_version.txt")


# ---------------------------------------------------------------------------
# Version helpers
# ---------------------------------------------------------------------------

def get_current_version() -> str:
    return APP_VERSION


def is_newer(latest: str, current: str) -> bool:
    try:
        return [int(x) for x in latest.split(".")] > [int(x) for x in current.split(".")]
    except Exception:
        return False


# ---------------------------------------------------------------------------
# GitHub API
# ---------------------------------------------------------------------------

def get_latest_release() -> dict | None:
    """Fetch latest release. Returns dict with version, exe_url, notes, published, name."""
    if DEV_MODE:
        log.info("Dev build — skipping release update check")
        return None
    try:
        req = urllib.request.Request(LATEST_URL, headers=_headers())
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read())
        tag = data.get("tag_name", "").lstrip("v")
        if not tag:
            return None
        exe_url = None
        for asset in data.get("assets", []):
            if asset["name"] == f"{APP_NAME}.exe":
                # Use API URL for private repos (browser_download_url requires browser auth)
                exe_url = asset.get("url") or asset["browser_download_url"]
                break
        return {
            "version": tag,
            "exe_url": exe_url,
            "notes": data.get("body", ""),
            "published": data.get("published_at", ""),
            "name": data.get("name", f"v{tag}"),
        }
    except Exception as e:
        log.warning("Failed to check latest release: %s", e)
        return None


def get_all_releases(limit: int = 10) -> list:
    """Fetch recent releases with notes."""
    try:
        req = urllib.request.Request(f"{RELEASES_URL}?per_page={limit}", headers=_headers())
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read())
        releases = []
        for r in data:
            tag = r.get("tag_name", "").lstrip("v")
            releases.append({
                "version": tag,
                "name": r.get("name", f"v{tag}"),
                "notes": r.get("body", ""),
                "published": r.get("published_at", ""),
                "current": tag == APP_VERSION,
            })
        return releases
    except Exception as e:
        log.warning("Failed to fetch releases: %s", e)
        return []


# ---------------------------------------------------------------------------
# Staging
# ---------------------------------------------------------------------------

def is_update_staged() -> bool:
    return os.path.exists(_staging_exe()) and os.path.exists(_staging_version_file())


def get_staged_version() -> str | None:
    try:
        with open(_staging_version_file()) as f:
            return f.read().strip()
    except Exception:
        return None


def clear_staging():
    try:
        for name in (f"{APP_NAME}_new.exe", "staged_version.txt", "apply_update.bat"):
            path = os.path.join(_staging_dir(), name)
            if os.path.exists(path):
                os.remove(path)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Download
# ---------------------------------------------------------------------------

def download_update(url: str, version: str, progress_callback=None) -> bool:
    """Download new exe to staging dir. Returns True on success."""
    try:
        staging_exe = _staging_exe()
        hdrs = _headers()
        # API asset URLs need octet-stream accept to get the binary
        if "api.github.com" in url:
            hdrs["Accept"] = "application/octet-stream"
        req = urllib.request.Request(url, headers=hdrs)
        with urllib.request.urlopen(req, timeout=300) as resp:
            total = int(resp.headers.get("Content-Length", 0))
            downloaded = 0
            with open(staging_exe, "wb") as f:
                while True:
                    chunk = resp.read(65536)
                    if not chunk:
                        break
                    f.write(chunk)
                    downloaded += len(chunk)
                    if progress_callback:
                        progress_callback(downloaded, total)

        file_size = os.path.getsize(staging_exe)
        if file_size < 1_000_000:
            log.error("Downloaded file too small (%d bytes) — likely corrupted", file_size)
            os.remove(staging_exe)
            return False

        with open(_staging_version_file(), "w") as f:
            f.write(version)

        log.info("Downloaded update v%s to %s (%d bytes)", version, staging_exe, file_size)
        return True
    except Exception as e:
        log.error("Failed to download update: %s", e)
        try:
            os.remove(_staging_exe())
        except Exception:
            pass
        return False


# ---------------------------------------------------------------------------
# Apply update (spawn batch script, exit app)
# ---------------------------------------------------------------------------

def _install_dir():
    return os.path.join(os.environ.get("LOCALAPPDATA", ""), APP_NAME)


def _write_apply_script() -> str:
    """Write batch script that waits for us to die, swaps exe, relaunches."""
    staging_exe = _staging_exe()
    target_exe = os.path.join(_install_dir(), f"{APP_NAME}.exe")
    bat_path = os.path.join(_staging_dir(), "apply_update.bat")
    pid = os.getpid()

    mei_cleanup = ""
    if getattr(sys, "_MEIPASS", None):
        mei_dir = sys._MEIPASS
        mei_cleanup = f'if exist "{mei_dir}" rmdir /s /q "{mei_dir}" 2>nul'

    script = f"""@echo off
echo Applying PLC Learning Environment update...

rem Wait for the app process to exit (up to 30s)
set COUNT=0
:waitloop
tasklist /fi "PID eq {pid}" 2>nul | find "{pid}" >nul
if errorlevel 1 goto proceed
set /a COUNT+=1
if %COUNT% geq 30 goto proceed
timeout /t 1 /nobreak >nul
goto waitloop

:proceed
timeout /t 1 /nobreak >nul

rem Backup old exe
if exist "{target_exe}.bak" del /f /q "{target_exe}.bak"
if exist "{target_exe}" move /y "{target_exe}" "{target_exe}.bak"

rem Install new exe
copy /y "{staging_exe}" "{target_exe}"

rem Clean up staging
del /f /q "{staging_exe}" 2>nul
del /f /q "{os.path.join(_staging_dir(), 'staged_version.txt')}" 2>nul

rem Clean up backup
if exist "{target_exe}.bak" del /f /q "{target_exe}.bak"

rem Clean up PyInstaller temp dir
{mei_cleanup}

rem Relaunch
start "" "{target_exe}"

rem Self-delete
del "%~f0"
"""
    with open(bat_path, "w") as f:
        f.write(script)
    return bat_path


def apply_update_and_restart():
    """Spawn the batch script and exit. Called when user clicks 'Restart Now'."""
    if not is_update_staged():
        return False
    if not getattr(sys, "frozen", False):
        log.info("Dev mode — clearing staging instead of applying")
        clear_staging()
        return False

    bat_path = _write_apply_script()
    subprocess.Popen(
        f'cmd /c "{bat_path}"',
        creationflags=_NO_WINDOW,
        close_fds=True,
    )
    log.info("Update apply script launched, exiting...")
    os._exit(0)


def check_and_apply_staged():
    """Called on cold start. If a staged update exists, apply it before starting."""
    if not is_update_staged():
        return False

    staged_ver = get_staged_version()
    if not staged_ver:
        clear_staging()
        return False

    if staged_ver == APP_VERSION:
        log.info("Staged v%s matches current — cleaning up stale staging", staged_ver)
        clear_staging()
        return False

    if not is_newer(staged_ver, APP_VERSION):
        log.info("Staged v%s is not newer than current v%s — discarding", staged_ver, APP_VERSION)
        clear_staging()
        return False

    if not getattr(sys, "frozen", False):
        log.info("Dev mode — clearing staging")
        clear_staging()
        return False

    log.info("Applying staged update v%s on cold start...", staged_ver)
    bat_path = _write_apply_script()
    subprocess.Popen(
        f'cmd /c "{bat_path}"',
        creationflags=_NO_WINDOW,
        close_fds=True,
    )
    os._exit(0)


# ---------------------------------------------------------------------------
# Download manager (thread-safe state for progress tracking)
# ---------------------------------------------------------------------------

class UpdateManager:
    def __init__(self):
        self._lock = threading.Lock()
        self.downloading = False
        self.progress = 0
        self.received = 0
        self.total = 0
        self.error = None

    def start_download(self, url: str, version: str) -> bool:
        if self.downloading:
            return False
        with self._lock:
            self.downloading = True
            self.progress = 0
            self.received = 0
            self.total = 0
            self.error = None
        t = threading.Thread(target=self._run, args=(url, version), daemon=True)
        t.start()
        return True

    def _run(self, url, version):
        def on_progress(received, total):
            with self._lock:
                self.received = received
                self.total = total
                self.progress = int(received * 100 / total) if total > 0 else -1

        ok = download_update(url, version, progress_callback=on_progress)
        with self._lock:
            self.downloading = False
            if not ok:
                self.error = "Download failed"
            else:
                self.progress = 100

    def get_status(self) -> dict:
        with self._lock:
            return {
                "downloading": self.downloading,
                "progress": self.progress,
                "received_mb": round(self.received / 1048576, 1),
                "total_mb": round(self.total / 1048576, 1),
                "error": self.error,
                "staged": is_update_staged(),
                "staged_version": get_staged_version(),
            }


update_manager = UpdateManager()
