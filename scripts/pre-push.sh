#!/usr/bin/env sh
# Pre-push hook -- runs the local test suite and blocks the push on red.
# Replaces the retired GitHub Actions "Tests" workflow (2026-05-14).
#
# Framework auto-detection (PD-PHK1, 2026-05-17):
#   pyproject.toml / requirements.txt  -> python -m pytest
#   package.json with "test" script    -> npm test
#   Cargo.toml                          -> cargo test
#   none of the above                   -> skip with a notice (exit 0)
#
# Install:
#   cp scripts/pre-push.sh .git/hooks/pre-push && chmod +x .git/hooks/pre-push
#
# Bypass once (use sparingly, only for hotfixes):
#   git push --no-verify

set -e

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

run_tests() {
    if [ -f "pyproject.toml" ] || [ -f "requirements.txt" ]; then
        echo "[pre-push] detected Python -- running pytest..."
        # test_server.py is a manual integration script that requires the
        # dev server on :8080. Excluded so the hook stays clean.
        python -m pytest -q --tb=short --ignore=test_server.py
        return $?
    fi
    if [ -f "package.json" ] && grep -q '"test"' package.json; then
        echo "[pre-push] detected Node -- running npm test..."
        npm test
        return $?
    fi
    if [ -f "Cargo.toml" ]; then
        echo "[pre-push] detected Rust -- running cargo test..."
        cargo test
        return $?
    fi
    echo "[pre-push] no test framework detected (no pyproject/requirements/package.json/Cargo.toml). Skipping."
    return 0
}

if run_tests; then
    echo "[pre-push] tests green -- proceeding."
    exit 0
else
    echo "[pre-push] tests RED -- push blocked. Fix or use --no-verify."
    exit 1
fi
