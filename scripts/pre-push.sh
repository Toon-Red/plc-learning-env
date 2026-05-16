#!/usr/bin/env sh
# Pre-push hook -- runs the local test suite and blocks the push on red.
# Replaces the retired GitHub Actions "Tests" workflow (2026-05-14).
#
# `test_server.py` is an interactive integration script that requires the
# dev server running on :8080 -- excluded so the hook stays clean for
# normal push gating.
#
# Install:
#   cp scripts/pre-push.sh .git/hooks/pre-push && chmod +x .git/hooks/pre-push
#
# Bypass once (use sparingly, only for hotfixes):
#   git push --no-verify

set -e

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

echo "[pre-push] running pytest..."
if python -m pytest -q --tb=short --ignore=test_server.py; then
    echo "[pre-push] tests green -- proceeding."
    exit 0
else
    echo "[pre-push] tests RED -- push blocked. Fix or use --no-verify."
    exit 1
fi
