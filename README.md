# PLC Learning Environment

> **DEPRECATED** (per PD project YAML, lifecycle: deprecated). Python conveyor-logic simulator -- does not interact with real Codesys PLC code, which is the actual skill Preston needs. Kept around for reference; will be replaced by a tool that drives real Codesys interaction.

## What this was

Interactive PLC learning simulator following IEC 61131-3 architecture. Implemented conveyor and sorter logic in Python to teach control-flow patterns without requiring real PLC hardware.

## Why deprecated

Python simulation does not teach real PLC skills. The replacement must drive an actual Codesys runtime so the learner exercises the same toolchain they will use in production.

## Pre-push hook (local CI gate)

If you do touch this repo (bug fixes, documentation, etc.), the pre-push hook still runs the test suite before allowing a push:

```sh
cp scripts/pre-push.sh .git/hooks/pre-push
chmod +x .git/hooks/pre-push
```

The hook excludes `test_server.py` (manual integration script that requires the dev server running on :8080). Reversible -- delete `.git/hooks/pre-push` to disable.

## Related repos

- [pipeline-dashboard](https://github.com/Toon-Red/pipeline-dashboard) -- tracks this project's deprecation status
