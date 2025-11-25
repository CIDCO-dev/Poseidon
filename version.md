# Version History (English)

## Unreleased
- Added HTTPS transfer integration tests for logger binary and text nodes (local TLS server, payload verification).
- Added memory-resilience tests: repeated failed transfers must not increase RSS.
- Fixed `create_json_str` to avoid allocation leak (use `SetString` without manual `new`).
- Updated agent guidelines to maintain this `version.md`.
