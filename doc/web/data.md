# data.html

## Overview
- Download/management page for recorded log files.
- Talks to `hydroball_files_websocket_node` over WebSocket (`ws://<host>:9003`) to list, delete, and publish files.
- Provides pagination, search, select/delete, and a “Publish All Files” flow with status modal.

## Data Source (WebSocket messages)
- Endpoint: `ws://<host>:9003` (no auth).
- Expected messages:
  - File list: `{fileslist: [[filename, downloadUrl], ...]}` — rendered into a table with checkboxes and download links.
  - Publish status: `{publishstatus: "<step text>", done: <bool>}` — appended to the modal list; `done: true` enables the Close button.

## Commands (browser → WebSocket server)
- `{"f-list":"fileslist"}`: periodic poll every 500 ms to refresh the file list.
- `{"delete":"<filename>"}`: delete a selected file (driven by the delete modal).
- `{"publishfiles": true}`: trigger transfer of all files; server streams back `publishstatus` messages.

## UI / Behavior
- File table with pagination (per-page selector, first/prev/next/last) and client-side search filtering by filename.
- “Select All Files” toggles checkboxes for the current page; “Delete Selected Files” shows a confirmation modal then sends delete commands for checked rows on the current page.
- “Publish All Files” opens a modal and requests transfer; status lines accumulate until completion.
- Download links open/trigger download via the provided URLs in `fileslist`.
- File list refreshes on a fixed interval; table rebuilds when the list length changes.

## Scripts
- `js/dataScript.js`: WebSocket setup, periodic polling, file list rendering, delete/select/pagination/search handlers, publish modal updates.
- Shared includes: `js/script.js`, SB Admin 2 assets; no charts used.

## Notes / Caveats
- No authentication/TLS on the WebSocket; keep the UI on trusted networks.
- Delete is permanent (no undo). Poll interval is 500 ms; adjust if server load is a concern.
