#!/usr/bin/env python3

import os
import subprocess

if __name__ == '__main__':
# Launch the WebSocket script in a separate process
    subprocess.run(['python3', os.path.join(os.path.dirname(__file__), 'diagnostics_websocket.py')])
