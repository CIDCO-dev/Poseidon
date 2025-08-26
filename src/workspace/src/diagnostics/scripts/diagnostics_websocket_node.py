#!/usr/bin/env python3

import os
import subprocess

if __name__ == '__main__':
    # Lance le script websocket dans un processus séparé
    subprocess.run(['python3', os.path.join(os.path.dirname(__file__), 'diagnostics_websocket.py')])
