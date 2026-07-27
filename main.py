#!/usr/bin/env python3
"""Start the manual-only control server from the repository root."""

import os
import runpy
import sys

_here = os.path.dirname(os.path.abspath(__file__))
_scripts = os.path.join(_here, "scripts")

sys.path.insert(0, _scripts)
sys.path.insert(0, os.path.join(_scripts, "core"))

runpy.run_path(os.path.join(_scripts, "main.py"), run_name="__main__")
