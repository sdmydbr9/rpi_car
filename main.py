#!/usr/bin/env python3
"""Backward-compatible wrapper — Python scripts now live in scripts/.
This simply delegates to scripts/main.py so existing workflows still work.
"""
import os
import sys
import runpy

_here = os.path.dirname(os.path.abspath(__file__))
_scripts = os.path.join(_here, "scripts")

# Ensure scripts/ is on the module path
sys.path.insert(0, _scripts)

# Run scripts/main.py as __main__ with correct __file__
runpy.run_path(os.path.join(_scripts, "main.py"), run_name="__main__")
