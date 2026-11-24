#!/usr/bin/env python3
"""
The sleep_between_turns variant: the whole of tests/test_cruise.py (cruise scenarios on a fake robot in real
time) with the variant on.

    python3 varianti/variant_tests/test_sleep_between_turns.py
"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))    # varianti/
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, 'tests')]
os.chdir(ROOT)

import sleep_between_turns      # noqa: E402
import test_cruise   # noqa: E402

sleep_between_turns.apply()
sys.exit(test_cruise.main())
