#!/usr/bin/env python3
"""
The wake_on_plan variant: the whole of tests/test_cruise.py (cruise scenarios on a fake robot in real
time) with the variant on.

    python3 varianti/variant_tests/test_wake_on_plan.py
"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))    # varianti/
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, 'tests')]
os.chdir(ROOT)

import wake_on_plan      # noqa: E402
import test_cruise   # noqa: E402

wake_on_plan.apply()
sys.exit(test_cruise.main())
