#!/usr/bin/env python3
"""
The event_driven_planning variant: the whole of tests/test_cruise.py (cruise scenarios on a fake robot in real
time) with the variant on.

    python3 varianti/variant_tests/test_event_driven_planning.py
"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))    # varianti/
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, 'tests')]
os.chdir(ROOT)

import event_driven_planning      # noqa: E402
import test_cruise   # noqa: E402

event_driven_planning.apply()
sys.exit(test_cruise.main())
