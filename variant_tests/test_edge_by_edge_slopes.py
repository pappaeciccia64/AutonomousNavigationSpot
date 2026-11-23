#!/usr/bin/env python3
"""
The edge_by_edge_slopes variant: test_replay on its two reference missions with the variant on
must give exactly the reference behaviour (the slopes are the same numbers).

    python3 varianti/variant_tests/test_edge_by_edge_slopes.py
"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))    # varianti/
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, 'tests')]
os.chdir(ROOT)

import edge_by_edge_slopes    # noqa: E402

edge_by_edge_slopes.apply()
sys.argv = ['test_replay.py']
import test_replay            # noqa: E402

sys.exit(test_replay.main())
