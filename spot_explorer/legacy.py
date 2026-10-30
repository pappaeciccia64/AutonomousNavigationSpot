"""
Names used by the missions recorded before the code was translated into English (up to
30-10-2026): values in the scan bundles, trajectory.csv, mission_summary.json and the logs.

The readers of saved mission data (tools/, tests/test_replay.py, tests/test_goal_selection.py)
pass what they read through modern(), so old and new missions read the same way. Nothing in
the mission itself uses this module.
"""

OLD_VALUES = {
    # frontier stop reason (frontier_reason in scans/*.npz)
    'ostacolo': 'obstacle', 'rugosita': 'roughness', 'pendenza': 'slope',
    'fine_dati': 'end_of_data', 'fine_percorso': 'end_of_path',
    # graph node method and random distributions ([CONFIG] line, mission_summary.json)
    'reticolo': 'lattice', 'casuale': 'random', 'uniforme': 'uniform', 'perturbata': 'jittered',
    # movement kind (column 'move' of trajectory.csv)
    'cono': 'cone', 'arretramento': 'backoff', 'ritirata': 'retreat',
    'graphnav_correzione': 'graphnav_correction',
}


def modern(value):
    """The current name of a value read from mission data; unchanged if it is already current."""
    return OLD_VALUES.get(value, value)
