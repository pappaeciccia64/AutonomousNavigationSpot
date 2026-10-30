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
    # kind of graph update ([TEMPO] line)
    'aggiornamento locale': 'local update', 'costruzione completa': 'full build',
}

# Keys of mission_summary.json (and of MISSION_STATS inside it).
OLD_KEYS = {
    'interruttori': 'switches', 'fiducial_partenza': 'start_fiducial', 'grafo': 'graph',
    'progresso': 'progress', 'celle_chiuse': 'closed_cells', 'celle_totali': 'total_cells',
    'percentuale': 'percent', 'visitate': 'visited', 'bloccate': 'blocked', 'area_vista': 'seen_area',
    'minuti': 'minutes', 'minuti_a_cella': 'minutes_per_cell', 'batteria': 'battery', 'finale': 'final',
    'metodo': 'method', 'nodi': 'nodes', 'coppie': 'pairs', 'arco_min_m': 'edge_min_m',
    'arco_max_m': 'edge_max_m', 'copertura_m': 'coverage_m', 'distribuzione': 'distribution',
    'densita_nodi_m2': 'node_density_m2', 'seme': 'seed', 'passo_m': 'spacing_m', 'anelli': 'rings',
    # phases of MISSION_STATS['time_s'] (timing_add)
    'movimento': 'motion', 'scansione': 'scan', 'PRM e ripianificazione': 'PRM and replanning',
    'costruzione grafo': 'graph construction', 'elaborazione griglia': 'grid processing',
    'mappe globali': 'global maps', 'fronte e decisione': 'frontier and decision',
    'dati figure': 'figure data',
}

# Phases of the [TIMING] Mission closure block of the log.
OLD_CLOSURE_PHASES = {
    'chiusura mappa e ancoraggio': 'map closure and anchoring', 'ritorno a wp_0': 'return to wp_0',
    'seduta e spegnimento': 'sit and power off', 'scaricamento della mappa': 'map download',
}

# Log tags, longest first so that '[GRAFO DISCONNESSO]' is not read as '[GRAFO]'.
OLD_TAGS = {
    'GRAFO DISCONNESSO': 'GRAPH DISCONNECTED',
    'LOCALIZZAZIONE': 'LOCALIZATION',
    'REPLAN FALLITO': 'REPLAN FAILED',
    'PASSO PARZIALE': 'PARTIAL STEP',
    'REPLAN STABILE': 'REPLAN STABLE',
    'REGISTRAZIONE': 'RECORDING',
    'ANELLI-BANCO': 'LOOPS-BENCH',
    'ARRETRAMENTO': 'BACKOFF',
    'PASSO FINALE': 'FINAL STEP',
    'SCORCIATOIA': 'SHORTCUT',
    'DEVIAZIONE': 'DETOUR',
    'PROGRESSO': 'PROGRESS',
    'EMERGENZA': 'EMERGENCY',
    'MULTINODO': 'MULTINODE',
    'ROTAZIONE': 'ROTATION',
    'OBIETTIVO': 'GOAL',
    'MOVIMENTO': 'MOVE',
    'FRONTIERA': 'CELL FRONTIER',
    'BATTERIA': 'BATTERY',
    'RITIRATA': 'RETREAT',
    'TRAPPOLA': 'TRAP',
    'BRICIOLA': 'BREADCRUMB',
    'PENDENZE': 'SLOPES',
    'RITORNO': 'RETURN',
    'DIVERSO': 'DIFFERENT',
    'SUBITO': 'EARLY',
    'CADUTA': 'FALL',
    'ARRIVO': 'ARRIVAL',
    'BLOCCO': 'BLOCK',
    'FRONTE': 'FRONTIER',
    'STALLO': 'STALL',
    'DERIVA': 'DRIFT',
    'UGUALE': 'SAME',
    'SPAZIO': 'SPACE',
    'ANELLI': 'LOOPS',
    'GRAFO': 'GRAPH',
    'TEMPO': 'TIMING',
    'TEMPI': 'TIMES',
    'QUASI': 'ALMOST',
    'FINTO': 'FAKE',
    'ARCHI': 'EDGES',
    'QUOTE': 'HEIGHTS',
    'CELLA': 'CELL',
    'PIANO': 'PLAN',
    'MANCA': 'MISSING',
    'FINE': 'END',
    'CONO': 'CONE',
    'NODO': 'NODE',
    'NODI': 'NODES',
}

# Phrases of the old logs that the readers look for, in the words of the current logs.
OLD_PHRASES = [
    ('Provo a entrare nella cella', 'Trying to enter cell'),
    (': avvio (massimo', ': start (at most'),
    ('prima del ritorno a wp_0', 'before the return to wp_0'),
    ('[WP0] CORREZIONE', '[WP0] CORRECTION'),
    ('Chiusura della missione', 'Mission closure'),
    ('ritorno a wp_0: arrivato', 'return to wp_0: arrived'),
    ('Ritorno a wp_0 NON riuscito', 'Return to wp_0 NOT successful'),
    ("Secondo il tag il robot e' a", 'According to the tag the robot is'),
    ('Dopo la correzione', 'After correction'),
    ('m dalla partenza (prima', 'm from the start (before'),
    ('min trascorsi', 'min elapsed'),
]


def modern(value):
    """The current name of a value read from mission data; unchanged if it is already current."""
    return OLD_VALUES.get(value, value)


def modern_keys(obj):
    """mission_summary.json with the current key names (and values), at every level."""
    if isinstance(obj, dict):
        return {OLD_KEYS.get(k, k): modern_keys(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [modern_keys(v) for v in obj]
    return modern(obj) if isinstance(obj, str) else obj


def modern_log_line(line):
    """A line of an old mission log with the current tags and the phrases the readers look for."""
    if '[' in line:
        for old, new in OLD_TAGS.items():
            line = line.replace('[' + old + ']', '[' + new + ']')
    for old, new in OLD_PHRASES:
        line = line.replace(old, new)
    return line
