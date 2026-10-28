#!/usr/bin/env python3
"""
Controllo pre-volo: verifica che i file sulla macchina del robot siano allineati fra
loro PRIMA di lanciare una missione.

Se sul robot `easy_walk.py` e' aggiornato ma un altro modulo e' una copia vecchia, la
missione si ferma con un AttributeError appena il robot e' acceso e in piedi: tempo perso,
e in esterno tempo perso a batteria accesa in mezzo a un campo. Qui lo si scopre prima.

Il controllo e' PURAMENTE STATICO: legge i file, non importa nulla, non tocca il robot,
non serve l'SDK bosdyn. Si puo' lanciare in qualunque momento.

    python3 preflight.py                 # nella cartella del progetto
    python3 preflight.py /percorso/cartella

Cosa controlla:
  1. ogni file della missione esiste e non ha errori di sintassi;
  2. IN AUTOMATICO, per ogni file: ogni `modulo.nome` e ogni `from modulo import nome` verso
     un altro modulo del progetto esiste davvero in quel modulo, e gli argomenti con nome
     passati a una funzione di un altro modulo sono fra i suoi parametri. Nessuna tabella
     da aggiornare quando si spostano funzioni o costanti;
  3. A MANO (METHODS, METHOD_PARAMS): i metodi chiamati sugli OGGETTI (local_grid.xxx(),
     prm.xxx()) e i loro argomenti, che un controllo statico non puo' attribuire a una classe.

Uscita 0 = tutto allineato. Uscita 1 = manca qualcosa, NON lanciare la missione.
"""

import ast
import os
import sys

# I file che la missione importa (direttamente o no).
MISSION_FILES = [
    "easy_walk.py", "parametri.py", "registro.py", "spotGrid.py", "navGraphUtils.py",
    "prm_graph.py", "mission_graph.py", "arcVerification.py", "movements.py",
    "environmentMap.py", "spotUtils.py", "spotLogInUtils.py", "batteria.py",
]

# Metodi chiamati sugli oggetti: (file, classe, metodo).
METHODS = [
    ("spotGrid.py", "GlobalGrid", "is_occupied"),
    ("spotGrid.py", "GlobalGrid", "is_known"),
    ("spotGrid.py", "GlobalGrid", "_set_state"),
    ("spotGrid.py", "GlobalGrid", "mark_hazard"),
    ("spotGrid.py", "GlobalGrid", "segment_hits_hazard"),
    ("spotGrid.py", "GlobalTerrainGrid", "compute_arc_slope_profile"),
    ("spotGrid.py", "LocalGrid", "compute_robot_footprint_mask"),
    ("spotGrid.py", "LocalGrid", "correct_terrain"),
    ("spotGrid.py", "LocalGrid", "_cells_above_ground"),
    ("spotGrid.py", "LocalGrid", "_cells_never_written"),
    ("spotGrid.py", "LocalGrid", "_fetch_layers"),
    ("spotGrid.py", "LocalGrid", "_layer_geometry"),
    ("spotGrid.py", "LocalGrid", "fuse_obstacle_mask"),
    ("spotGrid.py", "LocalGrid", "extra_layers"),
    ("spotGrid.py", "LocalGrid", "return_local_grid"),
    ("spotGrid.py", "LocalGrid", "compute_gradient_and_roughness"),
    ("mission_graph.py", "MissionGraph", "into_prm"),
    ("mission_graph.py", "MissionGraph", "add_edges_to_prm"),
    ("mission_graph.py", "MissionGraph", "as_sampler"),
    ("mission_graph.py", "MissionGraph", "save_npz"),
    ("mission_graph.py", "MissionGraph", "report"),
    ("mission_graph.py", "MissionGraph", "describe"),
    ("mission_graph.py", "LatticeSampler", "get_point_in_cell"),
    ("navGraphUtils.py", "RecordingInterface", "close_loops_checked"),
    ("navGraphUtils.py", "RecordingInterface", "create_edge_between_waypoint_objs"),
    ("navGraphUtils.py", "RecordingInterface", "invalidate_graph_cache"),
    ("navGraphUtils.py", "RecordingInterface", "_get_graph"),
    ("prm_graph.py", "PRM", "refresh_local_edge_weights"),
    ("prm_graph.py", "PRM", "sample_terrain_between_points"),
    ("prm_graph.py", "PRM", "_compute_edge_weight"),
    ("prm_graph.py", "PRM", "set_global_terrain_map"),
    ("prm_graph.py", "PRM", "add_stop_node"),
    ("prm_graph.py", "PRM", "_evaluate_and_add_edge"),
    ("prm_graph.py", "PRM", "set_robot_node"),
    ("prm_graph.py", "PRM", "_occupancy_blocked"),
    ("prm_graph.py", "PRM", "_global_slope"),
    ("prm_graph.py", "PRM", "_sync_global_slope_cache"),
    ("prm_graph.py", "PRM", "_traversed_edge_ok"),
    ("prm_graph.py", "PRM", "anchor_node"),
    ("prm_graph.py", "PRM", "set_allowed_nodes"),
    ("prm_graph.py", "PRM", "set_detour_limit"),
    ("prm_graph.py", "PRM", "path_length"),
    ("prm_graph.py", "PRM", "update_local_grid_data"),
    ("prm_graph.py", "PRM", "compute_path_cost"),
    ("arcVerification.py", "ArcVerificationTracker", "update_path"),
    ("arcVerification.py", "ArcVerificationTracker", "get_frontier"),
    ("arcVerification.py", "ArcVerificationTracker", "clear_path"),
]

# Argomenti con nome passati a quei metodi: (file, classe, metodo, parametro).
METHOD_PARAMS = [
    ("spotGrid.py", "LocalGrid", "correct_terrain", "cell_size"),
    ("spotGrid.py", "LocalGrid", "correct_terrain", "unwritten_value"),
    ("spotGrid.py", "LocalGrid", "correct_terrain", "return_unwritten"),
    ("spotGrid.py", "LocalGrid", "compute_gradient_and_roughness", "is_valid"),
    ("spotGrid.py", "LocalGrid", "return_local_grid", "max_attempts"),
    ("prm_graph.py", "PRM", "add_stop_node", "is_robot"),
    ("prm_graph.py", "PRM", "update_local_grid_data", "valid_2d"),
    ("prm_graph.py", "PRM", "compute_path_cost", "allow_direct_first_hop"),
    ("navGraphUtils.py", "RecordingInterface", "close_loops_checked", "upload_result"),
    ("navGraphUtils.py", "RecordingInterface", "close_loops_checked", "prune_edges"),
    ("navGraphUtils.py", "RecordingInterface", "close_loops_checked", "icp_iters"),
]


def _parse(path):
    with open(path, encoding="utf-8") as f:
        return ast.parse(f.read(), filename=path)


def _top_names(tree):
    """Nomi definiti a livello di modulo (anche dentro if/try/with) e importati."""
    names, funcs, classes = set(), {}, {}

    def scan(body):
        for node in body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                names.add(node.name)
                funcs[node.name] = node
            elif isinstance(node, ast.ClassDef):
                names.add(node.name)
                classes[node.name] = {n.name: n for n in node.body
                                      if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
            elif isinstance(node, (ast.Import, ast.ImportFrom)):
                names.update((a.asname or a.name).split('.')[0] for a in node.names if a.name != '*')
            elif isinstance(node, (ast.If, ast.Try, ast.With)):
                for part in ('body', 'orelse', 'finalbody'):
                    scan(getattr(node, part, []) or [])
                for h in getattr(node, 'handlers', []) or []:
                    scan(h.body)
            for t in ast.walk(node) if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)) else []:
                if isinstance(t, ast.Name) and isinstance(t.ctx, ast.Store):
                    names.add(t.id)
    scan(tree.body)
    return names, funcs, classes


def _params(fn):
    a = fn.args
    names = [p.arg for p in list(getattr(a, 'posonlyargs', [])) + list(a.args) + list(a.kwonlyargs)]
    return set(names), a.kwarg is not None


def _locals(fn):
    names = _params(fn)[0]
    if fn.args.vararg:
        names.add(fn.args.vararg.arg)
    for x in ast.walk(fn):
        if isinstance(x, ast.Name) and isinstance(x.ctx, ast.Store):
            names.add(x.id)
    return names


def _cross_module(fname, tree, info):
    """Problemi nei riferimenti di `fname` verso gli altri moduli del progetto."""
    ours = {os.path.splitext(f)[0]: f for f in info}
    alias, problems = {}, []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                if a.name in ours:
                    alias[a.asname or a.name] = a.name
        elif isinstance(node, ast.ImportFrom) and node.module in ours:
            names = info[ours[node.module]][0]
            for a in node.names:
                if a.name != '*' and a.name not in names:
                    problems.append(f"{fname}:{node.lineno}: 'from {node.module} import {a.name}' "
                                    f"-- {a.name} non esiste in {ours[node.module]}")

    def visit(node, shadow):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            shadow = shadow | _locals(node)
        target = None
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
                and node.value.id in alias and node.value.id not in shadow:
            mod = alias[node.value.id]
            names, funcs, _ = info[ours[mod]]
            if node.attr not in names:
                problems.append(f"{fname}:{node.lineno}: {node.value.id}.{node.attr} non esiste in {mod}.py")
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and isinstance(node.func.value, ast.Name) and node.func.value.id in alias \
                and node.func.value.id not in shadow:
            mod = alias[node.func.value.id]
            fn = info[ours[mod]][1].get(node.func.attr)
            if fn is not None:
                params, has_kwargs = _params(fn)
                for kw in node.keywords:
                    if kw.arg and kw.arg not in params and not has_kwargs:
                        problems.append(f"{fname}:{node.lineno}: {mod}.{node.func.attr}() non accetta "
                                        f"'{kw.arg}'")
        for c in ast.iter_child_nodes(node):
            visit(c, shadow)

    visit(tree, frozenset())
    return problems


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    print(f"Controllo pre-volo su: {root}\n")

    problems, trees, info = [], {}, {}
    for fname in sorted(f for f in os.listdir(root) if f.endswith('.py')):
        try:
            trees[fname] = _parse(os.path.join(root, fname))
            info[fname] = _top_names(trees[fname])
        except SyntaxError as e:
            if fname in MISSION_FILES:
                problems.append(f"{fname}: ERRORE DI SINTASSI riga {e.lineno}: {e.msg}")
    for fname in MISSION_FILES:
        if not os.path.isfile(os.path.join(root, fname)):
            problems.append(f"{fname}: FILE MANCANTE")

    for fname in MISSION_FILES:
        if fname not in trees:
            continue
        found = _cross_module(fname, trees[fname], info)
        problems += found
        if not found:
            print(f"  OK   {fname}")

    for fname, cls, meth in METHODS:
        if fname not in info:
            continue
        classes = info[fname][2]
        if cls not in classes:
            problems.append(f"{fname}: manca la classe {cls}")
        elif meth not in classes[cls]:
            problems.append(f"{fname}: manca il metodo {cls}.{meth}() -- copia vecchia")
    for fname, cls, meth, param in METHOD_PARAMS:
        fn = info.get(fname, (None, None, {}))[2].get(cls, {}).get(meth)
        if fn is not None:
            params, has_kwargs = _params(fn)
            if param not in params and not has_kwargs:
                problems.append(f"{fname}: {cls}.{meth}() non accetta '{param}' -- copia vecchia")

    print()
    if problems:
        print("=" * 70)
        print("NON LANCIARE LA MISSIONE. Problemi trovati:")
        print("=" * 70)
        for p in problems:
            print(f"  - {p}")
        print("\nRicopia i file segnalati dalla versione piu' recente.")
        return 1

    print("=" * 70)
    print("Tutti i file sono allineati fra loro.")
    print("=" * 70)
    print("NB: questo controllo verifica solo che i moduli siano coerenti.")
    print("    Non dice nulla sul comportamento del robot sul campo.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
