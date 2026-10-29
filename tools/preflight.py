#!/usr/bin/env python3
"""
Controllo pre-volo: verifica che i file sulla macchina del robot siano allineati fra
loro PRIMA di lanciare una missione.

Se sul robot `easy_walk.py` e' aggiornato ma un altro modulo e' una copia vecchia, la
missione si ferma con un AttributeError appena il robot e' acceso e in piedi: tempo perso,
e in esterno tempo perso a batteria accesa in mezzo a un campo. Qui lo si scopre prima.

Il controllo e' PURAMENTE STATICO: legge i file, non importa nulla, non tocca il robot,
non serve l'SDK bosdyn. Si puo' lanciare in qualunque momento.

    python3 tools/preflight.py                 # il progetto che contiene questo file
    python3 tools/preflight.py /percorso/cartella

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

# I moduli della missione: tutti quelli del pacchetto spot_explorer (vedi _moduli).
PACKAGE = "spot_explorer"

# Metodi chiamati sugli oggetti: (modulo, classe, metodo).
METHODS = [
    ("spot_explorer.perception.global_maps", "GlobalGrid", "is_occupied"),
    ("spot_explorer.perception.global_maps", "GlobalGrid", "is_known"),
    ("spot_explorer.perception.global_maps", "GlobalGrid", "_set_state"),
    ("spot_explorer.perception.global_maps", "GlobalGrid", "mark_hazard"),
    ("spot_explorer.perception.global_maps", "GlobalGrid", "segment_hits_hazard"),
    ("spot_explorer.perception.global_maps", "GlobalTerrainGrid", "compute_arc_slope_profile"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "compute_robot_footprint_mask"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "correct_terrain"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "_cells_above_ground"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "_cells_never_written"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "_fetch_layers"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "_layer_geometry"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "fuse_obstacle_mask"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "extra_layers"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "return_local_grid"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "compute_gradient_and_roughness"),
    ("spot_explorer.planning.mission_graph", "MissionGraph", "into_prm"),
    ("spot_explorer.planning.mission_graph", "MissionGraph", "add_edges_to_prm"),
    ("spot_explorer.planning.mission_graph", "MissionGraph", "as_sampler"),
    ("spot_explorer.planning.mission_graph", "MissionGraph", "save_npz"),
    ("spot_explorer.planning.mission_graph", "MissionGraph", "report"),
    ("spot_explorer.planning.mission_graph", "MissionGraph", "describe"),
    ("spot_explorer.planning.mission_graph", "LatticeSampler", "get_point_in_cell"),
    ("spot_explorer.graphnav.graphnav", "RecordingInterface", "close_loops_checked"),
    ("spot_explorer.graphnav.graphnav", "RecordingInterface", "create_edge_between_waypoint_objs"),
    ("spot_explorer.graphnav.graphnav", "RecordingInterface", "invalidate_graph_cache"),
    ("spot_explorer.graphnav.graphnav", "RecordingInterface", "_get_graph"),
    ("spot_explorer.planning.roadmap", "PRM", "refresh_local_edge_weights"),
    ("spot_explorer.planning.roadmap", "PRM", "sample_terrain_between_points"),
    ("spot_explorer.planning.roadmap", "PRM", "_compute_edge_weight"),
    ("spot_explorer.planning.roadmap", "PRM", "set_global_terrain_map"),
    ("spot_explorer.planning.roadmap", "PRM", "add_stop_node"),
    ("spot_explorer.planning.roadmap", "PRM", "_evaluate_and_add_edge"),
    ("spot_explorer.planning.roadmap", "PRM", "set_robot_node"),
    ("spot_explorer.planning.roadmap", "PRM", "_occupancy_blocked"),
    ("spot_explorer.planning.roadmap", "PRM", "_global_slope"),
    ("spot_explorer.planning.roadmap", "PRM", "_sync_global_slope_cache"),
    ("spot_explorer.planning.roadmap", "PRM", "_traversed_edge_ok"),
    ("spot_explorer.planning.roadmap", "PRM", "anchor_node"),
    ("spot_explorer.planning.roadmap", "PRM", "set_allowed_nodes"),
    ("spot_explorer.planning.roadmap", "PRM", "set_detour_limit"),
    ("spot_explorer.planning.roadmap", "PRM", "path_length"),
    ("spot_explorer.planning.roadmap", "PRM", "update_local_grid_data"),
    ("spot_explorer.planning.roadmap", "PRM", "compute_path_cost"),
    ("spot_explorer.planning.safe_frontier", "ArcVerificationTracker", "update_path"),
    ("spot_explorer.planning.safe_frontier", "ArcVerificationTracker", "get_frontier"),
    ("spot_explorer.planning.safe_frontier", "ArcVerificationTracker", "clear_path"),
]

# Argomenti con nome passati a quei metodi: (modulo, classe, metodo, parametro).
METHOD_PARAMS = [
    ("spot_explorer.perception.spot_grid", "LocalGrid", "correct_terrain", "cell_size"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "correct_terrain", "unwritten_value"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "correct_terrain", "return_unwritten"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "compute_gradient_and_roughness", "is_valid"),
    ("spot_explorer.perception.spot_grid", "LocalGrid", "return_local_grid", "max_attempts"),
    ("spot_explorer.planning.roadmap", "PRM", "add_stop_node", "is_robot"),
    ("spot_explorer.planning.roadmap", "PRM", "update_local_grid_data", "valid_2d"),
    ("spot_explorer.planning.roadmap", "PRM", "compute_path_cost", "allow_direct_first_hop"),
    ("spot_explorer.graphnav.graphnav", "RecordingInterface", "close_loops_checked", "upload_result"),
    ("spot_explorer.graphnav.graphnav", "RecordingInterface", "close_loops_checked", "prune_edges"),
    ("spot_explorer.graphnav.graphnav", "RecordingInterface", "close_loops_checked", "icp_iters"),
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
    ours = {m: m for m in info}
    alias, problems = {}, []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            for a in node.names:
                full = f"{node.module}.{a.name}"
                if full in ours:                     # from spot_explorer.x import modulo as alias
                    alias[a.asname or a.name] = full
                elif node.module in ours and a.name != '*' and a.name not in info[node.module][0]:
                    problems.append(f"{fname}:{node.lineno}: 'from {node.module} import {a.name}' "
                                    f"-- {a.name} non esiste in {node.module}")

    def visit(node, shadow):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            shadow = shadow | _locals(node)
        target = None
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
                and node.value.id in alias and node.value.id not in shadow:
            mod = alias[node.value.id]
            names, funcs, _ = info[mod]
            if node.attr not in names:
                problems.append(f"{fname}:{node.lineno}: {node.value.id}.{node.attr} non esiste in {mod}")
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and isinstance(node.func.value, ast.Name) and node.func.value.id in alias \
                and node.func.value.id not in shadow:
            mod = alias[node.func.value.id]
            fn = info[mod][1].get(node.func.attr)
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


def _moduli(root):
    """{nome del modulo con i punti: percorso del file} per tutto il pacchetto."""
    out = {}
    for dp, _, fs in os.walk(os.path.join(root, PACKAGE)):
        for f in fs:
            if f.endswith('.py'):
                rel = os.path.relpath(os.path.join(dp, f), root)[:-3].replace(os.sep, '.')
                out[rel[:-len('.__init__')] if rel.endswith('.__init__') else rel] = os.path.join(dp, f)
    return out


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    print(f"Controllo pre-volo su: {root}\n")

    problems, trees, info = [], {}, {}
    files = _moduli(root)
    if not files:
        problems.append(f"{PACKAGE}/: PACCHETTO MANCANTE")
    for mod, path in sorted(files.items()):
        try:
            trees[mod] = _parse(path)
            info[mod] = _top_names(trees[mod])
        except SyntaxError as e:
            problems.append(f"{mod}: ERRORE DI SINTASSI riga {e.lineno}: {e.msg}")

    for mod in sorted(trees):
        found = _cross_module(mod, trees[mod], info)
        problems += found
        if not found:
            print(f"  OK   {mod}")

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
