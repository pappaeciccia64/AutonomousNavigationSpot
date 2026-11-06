"""The mission graph during the mission: PRM construction, edge update, cells allowed for
planning, reachability of a cell (see planning/mission_graph.py for the graph itself)."""
import numpy as np

from spot_explorer import config as P
from spot_explorer.perception import geometry
from spot_explorer.planning import roadmap
from spot_explorer.planning.mission_graph import build_mission_graph


# =========================================================================================
# THE GRAPH DURING THE MISSION: PRM construction, edge update, cells
# allowed for planning
# =========================================================================================
def build_mission_prm(env, verbose=True):
    """
    Create the mission graph (lattice or random nodes, see MISSION_GRAPH_SAMPLING) and the
    PRM with its nodes, from the mission size and the starting pose `env` already holds
    (set_origin must be done BEFORE).

    Returns (prm, sampler, mission_lattice): `sampler` gives the nodes cell by cell (see
    mission_graph.LatticeSampler), `mission_lattice` is the MissionGraph.
    The EDGES are not created here: they need the terrain data, see rebuild_prm_edges.
    """
    mg = build_mission_graph(
        env=env, spacing_m=P.MISSION_LATTICE_SPACING_M, rings=P.MISSION_LATTICE_RINGS,
        min_edge_length=P.PRM_MIN_EDGE_M, max_edge_length=P.PRM_MAX_EDGE_M,
        connection_radius=P.PRM_CONNECTION_RADIUS_M, verbose=verbose,
        sampling=P.MISSION_GRAPH_SAMPLING, distribution=P.MISSION_RANDOM_DISTRIBUTION,
        density_per_m2=P.MISSION_RANDOM_DENSITY, seed=P.MISSION_RANDOM_SEED)
    prm = roadmap.PRM(min_edge_length=P.PRM_MIN_EDGE_M, max_edge_length=mg.reach_m,
                      connection_radius=P.PRM_CONNECTION_RADIUS_M)
    prm.mission_graph = mg          # rebuild_prm_edges reads it again
    prm.set_detour_limit(P.PLAN_MAX_DETOUR_RATIO, P.PLAN_DETOUR_SLACK_M)
    mg.into_prm(prm)
    return prm, mg.as_sampler(), mg


def node_cells(prm, env):
    """
    {node id: (row, column)}. From the lattice when there is one (mission_graph.node_cell,
    same rule as env.get_cell_from_world), otherwise from env. Cached on the PRM: the nodes
    do not change during the mission.
    """
    cache = getattr(prm, '_node_cell_cache', None)
    if cache is None:
        cache = {}
        prm._node_cell_cache = cache
    mg = getattr(prm, 'mission_graph', None)
    for nid, (x, y) in prm.nodes.items():
        if nid in cache:
            continue
        if mg is not None and 0 <= nid < mg.n_nodes:
            r, c = mg.node_cell[nid]
            cache[nid] = (int(r), int(c))
        else:
            cache[nid] = tuple(int(v) for v in env.get_cell_from_world(x, y))
    return cache


def graph_reaches_cell(prm, env, from_xy, row, col, global_map=None):
    """
    From the point from_xy (the waypoint the cell would be attempted from) does the graph
    reach at least one node of cell (row, col)? Same rules as the attempt: with
    PLAN_ONLY_EXPLORED_CELLS it passes only through the visited cells, the starting cell and
    the goal cell; edges and nodes as they are now (a node with infinite penalty does not
    count). It is a NECESSARY condition: if False the attempt would surely fail with "GRAPH
    DISCONNECTED"; if True the attempt decides (goal point, detour cap, frontier).
    Breadth-first search, no effect on the graph. None if the graph has no nodes.
    """
    reach = _reach_from(prm, env, from_xy, row, col, global_map)
    if reach is None:
        return None
    seen, cells = reach
    target = (int(row), int(col))
    return any(cells.get(u) == target and np.isfinite(prm.node_penalty(u)) for u in seen)


def unreached_cell_reason(prm, env, from_xy, row, col, global_map=None):
    """Why graph_reaches_cell is False, for the [PRECHECK] line: the cell has no usable node
    (all inside obstacles), or the edges entering it from the reachable area are all out
    of the graph (vetoed by obstacles or slope, even if the inside of the cell was never
    seen: 03-11 11:09, cells behind the wall of the room)."""
    reach = _reach_from(prm, env, from_xy, row, col, global_map)
    if reach is None:
        return "graph without nodes"
    seen, cells = reach
    target = (int(row), int(col))
    in_cell = [u for u, c in cells.items() if c == target]
    usable = [u for u in in_cell if np.isfinite(prm.node_penalty(u))]
    if not usable:
        return f"all the {len(in_cell)} nodes of the cell are discarded as inside obstacles"
    reached = sum(1 for u in seen if cells.get(u) != target)
    return (f"no edge of the graph links the {reached} nodes reachable from there to the "
            f"{len(usable)} usable nodes of the cell: the edges in between are vetoed "
            f"(obstacle or slope on the way)")


def _reach_from(prm, env, from_xy, row, col, global_map):
    """(nodes reachable from from_xy, cell of every node) with the rules of
    graph_reaches_cell; None if the graph has no nodes."""
    start = prm.anchor_node(float(from_xy[0]), float(from_xy[1]), global_map,
                            geometry.PRM_EDGE_SAFETY_MARGIN_M,
                            ignore_near_m=roadmap.ANCHOR_IGNORE_NEAR_M, connected_first=True)
    if start is None:
        return None
    cells = node_cells(prm, env)
    ok_cells = planning_cells(env, row, col, from_xy) if P.PLAN_ONLY_EXPLORED_CELLS else None
    seen, queue = {start}, [start]
    while queue:
        u = queue.pop()
        for v, _w in prm.edges.get(u, ()):
            if v in seen:
                continue
            if ok_cells is not None and cells.get(v) not in ok_cells:
                continue
            seen.add(v)
            queue.append(v)
    return seen, cells


# The cells next to both the robot's and the goal's are allowed too (see planning_cells).
PLAN_ALSO_SHARED_NEIGHBOUR_CELLS = True


def planning_cells(env, target_row, target_col, from_xy):
    """
    The cells a path to (target_row, target_col) from from_xy may use (PLAN_ONLY_EXPLORED_CELLS):
    the visited ones, the robot's, the goal's, and the cells next to both the robot's and the
    goal's (8-neighbourhood): the way around an obstacle on the border between the two
    cells. On 04-11 11:38 three cells out of eight were disconnected only by the cells
    around the border, and the cruise ended eight times in a row on (1,3); the path is
    checked live by the frontier anyway.
    """
    target = (int(target_row), int(target_col))
    here = tuple(int(v) for v in env.get_cell_from_world(from_xy[0], from_xy[1]))
    cells = {(r, c) for r in range(env.rows) for c in range(env.cols) if env.is_cell_visited(r, c)}
    cells |= {target, here}
    if not PLAN_ALSO_SHARED_NEIGHBOUR_CELLS:
        return cells

    def around(rc):
        return {(rc[0] + dr, rc[1] + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1)}
    cells |= {rc for rc in around(target) & around(here)
              if 0 <= rc[0] < env.rows and 0 <= rc[1] < env.cols}
    return cells


def restrict_planning_to_explored_cells(prm, env, target_row, target_col, robot_xy):
    """
    Limit the path search to the nodes of the visited cells, of the robot's cell and of the
    goal cell (see PLAN_ONLY_EXPLORED_CELLS). To be called before every cell attempt: the
    visited cells change from one attempt to the next.
    """
    if not P.PLAN_ONLY_EXPLORED_CELLS:
        prm.set_allowed_nodes(None)
        return
    cells = planning_cells(env, target_row, target_col, robot_xy)
    allowed = [nid for nid, cell in node_cells(prm, env).items() if cell in cells]
    prm.set_allowed_nodes(allowed)
    print(f"[PLAN] Paths allowed only in cells {sorted(cells)}: {len(allowed)} nodes out of "
          f"{len(prm.nodes)}. If a non-visited cell must be crossed, I give up this "
          f"side and move to another cell.")


def robot_on_node(prm, node_id, x, y):
    """True if the robot is practically on the node (see roadmap.ANCHOR_ON_NODE_M)."""
    pos = prm.get_node_position(node_id)
    return pos is not None and float(np.hypot(pos[0] - x, pos[1] - y)) <= roadmap.ANCHOR_ON_NODE_M


def rebuild_prm_edges(prm, global_map, incremental=True):
    """
    Re-evaluate the PRM edges on the current data, on the lattice pairs already computed
    (see mission_graph.MissionGraph.add_edges_to_prm). Vetoes and weights are computed by
    PRM._evaluate_and_add_edge. incremental=False forces the full build (same result:
    test_replay.py checks it).
    """
    return prm.mission_graph.add_edges_to_prm(
        prm, global_map=global_map,
        edge_safety_margin=geometry.PRM_EDGE_SAFETY_MARGIN_M,
        incremental=incremental)
