"""The mission graph during the mission: PRM construction, edge update, cells allowed for
planning, reachability of a cell (see planning/mission_graph.py for the graph itself)."""

from spot_explorer import config as P
from spot_explorer.perception import geometry
from spot_explorer.planning import roadmap
from spot_explorer.planning.mission_graph import build_mission_graph


# =========================================================================================
# The graph during the mission: PRM construction, edge update, cells
# allowed for planning
# =========================================================================================
def build_mission_prm(env, verbose=True):
    """
    Create the mission graph (lattice or random nodes, see MISSION_GRAPH_SAMPLING) and the
    PRM with its nodes, from the mission size and the starting pose `env` already holds
    (set_origin must be done before).

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




def planning_cells(env, target_row, target_col, from_xy):
    """
    The cells a path to (target_row, target_col) from from_xy may use: the visited ones, the
    robot's, the goal's, and the cells next to both the robot's and the goal's
    (8-neighbourhood): the way around an obstacle on the border between the two cells. Cells
    never explored are not crossed: if the only way goes through one, the attempt fails and the
    cell rules try the cell again later. None = every cell, always with the frontier goals
    (env.goal_mode, mission/frontier.py), whose paths may cross any cell.
    """
    if getattr(env, 'goal_mode', 'cells') == 'frontier':
        return None
    target = (int(target_row), int(target_col))
    here = tuple(int(v) for v in env.get_cell_from_world(from_xy[0], from_xy[1]))
    cells = {(r, c) for r in range(env.rows) for c in range(env.cols) if env.is_cell_visited(r, c)}
    cells |= {target, here}

    def around(rc):
        return {(rc[0] + dr, rc[1] + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1)}
    cells |= {rc for rc in around(target) & around(here)
              if 0 <= rc[0] < env.rows and 0 <= rc[1] < env.cols}
    return cells


def restrict_planning_to_explored_cells(prm, env, target_row, target_col, robot_xy):
    """
    Limit the path search to the nodes of the visited cells, of the robot's cell and of the
    goal cell (see planning_cells). To be called before every cell attempt: the visited cells
    change from one attempt to the next.
    """
    cells = planning_cells(env, target_row, target_col, robot_xy)
    if cells is None:
        prm.set_allowed_nodes(None)
        return
    allowed = [nid for nid, cell in node_cells(prm, env).items() if cell in cells]
    prm.set_allowed_nodes(allowed)
    print(f"[PLAN] Paths allowed only in cells {sorted(cells)}: {len(allowed)} nodes out of "
          f"{len(prm.nodes)}. If a non-visited cell must be crossed, I give up this "
          f"side and move to another cell.")


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
