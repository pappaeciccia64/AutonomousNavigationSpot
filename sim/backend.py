"""
SimBackend: what a simulator must provide to run the mission (see SIMULATOR.md).

Frame: one fixed world frame of the simulator, used as Spot's VISION (x, y on the ground,
z up, yaw from x towards y). Units: m, rad, seconds of the simulator clock (now()).

The mission sends paths as Spot's SE2 trajectories: points (x, y, yaw) each with the time at
which the robot should be there, plus speed limits. The adapter (spot_adapter.SimSpot) turns
them into follow_path() calls and reads the progress back with path_status().
"""


class SimBackend:
    """Subclass and implement every method. KinematicWorld is a complete example."""

    # --- clock -------------------------------------------------------------------------
    def now(self):
        """Seconds of the simulator clock."""
        raise NotImplementedError

    def step_to(self, t):
        """Advance the simulation up to time t (a simulator running on its own may only
        wait until its clock reaches t)."""
        raise NotImplementedError

    # --- robot -------------------------------------------------------------------------
    def pose(self):
        """(x, y, z, yaw, roll, pitch) of the body; z is the body height in the world."""
        raise NotImplementedError

    def fallen(self):
        """True if the robot is on the ground (it ends the cruise and triggers the recovery)."""
        return False

    def follow_path(self, points, max_linear, max_angular):
        """Follow a path: points = [(x, y, yaw, t), ...] with t on the simulator clock;
        max_linear in m/s, max_angular in rad/s. Replaces the path being followed."""
        raise NotImplementedError

    def stop(self):
        """Stop where the robot is."""
        raise NotImplementedError

    def path_status(self):
        """'moving', 'near_goal' (within ~0.3 m), 'at_goal', 'blocked' (the robot cannot go
        on: obstacle, too steep) or 'stopped'."""
        raise NotImplementedError

    def teleport(self, x, y, yaw):
        """Put the robot at (x, y, yaw) on the ground: used by the in-memory GraphNav, whose
        trips are not simulated."""
        raise NotImplementedError

    # --- world around the robot ----------------------------------------------------------
    def terrain(self, xs, ys):
        """Ground height (m) at the points (arrays): what the robot's sensors would measure;
        NaN where the ground is not seen."""
        raise NotImplementedError

    def obstacles(self, xs, ys):
        """True where there is an obstacle (something the robot must not walk into: walls,
        boxes, bushes) at the points (arrays)."""
        raise NotImplementedError

    def seen(self, xs, ys, robot_xy):
        """True where the sensors see the ground from robot_xy (occlusions, range). The
        default: everything inside the local grid is seen."""
        import numpy as np
        return np.ones(np.shape(xs), dtype=bool)
