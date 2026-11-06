"""Per-leg gait thresholds (PLAIN / MODERATE / HARD), see robot.motion.select_mobility_params."""
from spot_explorer.perception import slopes

# =========================================================================
# GAIT SELECTION (see motion.select_mobility_params)
# Three levels from the slope (with the body) and the roughness of the leg ahead; the worse
# of the two decides. As a net step height (H / 0.48): PLAIN below ~7 cm, HARD from
# ~12 cm. These are "how careful" thresholds, well below the obstacle ones.
# =========================================================================
GAIT_PLAIN_SLOPE_MAX = 0.15     # below this on BOTH -> PLAIN tier (flat ground)
GAIT_PLAIN_ROUGH_MAX = 0.05
GAIT_MODERATE_SLOPE_MAX = 0.25  # below this on BOTH -> MODERATE tier; at/above -> HARD tier
GAIT_MODERATE_ROUGH_MAX = 0.10
# The slope thresholds above are on today's estimate (differences on a base, with the body).
# With slopes.PLANE_SLOPES the plane reads lower (tools/plane_thresholds.py, 1382 legs of 51
# missions: +4.5 deg against +1.5 deg over the median body pitch on the slopes), so the same
# thresholds would put fewer legs in CRAWL: 9% HARD instead of 18%. These keep the inclined
# legs caught as today (measured pitch > 8.5 deg: 99%; > 14 deg: 94% against 95%) with fewer
# useless MODERATE legs (false alarms 18% against 26%).
PLANE_GAIT_PLAIN_SLOPE_MAX = 0.13
PLANE_GAIT_MODERATE_SLOPE_MAX = 0.20


def slope_thresholds():
    """(PLAIN -> MODERATE, MODERATE -> HARD) slope thresholds of the estimate in use."""
    if slopes.PLANE_SLOPES:
        return PLANE_GAIT_PLAIN_SLOPE_MAX, PLANE_GAIT_MODERATE_SLOPE_MAX
    return GAIT_PLAIN_SLOPE_MAX, GAIT_MODERATE_SLOPE_MAX

# ground_mu_hint per level (bosdyn.api.spot.TerrainParams): 0.8 is Spot's default (good
# grip); lower values tell it to expect less friction and place its feet more carefully.
GAIT_MU_PLAIN = 0.8
GAIT_MU_MODERATE = 0.6
GAIT_MU_HARD = 0.4

# Maximum speeds per level (m/s linear, rad/s rotation), meant for outdoors. See
# motion.DEFAULT_MAX_LINEAR_VEL_MPS for the caveat on min/max.
GAIT_VEL_PLAIN = 0.5
GAIT_VEL_MODERATE = 0.35
GAIT_VEL_HARD = 0.25
GAIT_ANG_PLAIN = 0.6
GAIT_ANG_MODERATE = 0.5
GAIT_ANG_HARD = 0.4
