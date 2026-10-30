"""Per-leg gait thresholds (PLAIN / MODERATE / HARD), see robot.motion.select_mobility_params."""

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
