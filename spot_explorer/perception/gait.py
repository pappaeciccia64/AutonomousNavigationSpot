"""Soglie dell'andatura per tratto (PLAIN / MODERATE / HARD), vedi robot.motion.select_mobility_params."""

# =========================================================================
# SCELTA DELL'ANDATURA (vedi movements.select_mobility_params)
# Tre livelli dalla pendenza (col corpo) e dalla rugosita' del tratto da percorrere; decide
# la peggiore delle due. In altezza di gradino netto (H / 0.48): PLAIN sotto ~7 cm, HARD da
# ~12 cm. Sono soglie di "quanta cura", molto sotto quelle di ostacolo.
# =========================================================================
GAIT_PLAIN_SLOPE_MAX = 0.15     # below this on BOTH -> PLAIN tier (flat ground)
GAIT_PLAIN_ROUGH_MAX = 0.05
GAIT_MODERATE_SLOPE_MAX = 0.25  # below this on BOTH -> MODERATE tier; at/above -> HARD tier
GAIT_MODERATE_ROUGH_MAX = 0.10

# ground_mu_hint per livello (bosdyn.api.spot.TerrainParams): 0.8 e' il default di Spot (buona
# aderenza); valori piu' bassi gli dicono di aspettarsi meno attrito e appoggiare con piu' cura.
GAIT_MU_PLAIN = 0.8
GAIT_MU_MODERATE = 0.6
GAIT_MU_HARD = 0.4

# Velocita' massime per livello (m/s lineare, rad/s rotazione), pensate per l'esterno. Vedi
# movements.DEFAULT_MAX_LINEAR_VEL_MPS per l'avvertenza su min/max.
GAIT_VEL_PLAIN = 0.5
GAIT_VEL_MODERATE = 0.35
GAIT_VEL_HARD = 0.25
GAIT_ANG_PLAIN = 0.6
GAIT_ANG_MODERATE = 0.5
GAIT_ANG_HARD = 0.4
