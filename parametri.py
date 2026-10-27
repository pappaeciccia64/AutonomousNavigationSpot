"""
PARAMETRI DELLA MISSIONE: tutti gli interruttori e i numeri che si cambiano sul campo.

Un solo posto da toccare. Ogni parametro ha accanto cosa fa; perche' ha quel valore (le
prove sul campo che l'hanno deciso) sta in CHANGELOG.md. I parametri propri di un modulo
(griglie e mappe in spotGrid.py, grafo in prm_graph.py, fronte in arcVerification.py,
movimento in movements.py) restano in quel modulo.

Gli argomenti da riga di comando di easy_walk.py (--rows, --grafo, --spacing, ...) cambiano
questi valori per una sola missione. Il codice li legge come parametri.NOME al momento
dell'uso, quindi anche le prove (test_replay.py, test_e2e_loop.py) li possono cambiare.
"""
import numpy as np

import spotGrid

# =========================================================================================
# LA MISSIONE: dimensione, grafo dei nodi, archi
# =========================================================================================
# Righe (a sinistra del robot) x colonne (davanti) x lato della cella. Da qui si costruiscono
# sia la mappa delle celle sia il grafo, quindi restano sempre uguali. Per una sola missione,
# senza toccare il file: argomenti --rows --cols --cell di easy_walk.py, oppure la variabile
# SPOT_MISSION_GRID="righe,colonne,lato".
MISSION_GRID_ROWS = 8
MISSION_GRID_COLS = 7
MISSION_CELL_SIZE_M = 2.0

# Come si generano i nodi del grafo (vedi mission_graph.py, NODI CASUALI):
#   'reticolo' reticolo rettangolare, deterministico; il centro di ogni cella e' un nodo;
#   'casuale'  nodi sparsi con MISSION_RANDOM_DISTRIBUTION ('uniforme', 'poisson',
#              'perturbata', 'halton'), centri delle celle sempre compresi.
# Da riga di comando: --grafo --distribuzione --densita --seme.
MISSION_GRAPH_SAMPLING = 'casuale'
MISSION_RANDOM_DISTRIBUTION = 'poisson'
# Nodi per m2: con 4 il numero di nodi e' quello del reticolo a passo 0.5 m; 5 un po' di
# piu', per i passaggi stretti.
MISSION_RANDOM_DENSITY = 5.0
MISSION_RANDOM_SEED = None              # None = estratto a caso, e scritto nel log
MISSION_POISSON_MIN_DIST_M = None       # None = 0.75 / sqrt(densita')

# Reticolo: passo fra i nodi, arrotondato a cell_size/n (None = mission_graph.
# suggested_spacing()); anelli di vicini collegati (None = tutti i nodi fra PRM_MIN_EDGE_M e
# PRM_MAX_EDGE_M). Da riga di comando: --spacing --rings.
MISSION_LATTICE_SPACING_M = 0.5
MISSION_LATTICE_RINGS = None

# Archi: si collegano tutte le coppie di nodi distanti fra PRM_MIN_EDGE_M e PRM_MAX_EDGE_M.
# Archi lunghi fino a 1.25 m: con archi piu' corti l'ingresso in una cella diventa una catena
# di molti waypoint, e ogni waypoint e' una rotazione, proprio nei corridoi stretti in cui
# ruotare non si puo'. PRM_MAX_EDGE_M e' anche il raggio entro cui il refresh del PRM
# ricostruisce le coppie candidate. Da riga di comando: --min-edge --max-edge.
PRM_MIN_EDGE_M = 0.5
PRM_MAX_EDGE_M = 1.25
PRM_CONNECTION_RADIUS_M = 3.0

# --- Dove puo' passare il pianificatore ---------------------------------------------------
# Per entrare in una cella il percorso puo' usare SOLO le celle gia' visitate, la cella in
# cui si trova il robot e la cella obiettivo: le celle mai esplorate non si attraversano. Se
# l'unica strada passa da una cella non visitata il tentativo fallisce, il lato viene segnato
# come provato e la cella si ritenta piu' avanti da un altro lato.
PLAN_ONLY_EXPLORED_CELLS = True
# Tetto alla deviazione: un percorso piu' lungo di RATIO * linea d'aria + SLACK vale "nessun
# percorso" (vedi PRM.set_detour_limit). None = nessun tetto.
PLAN_MAX_DETOUR_RATIO = 2.0
PLAN_DETOUR_SLACK_M = 1.0


# ==================================================================
# NAVIGAZIONE CON IL FRONTE SICURO -- vedi decide_next_move() e il ciclo
# in attempt_enter_cell_from_position(). Le distanze del fronte stanno in spotGrid
# (FRONTIER_*), qui solo la logica del ciclo.
#   MIN_PARTIAL_STEP_M           -- passo minimo: sotto questa lunghezza non vale la pena
#                                   muoversi (si pagherebbe una scansione per pochi cm).
#   FRONTIER_BLOCK_CONFIRM_SCANS -- quante scansioni consecutive, a robot FERMO, senza poter
#                                   avanzare nemmeno del passo minimo, prima di dichiarare
#                                   un blocco: la conferma "da vicino", invece di
#                                   condannare un arco al primo scorcio da lontano.
#   FRONTIER_RESCAN_WAIT_S       -- pausa fra due scansioni di conferma.
#   MAX_BLOCK_REPLANS            -- dopo un blocco si ripianifica dal punto in cui si e'; se
#                                   succede piu' di queste volte nello stesso tentativo, ci
#                                   si arrende (ritirata, cella rimandata). Lo stesso
#                                   valore limita anche le riscelte del punto obiettivo.
#   FRONTIER_ABORT_CONFIRM       -- a movimento in corso, quanti fronti consecutivi (~0.2 s
#                                   l'uno) devono dire "non arrivi dove stai andando" prima
#                                   di fermare il robot. 2 evita di inchiodare per un singolo
#                                   fotogramma sporco; a 0.5 m/s sono ~20 cm.
#   GOAL_REACHED_TOLERANCE_M     -- se il fronte impedisce gli ultimi centimetri verso
#                                   l'obiettivo (es. un muro appena oltre), ci si accontenta.
#   MAX_LOOP_ITERATIONS          -- salvaguardia anti-stallo del ciclo.
# ==================================================================
MIN_PARTIAL_STEP_M = 0.40
FRONTIER_BLOCK_CONFIRM_SCANS = 3
FRONTIER_RESCAN_WAIT_S = 0.3
MAX_BLOCK_REPLANS = 3
FRONTIER_ABORT_CONFIRM = 2
FRONTIER_ABORT_TOLERANCE_M = 0.05

# Legata allo scostamento che il fronte stesso impone: allowed_advance() ferma il CENTRO del
# robot a FRONTIER_BODY_HALF_LENGTH_M + FRONTIER_MARGIN_M = 0.65 m prima del primo punto non
# libero. Una tolleranza piu' piccola lascerebbe una banda morta in cui il robot non puo' ne'
# avanzare ne' dichiarare di essere arrivato, e finirebbe per dichiarare un blocco. Le due
# costanti devono essere la stessa cosa.
GOAL_REACHED_TOLERANCE_M = spotGrid.FRONTIER_BODY_HALF_LENGTH_M + spotGrid.FRONTIER_MARGIN_M
MAX_LOOP_ITERATIONS = 150

# --- GUARDIE ANTI-STALLO ------------------------------------------------------------
# Il ciclo puo' girare a vuoto con il robot immobile: la rotazione verso il prossimo nodo
# viene rifiutata, il cono non trova direzioni che avvicinino, ma le direzioni utilizzabili
# non sono zero (quindi non e' "trappola"); si scarta il tratto, si ripianifica, e al giro
# dopo il primo waypoint punta da un'altra parte e la rotazione viene rifiutata di nuovo.
#   MAX_ROTATION_DISCARDS -- scarti di tratto per rifiuto di rotazione ammessi in un
#       tentativo di cella. Contati per TENTATIVO e non per coppia di nodi: la coppia cambia
#       a ogni ripianificazione, e un contatore per coppia non scatterebbe mai (e' il
#       limite di MAX_ROTATION_FAILS_SAME_EDGE).
#   MAX_IDLE_LOOPS -- giri di ciclo consecutivi senza che il robot si sia spostato di
#       almeno IDLE_MOVE_M. E' la rete generica: qualunque sia il ramo che gira a vuoto,
#       dopo tanti giri fermi si rinuncia alla cella in modo ordinato invece di consumare
#       batteria. Deve stare SOPRA la sequenza legittima piu' lunga a robot fermo,
#       altrimenti taglia via proprio le reti di ripiego (ritorno con GraphNav) che quella
#       sequenza deve raggiungere. Calcolata sui limiti del ciclo:
#         [SUBITO]                MAX_EARLY_REPLANS              4
#         obiettivo riscelto      MAX_BLOCK_REPLANS              3
#         conferme di blocco      FRONTIER_BLOCK_CONFIRM_SCANS x (MAX_BLOCK_REPLANS+1) = 3 x 4 = 12
#         rifiuti di rotazione    MAX_ROTATION_DISCARDS          4
#         rifiuti d'arrivo        MAX_ARRIVAL_REFUSALS           3
#                                                        totale 26  -> 30
#       A ~0.7 s per giro fermo sono ~20 s. Il giro a vuoto sulla rotazione lo prende
#       prima MAX_ROTATION_DISCARDS; questa e' la rete per tutto il resto.
MAX_ROTATION_DISCARDS = 4
MAX_IDLE_LOOPS = 30
IDLE_MOVE_M = 0.05

# ==================================================================
# ROTAZIONE E ARRETRAMENTO.
#
# Decisione dell'utente: il robot punta SEMPRE il muso verso il punto da raggiungere.
# Niente marcia obliqua. Di traverso Spot e' largo 1.10 m invece di 0.50, quindi il
# fronte pretende 0.59 m di obstacle_distance invece di 0.30: andare di lato quando non
# si riesce a ruotare renderebbe il robot PIU' LARGO proprio dove lo spazio e' poco.
#
# Quando la rotazione non e' possibile il robot ARRETRA IN LINEA RETTA lungo l'asse
# del corpo -- che non e' marcia obliqua: l'ingombro resta 0.50 m e la soglia del
# fronte resta 0.30 -- e al giro dopo riprova da li'. Alla seconda volta sullo stesso
# tratto il tratto si scarta e si ripianifica.
#   ROTATION_RETREAT_M        -- quanto si tenta di arretrare (il fronte puo' concederne meno)
#   ROTATION_RETREAT_MIN_M    -- sotto questo arretramento non vale la pena muoversi
#   MAX_ROTATION_FAILS_SAME_EDGE -- tentativi di arretramento sullo stesso tratto prima di scartarlo
# ==================================================================
ROTATION_RETREAT_M = 0.60
ROTATION_RETREAT_MIN_M = 0.20
MAX_ROTATION_FAILS_SAME_EDGE = 2

# --- Ventaglio dentro il cono di rotazione possibile -- vedi plan_cone_move.
CONE_SWEEP_MAX_DEG = 90.0      # quanto in la' si guarda, a destra e a sinistra del muso
CONE_SWEEP_STEP_DEG = 10.0     # passo della scansione angolare (9 direzioni per lato)
CONE_MIN_GAIN_M = 0.20         # una direzione vale solo se avvicina all'obiettivo di tanto
# Il cono punta al punto del PERCORSO a questa distanza dal robot, non all'obiettivo della
# cella in linea d'aria: se il percorso gira attorno a un blocco, "avvicinarsi" all'obiettivo
# in linea d'aria puo' portare il robot dentro un vicolo cieco, dalla parte opposta al
# percorso. Se quel punto e' dietro, nessuna direzione del cono avvicina, e si passa
# all'arretramento -- che e' la mossa giusta.
CONE_PATH_LOOKAHEAD_M = 1.5
# Durante la rotazione sul posto un fronte "fine dati" piu' corto non interrompe il movimento:
# ruotando la zona vista si sposta e il bordo dei dati si accorcia di qualche centimetro senza
# alcun ostacolo. Vale finche'
# il robot non si e' spostato di almeno tanto; un fronte accorciato da un OSTACOLO interrompe
# sempre.
ABORT_IGNORE_DATA_EDGE_UNTIL_M = 0.15

# Arretramenti consecutivi ammessi, CONTATI PER SEQUENZA e non per arco:
# MAX_ROTATION_FAILS_SAME_EDGE e' indicizzato su (nodo_robot, nodo_successivo), e dopo un
# arretramento il robot puo' trovarsi agganciato a un altro nodo, quindi da solo non ferma
# una catena di arretramenti.
MAX_STRAIGHT_RETREATS_IN_A_ROW = 2

# --- Ventaglio a 360 gradi: riconoscere la trappola -- vedi scan_escape_directions.
ESCAPE_FAN_STEP_DEG = 15.0        # 24 direzioni; 24 valutazioni del fronte, ~frazione di scansione
ESCAPE_FAN_MIN_ADVANCE_M = 0.30   # sotto questo spazio una direzione non e' una via d'uscita
# Controllo preventivo sulla posa d'arrivo (vedi arrival_has_exit): si paga solo dove serve,
# cioe' sui passi corti e verso spazi stretti, che sono i casi in cui si entra negli angoli.
ARRIVAL_EXIT_CHECK_MAX_STEP_M = 0.40
ARRIVAL_EXIT_CHECK_OD_M = 0.50
MAX_ARRIVAL_REFUSALS = 3          # per tentativo di cella, per non avvitarsi sul rifiuto

# --- Ritorni con GraphNav invece che camminando all'indietro ------------------------------
# Tornare indietro a zampe, all'indietro, sui tratti registrati e' la manovra piu' rischiosa
# che facciamo: il robot non vede dove va e non si rilocalizza. GraphNav cammina in avanti,
# con la localizzazione e l'anti-ostacolo di Boston Dynamics. Si lascia una traccia di
# waypoint lungo la strada (vedi graphnav_drop_breadcrumb) e si torna su quella.
GRAPHNAV_WAYPOINT_MIN_SPACING_M = 0.75   # distanza minima fra due briciole
GRAPHNAV_MAX_BACKTRACK_HOPS = 3          # poi si rimanda la cella e si passa alla priorita' dopo

# --- Scorciatoie nel grafo GraphNav -----------------------------------------------------
# Ogni waypoint si collega solo al precedente: il grafo GraphNav e' una CATENA che ricalca la
# strada fatta, e GraphNav cammina solo sugli archi. Senza scorciatoie, tornare a un waypoint
# a un metro di distanza ma registrato molti passi prima vuol dire ripercorrere tutta la
# catena. Prima di ogni spostamento con GraphNav (graphnav_add_shortcuts):
#   1. scorciatoie NOSTRE, scelte SULLA STRADA (vedi plan_graphnav_shortcuts): si calcola la
#      strada piu' corta verso la destinazione considerando TUTTE le coppie di waypoint a
#      meno di GRAPHNAV_SHORTCUT_MAX_M, senza arco diretto, la cui linea retta e' stata
#      VISTA libera nella mappa globale con GRAPHNAV_SHORTCUT_MARGIN_M di margine, e si
#      creano solo quelle che la strada usa. Mai attraverso zone mai viste.
#   2. chiusure d'anello di Spot (fiducial + odometria) con il loro controllo di collisione.
# GraphNav conserva comunque il suo anti-ostacolo durante il movimento.
# GRAPHNAV_SHORTCUT_MARGIN_M: margine dalle celle occupate (che sono gia' a obstacle_distance
# <= 0.15): 0.30 + 0.15 = 0.45 m dall'ostacolo, largo perche' gli archi sono lunghi.
GRAPHNAV_SHORTCUTS = True
GRAPHNAV_SDK_LOOP_CLOSURE = True
# Tempo massimo di elaborazione LATO SPOT della chiusura d'anello (Params.timeout_seconds):
# allo scadere Spot restituisce gli archi trovati fino a li'. 0 = non chiamarla.
#   _BUDGET_S       prima di ogni spostamento GraphNav, in modalita' VELOCE (_FAST: senza ICP
#                   ne' feature matching). Spenta (vedi sotto);
#   _BUDGET_FINAL_S una volta, a fine missione, prima dell'ancoraggio e del ritorno a wp_0,
#                   completa (con ICP).
# Prove al banco del 26-10 sul grafo del 22-10 16:04 ricostruito a 100/200/388 waypoint
# (prova_anelli.py, MissionLogs/prova_anelli_26-10-2026_12-41-53 e _13-02-58):
#   - completa, limite 4 s: 4-5 archi a 100 wp, 1 a 200, 3 a 388, in 5-8 s, mai finita; senza
#     limite Spot si ferma da solo dopo ~30 s, insieme alla RPC (da qui i 20 timeout senza
#     risultato del 22-10); chiamate ripetute non riprendono il lavoro e si allungano;
#   - VELOCE: FINITA in 0.4 s (100 wp, 6 archi), 2.0 s (200 wp, 12), 4.0 s (388 wp, 24). Il
#     costo era tutto nell'ICP. Gli archi non sono corretti sulle scansioni (la posa viene
#     dall'odometria, come per le nostre scorciatoie) ma passano il controllo di collisione
#     di Spot: sono scorciatoie in piu' su tutto il grafo, non solo sulla strada del viaggio;
#   - a fine missione, completa: 45 s -> 17 archi in 59 s, 60 -> 18 in 72 s, 90 -> 21 in 90 s,
#     120 -> 23 in 152 s. Oltre 45 s ~1 arco ogni 10-15 s. Questi archi correggono la deriva
#     della mappa scaricata (ancoraggio).
# Prova all'aperto del 26-10 15:11: veloce 0.0-2.0 s a viaggio, 13 archi; la completa finale
# 2 archi. Serie 3 al banco sul grafo di quella missione: la completa finisce in 62-71 s con
# 0 archi, con o senza prune_edges, con 45 o 90 s. Spot cerca coppie vicine nello spazio ma
# LONTANE lungo il grafo, e gli archi veloci e le nostre scorciatoie quei giri li hanno gia'
# accorciati; gli archi esistenti non li ricontrolla mai. Le due cose si escludono: si tiene la
# completa (archi corretti sulle scansioni, gli unici che correggono la deriva della mappa
# scaricata: 17 in 45 s al banco su un grafo senza archi veloci) e si spegne la veloce. Gli
# spostamenti usano le nostre scorciatoie: il 22-10 23 viaggi riusciti su 23 con quelle sole.
GRAPHNAV_SDK_LOOP_BUDGET_S = 0.0
GRAPHNAV_SDK_LOOP_FAST = True
GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S = 45.0
# prune_edges della chiusura completa finale: None = valore di Spot (True: un solo arco per
# zona). Il 26-10 15:36 la finale ha dato 2 archi: le zone erano gia' collegate dalla veloce e
# dalle scorciatoie, e Spot non ne aggiungeva altri accanto. False le farebbe aggiungere anche
# li', corretti sulle scansioni. Serie 3: nessuna differenza (0 archi in entrambi i casi).
GRAPHNAV_SDK_LOOP_FINAL_PRUNE = None
# Limiti sulla distanza lungo il grafo fra i due waypoint di un anello candidato
# (max/min_loop_closure_path_length). None = valori di Spot. Da scegliere con prova_anelli.py.
GRAPHNAV_SDK_LOOP_MAX_PATH_M = None
GRAPHNAV_SDK_LOOP_MIN_PATH_M = None
GRAPHNAV_SHORTCUT_MAX_M = 3.0
GRAPHNAV_SHORTCUT_MARGIN_M = 0.30
# Lunghezza massima degli archi che crea la chiusura d'anello di Spot.
GRAPHNAV_SDK_LOOP_MAX_M = 2.0
# Un nodo PRM con meno di questo spazio libero non e' raggiungibile dal fronte: si scarta.
PRM_NODE_MIN_CLEARANCE_M = spotGrid.FRONTIER_CLEARANCE_M   # 0.30
# Ripianificazioni "precoci" (alla prima cella bloccante, senza aspettare la conferma da
# ferma) ammesse per tentativo di cella. Vedi il ramo 'wait'.
MAX_EARLY_REPLANS = 4
# Ogni ritorno indietro ha memoria per il tentativo (vedi _trap_backtrack): l'arco su cui il
# robot si e' fermato diventa impercorribile fino alla fine del tentativo, e al
# MAX_BACKTRACKS_PER_ATTEMPT-esimo ritorno il robot torna indietro e rinuncia alla cella da
# questo lato. Senza memoria, dalla briciola precedente il pianificatore ritroverebbe lo
# stesso percorso e il robot rientrerebbe nella stessa trappola.
MAX_BACKTRACKS_PER_ATTEMPT = 2
# Entro questa distanza l'obiettivo del recupero e' "lo stesso" di quello appena fallito.
RECOVERY_SAME_GOAL_M = 0.30

# --- SCELTA DELL'OBIETTIVO DELLA CELLA (vedi find_best_point_in_cell) -----------------
# TARGET_CLEARANCE_M: spazio libero PREFERITO attorno all'obiettivo, sulla mappa globale e
# sulla scansione dal vivo. Il fronte pretende FRONTIER_CLEARANCE_M (0.25) e l'isteresi del
# fronte e' 0.04 m: un obiettivo scelto a 0.28 (PLAN_CLEARANCE_M, che resta il minimo
# assoluto) oscilla attorno alla soglia. All'aperto (22-10 16:04) gli obiettivi scelti a
# 0.28 sono stati rifiutati dal vivo a 0.24, al bordo dell'erba.
TARGET_CLEARANCE_M = 0.33
# TARGET_CELL_BORDER_M: l'obiettivo si sceglie preferibilmente ad almeno questa distanza dal
# bordo della cella. Un obiettivo sul bordo lascia il robot, fermo entro la tolleranza
# d'arrivo, fuori dalla cella ("Mancano 0.06 m ... ancora fuori dalla cella", 44 volte il
# 22-10 16:04).
TARGET_CELL_BORDER_M = 0.30
# Arrivo per tolleranza con il robot appena FUORI dalla cella: vale se il centro e' fuori di
# al massimo ARRIVAL_OUTSIDE_CELL_M (il muso, 0.55 m avanti, e' comunque dentro). E' la
# tolleranza d'arrivo meno il margine dal bordo: un obiettivo scelto a TARGET_CELL_BORDER_M
# dentro la cella, con il robot fermo a GOAL_REACHED_TOLERANCE_M da lui, vale sempre. Il
# 22-10 16:04, a ogni rifiuto "fuori dalla cella" il robot era a 0.5-0.65 m dall'obiettivo,
# fuori di 0.01-0.61 m.
ARRIVAL_OUTSIDE_CELL_M = GOAL_REACHED_TOLERANCE_M - TARGET_CELL_BORDER_M      # = 0.35
# Un obiettivo rifiutato dal fronte (spazio libero dal vivo sotto la soglia, o blocco
# confermato entro questa distanza da lui) resta escluso per la sua cella fino alla fine
# della missione: vedi _refuse_goal.
REFUSED_GOAL_BLOCK_M = 0.35

# ==================================================================
# DATI DI MISSIONE.
#   Durante la missione non si disegna nulla: ogni figura viene salvata come dati in un
#   <nome>_vis.npz (vedi visualize_grid_with_candidates) e le figure si fanno dopo con
#   mission_viewer.py.
#   SAVE_SCAN_BUNDLES -- un .npz per ogni scansione del ciclo di navigazione, nella
#       sottocartella scans/: layer GREZZI come arrivano da Spot (terrain, terrain_valid,
#       obstacle_distance), i derivati usati per decidere, la georeferenziazione (origine,
#       cella, posa del robot, suolo stimato), percorso, fronte e decisione. E' cio' che usano
#       mission_viewer.py, mission_report.py e test_replay.py. ~0.3 MB a scansione.
# ==================================================================
SAVE_SCAN_BUNDLES = True
# Layer di Spot salvati in piu' nei pacchetti, con una richiesta a parte che non tocca la
# navigazione (spotGrid.LocalGrid.extra_layers): per capire l'erba alta (22-10 16:04), cioe'
# se intensita' e no_step distinguono la vegetazione da un ostacolo solido. Ogni layer ha
# accanto <nome>_origin e <nome>_cell, perche' la sua griglia puo' non coincidere con le
# altre. () = spento.
SAVE_EXTRA_LAYERS = ('intensity', 'no_step')

# Raggio del disco segnato come pericoloso nella mappa globale attorno al punto di una
# caduta (vedi movements.check_and_recover_fall e GlobalGrid.mark_hazard).
FALL_HAZARD_RADIUS_M = 0.5

# ==================================================================
# RITIRATA (vedi retreat_along_traveled_path).
#   RETREAT_MAX_SEGMENTS  -- quanti tratti gia' percorsi ripercorrere a ritroso al
#                            massimo. Pochi: serve a uscire da una tasca, non a
#                            tornare al punto di partenza.
#   RETREAT_MIN_SEGMENT_M -- sotto questa lunghezza il tratto si salta (non vale un
#                            comando di movimento a se').
#   RETREAT_AFTER_ABORT_M -- quanto arretrare subito dopo un arresto a meta'
#                            movimento, per staccarsi da cio' che ha fatto fermare.
# ==================================================================
RETREAT_MAX_SEGMENTS = 3
RETREAT_MIN_SEGMENT_M = 0.20
RETREAT_AFTER_ABORT_M = 0.40
#   RETREAT_CONTINUITY_M  -- si arretra lungo un tratto registrato solo se il robot sta
#                            davvero dove quel tratto finiva. Dopo uno spostamento con GraphNav
#                            fra celle (che non registra tratti) l'ultimo tratto registrato puo'
#                            stare a molti metri, e il robot ci andrebbe camminando
#                            all'indietro, alla cieca.
#   RETREAT_MAX_SEGMENT_M -- tratto piu' lungo di cosi': non lo si ripercorre all'indietro.
RETREAT_CONTINUITY_M = 0.50
RETREAT_MAX_SEGMENT_M = 2.50

# ==================================================================
# SPOSTAMENTI CON GRAPHNAV -- fra celle lontane, per i ritorni e per il ritorno a wp_0.
# Tutti passano da navGraphUtils.graphnav_navigate_to, che:
#   - ha un tempo massimo (GRAPHNAV_TIMEOUT_S), cosi' un robot caduto o senza percorso non
#     blocca la missione;
#   - limita la velocita' (GRAPHNAV_MAX_*_VEL): gli spostamenti con GraphNav sono i piu'
#     lunghi della missione;
#   - tratta STATUS_LOST come fallimento, a robot fermo, dopo i tentativi di riaggancio: non
#     forza la localizzazione sul waypoint di destinazione dichiarando "arrivato" quando il
#     robot potrebbe trovarsi altrove.
# ==================================================================
MAX_FALLS_PER_MISSION = 3      # alla terza caduta il robot si rialza e la missione si ferma
GRAPHNAV_TIMEOUT_S = 240.0
GRAPHNAV_MAX_LINEAR_VEL_MPS = 0.5
GRAPHNAV_MAX_ANGULAR_VEL_RPS = 0.6

# --- LOCALIZZAZIONE DI GRAPHNAV --------------------------------------------------------
# All'aperto il localizzatore dal vivo di GraphNav si puo' perdere anche quando la mappa e'
# sana (odometria delle zampe e visione d'accordo entro ~15 cm su decine di metri), e un
# robot perso si vede rifiutare ogni spostamento. Quindi:
#   1. riga [LOCALIZZAZIONE] a ogni briciola, solo quando qualcosa cambia, per vedere QUANDO
#      e PERCHE' si perde (rilevatore di perdita di Spot);
#   2. riaggancio: se GraphNav risulta perso, si rilocalizza sul waypoint piu' vicino secondo
#      le nostre pose, con la stima iniziale dall'odometria (waypoint_tform_ko del grafo per
#      ko_tform_body di adesso: esatta a meno della deriva), poi la rifinisce l'ICP di Spot.
#      NON si dichiara "arrivato": si riprova a navigare.
#   3. rilevatore di perdita PERMISSIVO negli spostamenti GraphNav.
GRAPHNAV_LOST_DETECTOR_STRICTNESS = 'LOST_DETECTOR_STRICTNESS_PERMISSIVE'   # None = quella di Spot
GRAPHNAV_MAX_RELOCALIZATIONS = 3       # riagganci per singolo spostamento GraphNav
GRAPHNAV_RELOC_MAX_DIST_M = 0.5        # raggio di ricerca dell'ICP attorno alla stima
GRAPHNAV_RELOC_MAX_YAW_RAD = float(np.radians(20.0))
GRAPHNAV_RELOC_MAX_WP_DIST_M = 3.0     # oltre questa distanza dal waypoint piu' vicino non si prova

# Prima di spostarsi con GraphNav al waypoint da cui tentare una cella, si controlla sul grafo
# che da li' si possa arrivare alla cella (vedi graph_reaches_cell). Nella prova del 22-10
# 15:24, tre spostamenti su sei finivano in un tentativo fallito in meno di un secondo per
# "GRAFO DISCONNESSO", che il grafo sapeva gia' prima di partire: ~25 s su 3 minuti.
# False = si parte comunque e decide il tentativo.
GRAPHNAV_PRECHECK = True

# True: all'inizio di ogni cella si rivalutano solo gli archi vicini alle zone con dati
# nuovi (vedi mission_graph.MissionGraph.add_edges_to_prm, incremental). False = si
# rivalutano tutti; il risultato e' lo stesso.
INCREMENTAL_GRAPH_UPDATE = True

SHORTCUT_MAX_DIST_M = 2.5          # oltre, il tratto non sta comunque nella finestra
