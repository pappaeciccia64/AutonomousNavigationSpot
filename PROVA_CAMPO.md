# Prova sul campo: cosa controllare

Le modifiche del 26-10 (righe con quella data) non sono mai andate sul robot; quelle dal 14-10 al 22-10 sono andate sul robot il 22-10 al chiuso e all'aperto. Alcune del 13-10 erano nel codice delle missioni delle 12:14–12:40 ma non sono mai scattate, perché la situazione non si è presentata. Questa lista dice cosa guardare e cosa aspettarsi; `mission_report.py` fa gli stessi controlli da solo, nella sezione **Controlli della prova**.

---

## Prima di partire

1. **Copiare tutta la cartella** sul PC che comanda il robot, non solo `easy_walk.py`. Il 05-10 una copia vecchia di `spotGrid.py` ha fermato la missione appena costruito il grafo.
2. Sul PC del robot: `python3 preflight.py` → deve dire **"Tutti i file sono allineati fra loro."**
3. **Dimensione della missione.** Oggi è fissata in `mission_graph.py`: `MISSION_ROWS = 8`, `MISSION_COLS = 8`, `MISSION_CELL_SIZE_M = 2`, cioè **16 m a sinistra × 16 m davanti** al robot, 64 celle, come la prova all'aperto del 22-10 16:04 (42 minuti di missione più 6 di chiusura). Se l'area è diversa, senza toccare il codice: `python3 easy_walk.py --rows 6 --cols 6 --cell 2`. La prima riga `[CONFIG] Griglia di missione` del log la conferma.
4. **Grafo.** Default: nodi casuali poisson, 5 nodi/m², seme estratto a caso. Le righe `[CONFIG] Grafo` e `[GRAFO] Nodi casuali` del log riportano il seme e il comando per rifare lo stesso grafo.
5. Facoltativo: il tag **549** a circa 1 m davanti al robot alla partenza. Senza, la missione parte lo stesso (l'origine è il primo waypoint), come il 13-10.

---

## Le novità da verificare

| Novità | Riga di log | Cosa aspettarsi | Se va male |
|---|---|---|---|
| **Aggiornamento locale del grafo** e **pendenze in blocco** (14-10) | `[TEMPO] aggiornamento locale del grafo: X s` (oppure `costruzione completa`) | Una sola costruzione completa, al primo tentativo; poi aggiornamenti locali da **1–3 s**. Il 13-10 12:40 erano 11 costruzioni complete da 7.7 s in media. | `INCREMENTAL_GRAPH_UPDATE = False` in `easy_walk.py`, `BATCH_SLOPES = False` in `mission_graph.py`: stessi risultati, solo più lenti. |
| **Chiusura d'anello di Spot solo a fine missione, completa** (26-10 sera, `GRAPHNAV_SDK_LOOP_BUDGET_S = 0`, `_FINAL_S = 45`; archi caricati da noi) | Nessuna riga `Chiusura d'anello` prima dei viaggi; una riga `(completa): N archi in X s` prima di `[ANCHORING]` | Partenza dei viaggi GraphNav 1–2 s dopo la preparazione. A fine missione circa un minuto e una quindicina di archi (al banco 17 in 45 s su un grafo senza archi veloci). Nessuna riga `tentativo N non riuscito`; se compare `riprovo facendoli scrivere a Spot`, il caricamento nostro è stato rifiutato: annotarlo. | `GRAPHNAV_SDK_LOOP_CLOSURE = False` in `easy_walk.py`. Per riavere la veloce: `GRAPHNAV_SDK_LOOP_BUDGET_S = 3` (ma allora la completa finale non trova niente). |
| **Layer in più nelle scansioni** (26-10, `SAVE_EXTRA_LAYERS`) | nessuna: dopo, `intensity` e `no_step` nei pacchetti `scans/*.npz` | Presenti nei pacchetti; tempo di `scansione` nel rapporto quasi invariato. | `SAVE_EXTRA_LAYERS = ()`. |
| **Scelta dell'obiettivo** (26-10: raggiungibile, 0.33 m, 0.30 m dal bordo, rifiutati esclusi) | `[TARGET] Cella (r,c): …`, `[TARGET] Obiettivo … escluso per il resto della missione`, `NESSUN punto della cella e' raggiungibile` | Pochi "GRAFO DISCONNESSO" all'inizio del tentativo; mai lo stesso obiettivo riscelto dopo l'esclusione (il rapporto li conta). | Annotare cella e ora; `test_obiettivi.py --missioni <nome> --dettaglio`. |
| **Arrivo appena fuori dalla cella** (26-10, fino a 0.35 m) | `[OBIETTIVO] Il robot e' fuori dalla cella … vale come arrivo` | Pochi `ancora fuori dalla cella` (il 22-10 erano 44). | `ARRIVAL_OUTSIDE_CELL_M = 0` in `easy_walk.py` torna alla regola del 22-10. |
| **Ritorno senza movimento** (26-10) | `[RITORNO] Torno a wp_N, a X m dal robot`, `… non mi muovo, ripianifico da qui` | Nessun ritorno GraphNav di 0 m. | — |
| **Controllo prima del GraphNav** (22-10 sera, `GRAPHNAV_PRECHECK`) | `[PRECHECK] Cella (r,c): dal waypoint …` | Nessuno spostamento GraphNav seguito da un fallimento immediato con "GRAFO DISCONNESSO". | `GRAPHNAV_PRECHECK = False` in `easy_walk.py`. |
| **Pianificazione a 0.28 m** (22-10 sera, `PLAN_CLEARANCE_EXTRA_M = 0.03`) | `[FRONTE] … ostacolo a 0.2x m (minimo 0.25 m)` | Molte meno fermate del fronte a 0.23–0.24 m (alle 15:24 erano 18 giri su 25). | `PLAN_CLEARANCE_EXTRA_M = 0` in `spotGrid.py`. |
| **Nodi casuali poisson** (22-10, prima volta sul robot) | `[GRAFO] Nodi casuali …`, `[TEMPO] … grafo …` | Grafo da ~2100 nodi e ~21 mila archi sulla 10 × 10 da 2 m, costruito in meno di un secondo; aggiornamenti locali molto più brevi di quelli del reticolo a 0.25 m. Qualche passaggio stretto (sotto ~1 m) può non avere nodi: il robot rinuncia a quel lato e prova la cella da un altro. | `--grafo reticolo --spacing 0.5` (o `0.25` per i passaggi stretti, più lento). |
| **Celle da 2 m** (22-10) | `[ATTEMPT]`, `[TARGET]` | Tentativi più corti; obiettivo quasi sempre il centro. | `--cell 4` con meno righe e colonne. |
| **Margine attorno al corpo 0.25 m** (22-10, `ROBOT_CLEARANCE_AIR_M = 0`) | `[FRONTE]`, `[BLOCCO]`, fermate di Spot vicino agli ostacoli | Il robot passa più vicino agli ostacoli (linea a 0.25 m invece di 0.30) senza toccarli; nessuna fermata di Spot stesso nei passaggi stretti. | `ROBOT_CLEARANCE_AIR_M = 0.05` in `spotGrid.py`. Annotare dove ha sfiorato o si è fermato. |
| **Refresh con le pendenze in blocco** (21-10) | `[TEMPO] scansione … \| PRM X \| …` a ogni giro | PRM sotto ~0.5 s per giro (il 13-10 12:40 erano 1.35 s di media). | `REFRESH_BATCH_SLOPES = False` in `prm_graph.py`: stessi risultati, solo più lento. |
| **Obiettivo al centro della cella** (14-10) | `[TARGET] Cella (r,c): uso il centro geometrico …` oppure `centro …, uso il punto … a D m dal centro` | Quasi sempre il centro; ripieghi entro ~1 m dal centro; nessun arrivo fuori dalla cella. | Annotare cella e ora: si guarda con `mission_viewer.py`. |
| **Pendenza col corpo** nel veto e nell'andatura (14-10) | `[GAIT] Terrain tier: …`, `[PENDENZE] costruzione: …` | Ogni tratto in cui il robot si inclina di più di 18 gradi è **HARD** (il rapporto li conta). Qualche arco vetato in più vicino ai gradini. | `ARC_BODY_EXTENSION_M` in `spotGrid.py`. |
| **Scorciatoie GraphNav sulla strada** (14-10) | `[GRAPHNAV] …: strada verso wp_X A m -> B m con N scorciatoie` | Ritorno a wp_0 molto più corto: il 13-10 12:40 erano 46.7 m e 115 s; rigiocato col codice di oggi, 18.5 m. Nessuna riga `Scorciatoia … rifiutata`. | `GRAPHNAV_SHORTCUTS = False` in `easy_walk.py` (si torna alla sola catena). |
| **Mappa del terreno senza celle ostacolo** (14-10) | nessuna riga: si controlla dopo, col rigioco | `test_replay.py --missioni <nome>` → invariante 2 OK (pendenza della mappa globale uguale al vivo). | — |
| **Riaggancio di GraphNav** (13-10, mai scattato) | `[LOCALIZZAZIONE] …: PERSO`, `[LOCALIZZAZIONE] Riaggancio …: riuscito su wp_N` | Se si perde, il riaggancio riesce e lo spostamento continua. Il 13-10 non si è mai perso. | Annotare dove: `[LOCALIZZAZIONE]` riporta andatura e beccheggio. |
| **Ritorni con memoria e recupero** (13-10, scattato una volta) | `[RITORNO] Arco a->b fuori dal piano …`, `[RECOVERY] L'obiettivo … è quello del tentativo appena fallito` | Mai lo stesso giro trappola → ritorno → stesso percorso più di due volte. | — |

---

## Durante la prova

- Annotare l'**ora** di ogni cosa strana (il log ha l'ora a ogni riga): si ritrova subito nel log e nelle scansioni.
- **Erba**: quando il robot si ferma o gira attorno a qualcosa che sembra erba, annotare l'ora e fare una foto di cosa c'è davanti. Serve come verità per confrontare le scansioni (con i layer `intensity` e `no_step`).
- I messaggi di log sono in italiano dal 21-10; le etichette fra parentesi quadre (`[ATTEMPT]`, `[BLOCKED]`, …) sono rimaste le stesse.
- Il robot non deve mai restare **fermo a girare a vuoto**: se succede, prima o poi compare `[STALLO]`; annotare quanto ci mette.
- Sfioramenti durante le rotazioni: il primo numero da cambiare è `ROTATION_NOISE_TOLERANCE_M` in `spotGrid.py` (0.15 → 0.10, vedi CHANGELOG 08-10).

---

## Dopo la prova

1. `python3 mission_report.py` → rapporto dell'ultima missione, con in fondo i **Controlli della prova** (OK / DA GUARDARE / NON SCATTATO / DECISIONE).
2. `python3 mission_viewer.py <nome>` per guardare i punti segnati come DA GUARDARE.
3. `python3 test_replay.py --missioni <nome>` → gli invarianti sulle scansioni nuove, compreso il ritorno a wp_0 se il grafo è stato scaricato. Se va tutto bene, la missione si aggiunge a `ALL_MISSIONS` in `test_replay.py` e al riferimento con `--aggiorna-riferimento`.

---

## Decisioni da prendere con i dati della prova

- **`CLEARANCE_MAP_DECIDES`** (`spotGrid.py`, oggi `False`): la mappa dello spazio libero impara ma non decide. Il rapporto dà quanti archi bloccherebbe in più o in meno (riga DECISIONE). Sulle missioni del 12-10 riconosceva il 52–86% delle fermate del fronte contro il 39–43% della mappa binaria. Da accendere se nella prova il pianificatore ripropone lo stesso varco stretto più volte (fermate "di striscio" nel rapporto, `[SUBITO]` ripetuti nello stesso punto).
- **CRAWL in MODERATE**: il rapporto dà il beccheggio per andatura. Se i tratti MODERATE restano tranquilli (sotto ~8 gradi), si può tornare all'andatura normale (`HINT_AUTO` in `select_mobility_params`), che è più veloce.
- **Fluidità del movimento**: oggi il robot cammina in media a 0.14 m/s (13-10 12:40), perché ogni tratto aspetta che il corpo sia fermo e ruota a parte. Il rapporto lo misura nella riga **Fluidita'** (velocità media, tempo fermo a ruotare o a partire). Se la modifica viene preparata prima della prova, va confrontata qui, con l'interruttore acceso e spento.
