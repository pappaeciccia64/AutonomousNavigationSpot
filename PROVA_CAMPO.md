# Prova sul campo: cosa controllare

Le modifiche del 14-10 non sono mai andate sul robot. Alcune del 13-10 erano nel codice delle missioni delle 12:14–12:40 ma non sono mai scattate, perché la situazione non si è presentata. Questa lista dice cosa guardare e cosa aspettarsi; `mission_report.py` fa gli stessi controlli da solo, nella sezione **Controlli della prova**.

---

## Prima di partire

1. **Copiare tutta la cartella** sul PC che comanda il robot, non solo `easy_walk.py`. Il 05-10 una copia vecchia di `spotGrid.py` ha fermato la missione appena costruito il grafo.
2. Sul PC del robot: `python3 preflight.py` → deve dire **"Tutti i file sono allineati fra loro."**
3. **Dimensione della missione.** Oggi è fissata in `mission_graph.py`: `MISSION_ROWS = 3`, `MISSION_COLS = 4`, `MISSION_CELL_SIZE_M = 4`, cioè **12 m a sinistra × 16 m davanti** al robot. Se l'area è diversa, senza toccare il codice: `python3 easy_walk.py --rows 2 --cols 3 --cell 4`. La prima riga `[CONFIG] Griglia di missione` del log la conferma.
4. Facoltativo: il tag **549** a circa 1 m davanti al robot alla partenza. Senza, la missione parte lo stesso (l'origine è il primo waypoint), come il 13-10.

---

## Le novità da verificare

| Novità | Riga di log | Cosa aspettarsi | Se va male |
|---|---|---|---|
| **Aggiornamento locale del grafo** e **pendenze in blocco** (14-10) | `[TEMPO] aggiornamento locale del grafo: X s` (oppure `costruzione completa`) | Una sola costruzione completa, al primo tentativo; poi aggiornamenti locali da **1–3 s**. Il 13-10 12:40 erano 11 costruzioni complete da 7.7 s in media. | `INCREMENTAL_GRAPH_UPDATE = False` in `easy_walk.py`, `BATCH_SLOPES = False` in `mission_graph.py`: stessi risultati, solo più lenti. |
| **Obiettivo al centro della cella** (14-10) | `[TARGET] Cella (r,c): uso il centro geometrico …` oppure `centro …, uso il punto … a D m dal centro` | Quasi sempre il centro; ripieghi entro ~1 m dal centro; nessun arrivo fuori dalla cella. | Annotare cella e ora: si guarda con `mission_viewer.py`. |
| **Pendenza col corpo** nel veto e nell'andatura (14-10) | `[GAIT] Terrain tier: …`, `[PENDENZE] costruzione: …` | Ogni tratto in cui il robot si inclina di più di 18 gradi è **HARD** (il rapporto li conta). Qualche arco vetato in più vicino ai gradini. | `ARC_BODY_EXTENSION_M` in `spotGrid.py`. |
| **Scorciatoie GraphNav sulla strada** (14-10) | `[GRAPHNAV] …: strada verso wp_X A m -> B m con N scorciatoie` | Ritorno a wp_0 molto più corto: il 13-10 12:40 erano 46.7 m e 115 s; rigiocato col codice di oggi, 18.5 m. Nessuna riga `Scorciatoia … rifiutata`. | `GRAPHNAV_SHORTCUTS = False` in `easy_walk.py` (si torna alla sola catena). |
| **Mappa del terreno senza celle ostacolo** (14-10) | nessuna riga: si controlla dopo, col rigioco | `test_replay.py --missioni <nome>` → invariante 2 OK (pendenza della mappa globale uguale al vivo). | — |
| **Riaggancio di GraphNav** (13-10, mai scattato) | `[LOCALIZZAZIONE] …: PERSO`, `[LOCALIZZAZIONE] Riaggancio …: riuscito su wp_N` | Se si perde, il riaggancio riesce e lo spostamento continua. Il 13-10 non si è mai perso. | Annotare dove: `[LOCALIZZAZIONE]` riporta andatura e beccheggio. |
| **Ritorni con memoria e recupero** (13-10, scattato una volta) | `[RITORNO] Arco a->b fuori dal piano …`, `[RECOVERY] L'obiettivo … è quello del tentativo appena fallito` | Mai lo stesso giro trappola → ritorno → stesso percorso più di due volte. | — |

---

## Durante la prova

- Annotare l'**ora** di ogni cosa strana (il log ha l'ora a ogni riga): si ritrova subito nel log e nelle scansioni.
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
- **Fluidità del movimento**: oggi il robot cammina in media a 0.14 m/s (13-10 12:40), perché ogni tratto aspetta che il corpo sia fermo e ruota a parte. Se la modifica viene preparata prima della prova, va confrontata qui, con l'interruttore acceso e spento.
