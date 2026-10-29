# Esplorazione autonoma con Spot

Spot esplora da solo un'area rettangolare divisa in celle: entra in una cella dopo l'altra, costruisce mentre cammina una mappa degli ostacoli e del terreno, sceglie i percorsi su un grafo di nodi e li percorre solo fin dove la scansione dal vivo conferma che sono liberi. A fine missione torna al punto di partenza con GraphNav e salva tutto ciò che serve per capire, dopo, cosa ha fatto e perché.

La storia delle modifiche, con le misure delle missioni che le hanno motivate, sta in [`CHANGELOG.md`](CHANGELOG.md). Cosa controllare nella prossima prova sul campo sta in [`PROVA_CAMPO.md`](PROVA_CAMPO.md); idee, decisioni rimandate e lavori da fare, con i dati che servono per deciderli, in [`TODO.md`](TODO.md).

---

## Avviare una missione

**Requisiti** (quelli con cui gira oggi): Python 3.8, SDK di Spot (`bosdyn-client`, `bosdyn-core`, `bosdyn-api` 5.1.x), `numpy`, `scipy`; `matplotlib` solo per il visualizzatore. Per l'SDK vedi anche `docker-requirements.txt`.

1. Copiare **tutta la cartella** sul computer collegato al robot (non un solo file: i moduli devono essere della stessa versione).
2. Controllare che i moduli siano allineati:
   ```bash
   python3 preflight.py          # deve dire "Tutti i file sono allineati fra loro."
   ```
3. Mettere il robot al punto di partenza, rivolto verso l'area da esplorare (facoltativo: un fiducial qualunque a 1–2 m, davanti o dietro; Spot riconosce gli AprilTag 36h11 di Boston Dynamics, non gli ArUco. Vedi `START_FIDUCIAL_ID` in `parametri.py`).
4. Avviare:
   ```bash
   python3 easy_walk.py
   ```
   L'indirizzo del robot è in `main()` di `easy_walk.py` (`192.168.80.3`). Le credenziali le gestisce l'SDK (`bosdyn.client.util.authenticate`): con l'SDK originale si danno con le variabili `BOSDYN_CLIENT_USERNAME` e `BOSDYN_CLIENT_PASSWORD`, altrimenti le chiede da terminale. Attenzione: sul computer di sviluppo quel file dell'SDK è stato modificato per usare credenziali fisse; su un computer nuovo, con l'SDK appena installato, servono le variabili.

**Parametri.** Tutti i numeri e gli interruttori che si cambiano sul campo stanno in `parametri.py`; quelli propri di un modulo (soglie del terreno in `spotGrid.py`, grafo in `prm_graph.py`, fronte in `arcVerification.py`, velocità in `movements.py`) restano lì.

**Dimensione della missione.** Il default è in `parametri.py` (`MISSION_GRID_ROWS`, `MISSION_GRID_COLS`, `MISSION_CELL_SIZE_M`): oggi 8 righe × 7 colonne da 2 m, cioè 16 m a sinistra e 14 m davanti al robot. Per una sola missione, senza toccare il codice (in ordine di precedenza):
```bash
python3 easy_walk.py --rows 2 --cols 3 --cell 4      # argomenti
SPOT_MISSION_GRID="2,3,4" python3 easy_walk.py       # variabile d'ambiente
```
La riga `[CONFIG] Griglia di missione` del log dice quale valore è stato usato e da dove.

**Grafo della missione.** I nodi si generano in due modi (`--grafo`, default in `parametri.MISSION_GRAPH_SAMPLING`):
```bash
python3 easy_walk.py --grafo casuale --distribuzione poisson --densita 5 --seme 7       # nodi casuali (default: poisson, 5 nodi/m², seme a caso)
python3 easy_walk.py --grafo reticolo --spacing 0.5                                    # reticolo
```
| Argomento | Default | Cosa fa |
|---|---|---|
| `--min-edge`, `--max-edge` | 0.5, 1.25 m | Lunghezza minima e massima degli archi, per tutti e due i metodi |
| `--spacing` | 0.5 m | Reticolo: passo fra i nodi, arrotondato a lato cella / n |
| `--rings` | tutti | Reticolo: anelli di vicini collegati (dentro min e max) |
| `--distribuzione` | `poisson` | Nodi casuali: `uniforme` (PRM classico, con buchi e grumi), `poisson` (distanza minima fra i nodi), `perturbata` (un punto a caso in ogni quadretto), `halton` (quasi casuale, molto uniforme) |
| `--densita` | 5 nodi/m² | Nodi casuali: quanti per m² (4 = la densità del reticolo a 0.5 m) |
| `--seme` | a caso | Nodi casuali: stesso seme, stesso grafo. Se manca se ne estrae uno; quello usato è nel log (`[GRAFO] Nodi casuali`, con il comando per ripetere) e in `mission_summary.json` |

Con i nodi casuali i centri delle celle e il punto di partenza restano comunque nodi. La copertura (quanto può distare un punto dell'area dal nodo più vicino) è misurata e scritta nel log. Per guardare i grafi prima di uscire: `python3 prova_grafo.py`. Usa gli stessi parametri della missione (`parametri.py`) e apre un'immagine con i metodi affiancati e una tabella; in fondo stampa, per ogni grafo, le righe da mettere in `parametri.py` (o gli argomenti di `easy_walk.py`) per usare in missione proprio quello, seme compreso. Per provare altri valori senza toccare `parametri.py` si cambia il blocco PROVA in cima al file. Da riga di comando, con gli stessi default della missione:
```bash
python3 mission_graph.py --confronta-metodi metodi.png --seme 7 --zoom -2 2 -2 2       # i cinque metodi affiancati, con i numeri
python3 mission_graph.py --grafo casuale --distribuzione halton --seme 7 --preview halton.png --zoom -2 2 -2 2   # uno solo
```

---

## Come funziona una missione

### Celle ed esplorazione
L'area è una griglia di celle (`environmentMap.py`) riferita alla posa del robot all'avvio: le **colonne** vanno in avanti, le **righe** a sinistra. Le celle si visitano a serpentina; a ogni passo si sceglie la cella di frontiera confinante di rango più basso. Se nessuna confina con la posizione attuale, il robot va con GraphNav al waypoint più vicino a quella di rango più basso e prova da lì. Una cella fallita da un lato resta candidata e si ritenta più avanti da un altro lato; diventa bloccata solo quando tutti i lati sono stati provati, o subito se l'obiettivo risulta dentro un ostacolo.

### Un tentativo di cella (`TentativoCella` in `easy_walk.py`)
Il tentativo prepara scansione, mappe, grafo e percorso (`_prepara`), poi ripete un giro di fasi (`_guardie`, `_scansione_e_grafo`, `_fronte_e_decisione`, `_rotazione`, `_nodo_o_arrivo`, `_attesa`, `_blocco`, `_esegui_tratti`) finché il robot arriva o il tentativo fallisce.
1. **Scansione** della griglia locale di Spot (obstacle_distance, terrain, terrain_valid): terreno corretto, celle mai scritte scartate, pendenza e rugosità, maschera degli ostacoli (`spotGrid.py`).
2. **Mappe globali** accumulate per tutta la missione: occupazione (un ostacolo conta solo dopo più osservazioni), terreno, spazio libero (`spotGrid.GlobalGrid`, `GlobalTerrainGrid`).
3. **Grafo** (`mission_graph.py`, `prm_graph.py`): i nodi sono un reticolo regolare a passo 0.5 m, deterministico, con il centro di ogni cella come nodo, oppure nodi casuali con i centri cella compresi (vedi *Grafo della missione*); gli archi vanno da 0.5 a 1.25 m. Ogni arco viene vetato o pesato con occupazione, pendenza longitudinale e laterale (misurata anche oltre i nodi, di mezzo corpo per parte) e scarto preliminare sulla scansione dal vivo. A inizio cella si rivalutano solo gli archi vicini ai dati nuovi.
4. **Obiettivo** (`find_best_point_in_cell`): fra i punti della cella non dentro ostacoli, prima quelli raggiungibili dal robot nel grafo, con almeno 0.33 m di spazio libero e a 0.30 m dal bordo della cella; un obiettivo rifiutato dal fronte resta escluso per tutta la missione. L'arrivo vale anche con il robot fino a 0.35 m fuori dalla cella. **Percorso** con Dijkstra, solo sulle celle già visitate, con un tetto alla deviazione.
5. **Ciclo di navigazione** con il **fronte sicuro** (`arcVerification.py`): a ogni giro una scansione fresca dice fin dove il percorso è libero adesso, e il robot avanza fino a lì, anche per più nodi di fila. Durante il movimento un thread ricalcola il fronte circa 5 volte al secondo e ferma il robot se si accorcia.
   - Il robot punta sempre il muso verso il punto da raggiungere (niente marcia di traverso). Se non può ruotare: avanza dentro il cono di rotazione possibile, oppure arretra dritto se da lì la rotazione diventa possibile, altrimenti scarta il tratto.
   - Una cella bloccante si scarta subito e si ripianifica; un blocco confermato da tre scansioni ferme diventa permanente.
   - Se nessuna direzione del giro è percorribile (trappola), il robot torna indietro con GraphNav lungo le "briciole" (waypoint lasciati ogni 0.75 m) e ripianifica da lì; dopo due ritorni nello stesso tentativo rinuncia alla cella da quel lato.
6. **Andatura** scelta tratto per tratto (`movements.select_mobility_params`): PLAIN, MODERATE (crawl), HARD (crawl), con velocità massime per livello.

### Sicurezza
- Limiti di velocità su ogni comando, arresto a metà movimento, tempo massimo per movimento e per spostamento GraphNav.
- Rilevazione delle cadute: il robot si rialza, il punto viene segnato pericoloso nella mappa; alla terza caduta la missione si ferma.
- **Batteria** (`batteria.py`): carica letta a ogni giro e salvata in `battery.csv` con il consumo per minuto e per metro. Sotto `BATTERY_RETURN_PCT` (oggi 30%, provvisorio) la missione smette di esplorare e chiude come a fine missione, con il ritorno a `wp_0`; le celle non esplorate restano in frontiera.
- GraphNav riaggancia la localizzazione sul waypoint più vicino se si perde; non dichiara mai "arrivato" senza esserlo.

### Fine missione
Si arriva qui quando la frontiera è vuota, oppure prima se la batteria scende sotto la soglia di rientro. Scansione finale, riepilogo (`[MISSION STATS]`, `[TEMPO]`), chiusura d'anello di Spot sull'intero grafo GraphNav (al massimo 45 s) e ancoraggio, ritorno a `wp_0` con GraphNav (con scorciatoie scelte sulla strada), seduta, spegnimento, scaricamento del grafo GraphNav. Durante l'esplorazione la chiusura d'anello di Spot è spenta: gli spostamenti usano le nostre scorciatoie (archi visti liberi nella mappa globale). A ogni arrivo GraphNav si misura la deriva dell'odometria (`drift.csv`).

---

## Moduli

| File | Cosa fa |
|---|---|
| `easy_walk.py` | La missione: scelta dell'obiettivo in una cella, `TentativoCella` (un ingresso in una cella), ritorni e ritirata, celle bloccate, `Missione` (il ciclo cella per cella), avvio e chiusura |
| `parametri.py` | Tutti i parametri e gli interruttori della missione, letti come `P.NOME` |
| `registro.py` | Statistiche, log e dati salvati (CSV, pacchetti di scansione, viste); niente decide il movimento |
| `environmentMap.py` | Griglia delle celle, stato (visitata, bloccata, lati provati), serpentina e frontiera |
| `mission_graph.py` | Nodi (reticolo o casuali), coppie candidate, costruzione e aggiornamento degli archi |
| `prm_graph.py` | Grafo (PRM): veti, pesi, penalità, refresh attorno al robot, Dijkstra |
| `spotGrid.py` | Scansione (`LocalGrid.fetch_scan`, `process_scan`), correzioni, pendenza e rugosità, mappe globali, soglie e andature |
| `arcVerification.py` | Fronte sicuro e decisioni di movimento: fin dove si può avanzare, rotazione, cono, arretramento, trappole |
| `movements.py` | Comandi di movimento (velocità, arresto, tempo massimo), scelta dell'andatura, cadute |
| `navGraphUtils.py` | Tutto GraphNav: registrazione, briciole, spostamenti, scorciatoie, chiusura d'anello, ritorni, deriva |
| `batteria.py` | Carica e consumo della batteria (`battery.csv`), decisione di rientro a batteria bassa |
| `spotLogInUtils.py`, `spotUtils.py` | Collegamento al robot, E-Stop, posizione |
| `mission_report.py`, `mission_viewer.py` | Rapporto e figure, dopo la missione (senza SDK) |
| `visualizzatore_live.py` | Una figura che si aggiorna da sola durante la missione, in un terminale a parte (senza SDK) |
| `test_e2e_loop.py`, `test_missione.py`, `test_replay.py`, `test_obiettivi.py`, `preflight.py`, `confronta_log.py` | Prove senza robot (vedi sotto) |
| `prova_grafo.py`, `prova_anelli.py` | Prove dei grafi e della chiusura d'anello di Spot, al banco |

---

## Interruttori

Ognuno permette un confronto acceso/spento sul campo; il log li riporta nella riga `[CONFIG] INTERRUTTORI` e `mission_summary.json` nella voce `interruttori`.

| Interruttore | Default | Cosa fa |
|---|---|---|
| `parametri.MISSION_GRAPH_SAMPLING` | `casuale` | Come si generano i nodi: `reticolo` o `casuale` (vedi *Grafo della missione*) |
| `parametri.INCREMENTAL_GRAPH_UPDATE` | `True` | A inizio cella rivaluta solo gli archi vicini ai dati nuovi (stesso risultato della ricostruzione completa) |
| `mission_graph.BATCH_SLOPES` | `True` | Pendenze a inizio cella calcolate in blocco (stesso risultato, più veloce) |
| `prm_graph.REFRESH_BATCH_SLOPES` | `True` | Pendenze del refresh calcolate in blocco (stesso risultato, più veloce) |
| `spotGrid.ROBOT_CLEARANCE_AIR_M` | `0.0` | Aria fra corpo e ostacoli: la linea del percorso sta a 0.25 m + questo valore dagli ostacoli (fronte e veto degli archi). 0.05 = con margine |
| `spotGrid.PLAN_CLEARANCE_EXTRA_M` | `0.03` | Sulla mappa globale e nella scelta dell'obiettivo, il pianificatore tiene gli archi a (distanza del fronte + questo valore) dagli ostacoli, così il rumore della scansione non fa rifiutare dal fronte i tratti appena scelti. 0 = stessa distanza del fronte |
| `spotGrid.CLEARANCE_MAP_DECIDES` | `False` | La mappa dello spazio libero decide la percorribilità al posto della mappa binaria |
| `parametri.PLAN_ONLY_EXPLORED_CELLS` | `True` | I percorsi passano solo dalle celle già visitate |
| `parametri.GRAPHNAV_ARRIVAL_HEADING` | `verso_obiettivo` | Orientamento all'arrivo dei viaggi GraphNav fra celle e dei ritorni: girato verso la cella da tentare (o l'obiettivo del tentativo); `waypoint` = quello del waypoint, `libero` = nessuna rotazione finale. Il ritorno a wp_0 tiene sempre quello di partenza |
| `parametri.GRAPHNAV_RETURN_NEAR_M` | `1.0` | Ritorno a wp_0 fermo entro questa distanza: resta non riuscito, ma fiducial e deriva si misurano lo stesso |
| `parametri.MOVE_NEAR_GOAL_M` | `0.10` | Tratto il cui comando non si conclude (tempo scaduto) con il robot entro questa distanza dal punto: vale come riuscito invece che come fallimento fisico (0 = spento) |
| `parametri.GRAPHNAV_PRECHECK` | `True` | Prima di andare col GraphNav al waypoint da cui tentare una cella, controlla sul grafo che da lì si arrivi alla cella; se no, segna il lato come provato senza muoversi |
| `parametri.GRAPHNAV_SHORTCUTS` | `True` | Scorciatoie nel grafo GraphNav prima di ogni spostamento |

| `parametri.GRAPHNAV_SDK_LOOP_BUDGET_S` | `0.0` | Chiusura d'anello veloce di Spot prima di ogni spostamento GraphNav (0 = spenta) |
| `parametri.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S` | `45.0` | Chiusura d'anello completa a fine missione, prima dell'ancoraggio (0 = spenta) |
| `parametri.BATTERY_RETURN_PCT` | `30.0` | Sotto questa carica (2 letture di fila) la missione chiude e torna a `wp_0` (0 = rientro spento, la carica si misura comunque) |
---

## Cosa si salva

Ogni missione ha un nome `Mission_<giorno-mese-anno>_<ora>` e tre cartelle:

| Cartella | Contenuto | Peso tipico |
|---|---|---|
| `MissionLogs/<missione>/` | `mission_log.txt` (tutto l'output, con l'ora a ogni riga), `segments.csv` (un tratto per riga: pendenza, andatura, inclinazione reale), `trajectory.csv` (posizione 5 volte al secondo durante i movimenti), `frontier_live.csv`, `drift.csv`, `battery.csv` (carica e consumo), `mission_summary.json` | 0.5 MB |
| `MissionMap/<missione>/` | `scans/` (un pacchetto per scansione: layer grezzi e derivati, fronte, decisione), `*_vis.npz` (viste del grafo), `*_BLOCKED_*.npz`, mappe globali `.pkl` | 70 MB |
| `graph/<missione>/` | grafo GraphNav scaricato, con le istantanee dei waypoint | 150 MB |

---

## Durante la missione

```bash
python3 visualizzatore_live.py                   # in un secondo terminale: l'ultima missione, dal vivo
python3 visualizzatore_live.py 28-10-2026_14-23-50 --rigioca --velocita 20   # una missione finita, rigiocata
```

Una sola figura con tutto sopra: mappa vista finora (rosso ostacolo, verde libero), ultima scansione nel riquadro azzurro, celle (verde visitata, rossa bloccata, bordo blu = obiettivo), grafo PRM, strada fatta (blu, tratteggio = GraphNav), percorso pianificato (arancione), obiettivo (stella), punto dove si ferma il fronte (X) e avanzamento consentito (verde spesso), robot (freccia). A destra cella, decisione, fronte, avanzamento, batteria e le ultime righe utili del log. Legge solo i file che la missione scrive già: non la rallenta e non la può fermare. Tasti: `f` segui il robot / tutta la missione, spazio pausa, `q` esci.

## Dopo la missione

```bash
python3 mission_report.py                        # rapporto dell'ultima missione (o: nome della missione, --tutte)
python3 mission_viewer.py 13-10-2026_12-40-38    # scansioni, mappa globale e grafo, interattivo (--png <dir> per esportare)
python3 test_replay.py --missioni <missione>     # rigioca le scansioni attraverso il codice di oggi
python3 test_replay.py --tutte --grafo casuale --distribuzione halton --seme 7   # stesse scansioni, altro grafo
```

- **`mission_report.py`**: una pagina con celle, tempi, fluidità del movimento, andature e inclinazioni, fermate del fronte, ritorno a wp_0, e i **controlli della prova** (OK / DA GUARDARE / NON SCATTATO / DECISIONE).
- **`mission_viewer.py`**: le figure si fanno qui, dopo; durante la missione il robot non disegna.
- **`test_replay.py`**: verifica una modifica senza il robot, sulle scansioni vere di 10 missioni (`--tutte`, circa 5 minuti): invarianti che devono valere sempre, e confronto con il comportamento di riferimento (`test_replay_riferimento.json`). Con `--grafo` (e `--distribuzione`, `--densita`, `--seme`, `--min-edge`, `--max-edge`) rigioca le stesse scansioni con un altro grafo: la tabella finale (percorsi trovati, lunghezza, archi per percorso, tempo del grafo) si confronta con quella del reticolo.
- **`test_e2e_loop.py`**: il ciclo di navigazione vero contro un robot finto in mondi sintetici (corridoi, trappole, muri).
- **`test_missione.py`**: la missione intera (`easy_walk.main()` vero) con robot e GraphNav finti, in quattro mondi: muri, cella chiusa, blocchi, ritorni, fallimenti forzati, rientro a batteria bassa.
- **`test_obiettivi.py`**: la scelta dell'obiettivo rigiocata sulle missioni vere.
- **`confronta_log.py`**: confronta i log di due versioni (`E2E_LOG_DIR=<cartella> python3 test_e2e_loop.py`, idem `test_missione.py`), togliendo ore e durate: una modifica che non deve cambiare il comportamento deve dare log identici.
- **`preflight.py`**: controlla che i moduli siano coerenti fra loro.

Ogni modifica si verifica con `test_replay.py --tutte`, `test_e2e_loop.py`, `test_missione.py` e `preflight.py` prima di andare sul robot.

---

## Convenzioni

- **Coordinate**: frame VISION di Spot. x, y e direzione sono riferiti alla posa all'avvio della missione (`env.set_origin`); le quote alla quota del suolo all'avvio (`mission_z0`), perché Spot le misura da dove è stato acceso.
- **Celle**: colonne in avanti, righe a sinistra; la cella di partenza è (0,0).
- **Lati di una cella**: 4 bit, `0b1000` nord, `0b0100` est, `0b0010` sud, `0b0001` ovest.
- **Waypoint GraphNav**: `wp_0` alla partenza, poi numerati in ordine (`wp_1`, `wp_2`, …).
- **Log**: ogni riga ha un'etichetta fra parentesi quadre (`[ATTEMPT]`, `[FRONTE]`, `[BLOCCO]`, `[GRAPHNAV]`, `[TEMPO]`, …), che resta stabile: i messaggi possono cambiare, le etichette no.
