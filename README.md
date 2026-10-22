# Esplorazione autonoma con Spot

Spot esplora da solo un'area rettangolare divisa in celle: entra in una cella dopo l'altra, costruisce mentre cammina una mappa degli ostacoli e del terreno, sceglie i percorsi su un grafo di nodi e li percorre solo fin dove la scansione dal vivo conferma che sono liberi. A fine missione torna al punto di partenza con GraphNav e salva tutto ciò che serve per capire, dopo, cosa ha fatto e perché.

La storia delle modifiche, con le misure delle missioni che le hanno motivate, sta in [`CHANGELOG.md`](CHANGELOG.md). Cosa controllare nella prossima prova sul campo sta in [`PROVA_CAMPO.md`](PROVA_CAMPO.md).

---

## Avviare una missione

**Requisiti** (quelli con cui gira oggi): Python 3.8, SDK di Spot (`bosdyn-client`, `bosdyn-core`, `bosdyn-api` 5.1.x), `numpy`, `scipy`; `matplotlib` solo per il visualizzatore. Per l'SDK vedi anche `docker-requirements.txt`.

1. Copiare **tutta la cartella** sul computer collegato al robot (non un solo file: i moduli devono essere della stessa versione).
2. Controllare che i moduli siano allineati:
   ```bash
   python3 preflight.py          # deve dire "Tutti i file sono allineati fra loro."
   ```
3. Mettere il robot al punto di partenza, rivolto verso l'area da esplorare (facoltativo: il tag fiducial 549 a circa 1 m davanti).
4. Avviare:
   ```bash
   python3 easy_walk.py
   ```
   L'indirizzo del robot è in `main()` di `easy_walk.py` (`192.168.80.3`). Le credenziali le gestisce l'SDK (`bosdyn.client.util.authenticate`): con l'SDK originale si danno con le variabili `BOSDYN_CLIENT_USERNAME` e `BOSDYN_CLIENT_PASSWORD`, altrimenti le chiede da terminale. Attenzione: sul computer di sviluppo quel file dell'SDK è stato modificato per usare credenziali fisse; su un computer nuovo, con l'SDK appena installato, servono le variabili.

**Dimensione della missione.** Il default è in `mission_graph.py` (`MISSION_ROWS`, `MISSION_COLS`, `MISSION_CELL_SIZE_M`): oggi 3 × 4 celle da 4 m, cioè 12 m a sinistra e 16 m davanti al robot. Per una sola missione, senza toccare il codice (in ordine di precedenza):
```bash
python3 easy_walk.py --rows 2 --cols 3 --cell 4      # argomenti
SPOT_MISSION_GRID="2,3,4" python3 easy_walk.py       # variabile d'ambiente
```
La riga `[CONFIG] Griglia di missione` del log dice quale valore è stato usato e da dove.

**Grafo della missione.** I nodi si generano in due modi (`--grafo`, default in `mission_graph.GRAPH_SAMPLING`):
```bash
python3 easy_walk.py --grafo reticolo --spacing 0.25                                   # reticolo (default)
python3 easy_walk.py --grafo casuale --distribuzione poisson --densita 16 --seme 7      # nodi casuali
```
| Argomento | Default | Cosa fa |
|---|---|---|
| `--min-edge`, `--max-edge` | 0.5, 1.25 m | Lunghezza minima e massima degli archi, per tutti e due i metodi |
| `--spacing` | 0.25 m | Reticolo: passo fra i nodi, arrotondato a lato cella / n |
| `--rings` | tutti | Reticolo: anelli di vicini collegati (dentro min e max) |
| `--distribuzione` | `poisson` | Nodi casuali: `uniforme` (PRM classico, con buchi e grumi), `poisson` (distanza minima fra i nodi), `perturbata` (un punto a caso in ogni quadretto), `halton` (quasi casuale, molto uniforme) |
| `--densita` | 16 nodi/m² | Nodi casuali: quanti per m² (16 = la densità del reticolo a 0.25 m) |
| `--seme` | a caso | Nodi casuali: stesso seme, stesso grafo. Se manca se ne estrae uno; quello usato è nel log (`[GRAFO] Nodi casuali`, con il comando per ripetere) e in `mission_summary.json` |

Con i nodi casuali i centri delle celle e il punto di partenza restano comunque nodi. La copertura (quanto può distare un punto dell'area dal nodo più vicino) è misurata e scritta nel log. Per guardare un grafo prima di uscire: `python3 mission_graph.py --grafo casuale --seme 7 --max-edge 1.25 --preview grafo.png --zoom -2 2 -2 2`.

---

## Come funziona una missione

### Celle ed esplorazione
L'area è una griglia di celle (`environmentMap.py`) riferita alla posa del robot all'avvio: le **colonne** vanno in avanti, le **righe** a sinistra. Le celle si visitano a serpentina; a ogni passo si sceglie la cella di frontiera confinante di rango più basso. Se nessuna confina con la posizione attuale, il robot va con GraphNav al waypoint più vicino a quella di rango più basso e prova da lì. Una cella fallita da un lato resta candidata e si ritenta più avanti da un altro lato; diventa bloccata solo quando tutti i lati sono stati provati, o subito se l'obiettivo risulta dentro un ostacolo.

### Un tentativo di cella (`attempt_enter_cell_from_position` in `easy_walk.py`)
1. **Scansione** della griglia locale di Spot (obstacle_distance, terrain, terrain_valid): terreno corretto, celle mai scritte scartate, pendenza e rugosità, maschera degli ostacoli (`spotGrid.py`).
2. **Mappe globali** accumulate per tutta la missione: occupazione (un ostacolo conta solo dopo più osservazioni), terreno, spazio libero (`spotGrid.GlobalGrid`, `GlobalTerrainGrid`).
3. **Grafo** (`mission_graph.py`, `prm_graph.py`): i nodi sono un reticolo regolare a passo 0.25 m, deterministico, con il centro di ogni cella come nodo, oppure nodi casuali con i centri cella compresi (vedi *Grafo della missione*); gli archi vanno da 0.5 a 1.25 m. Ogni arco viene vetato o pesato con occupazione, pendenza longitudinale e laterale (misurata anche oltre i nodi, di mezzo corpo per parte) e scarto preliminare sulla scansione dal vivo. A inizio cella si rivalutano solo gli archi vicini ai dati nuovi.
4. **Obiettivo**: il centro della cella, cambiato solo se visto occupato. **Percorso** con Dijkstra, solo sulle celle già visitate, con un tetto alla deviazione.
5. **Ciclo di navigazione** con il **fronte sicuro** (`arcVerification.py`): a ogni giro una scansione fresca dice fin dove il percorso è libero adesso, e il robot avanza fino a lì, anche per più nodi di fila. Durante il movimento un thread ricalcola il fronte circa 5 volte al secondo e ferma il robot se si accorcia.
   - Il robot punta sempre il muso verso il punto da raggiungere (niente marcia di traverso). Se non può ruotare: avanza dentro il cono di rotazione possibile, oppure arretra dritto se da lì la rotazione diventa possibile, altrimenti scarta il tratto.
   - Una cella bloccante si scarta subito e si ripianifica; un blocco confermato da tre scansioni ferme diventa permanente.
   - Se nessuna direzione del giro è percorribile (trappola), il robot torna indietro con GraphNav lungo le "briciole" (waypoint lasciati ogni 0.75 m) e ripianifica da lì; dopo due ritorni nello stesso tentativo rinuncia alla cella da quel lato.
6. **Andatura** scelta tratto per tratto (`select_mobility_params`): PLAIN, MODERATE (crawl), HARD (crawl), con velocità massime per livello.

### Sicurezza
- Limiti di velocità su ogni comando, arresto a metà movimento, tempo massimo per movimento e per spostamento GraphNav.
- Rilevazione delle cadute: il robot si rialza, il punto viene segnato pericoloso nella mappa; alla terza caduta la missione si ferma.
- GraphNav riaggancia la localizzazione sul waypoint più vicino se si perde; non dichiara mai "arrivato" senza esserlo.

### Fine missione
Scansione finale, riepilogo (`[MISSION STATS]`, `[TEMPO]`), ritorno a `wp_0` con GraphNav (con scorciatoie scelte sulla strada), seduta, spegnimento, scaricamento del grafo GraphNav.

---

## Moduli

| File | Cosa fa |
|---|---|
| `easy_walk.py` | La missione: ciclo di esplorazione, tentativo di cella, ciclo di navigazione, GraphNav, log e salvataggio dati |
| `environmentMap.py` | Griglia delle celle, stato (visitata, bloccata, lati provati), serpentina e frontiera |
| `mission_graph.py` | Nodi (reticolo o casuali), coppie candidate, aggiornamento degli archi; dimensione della missione |
| `prm_graph.py` | Grafo (PRM): veti, pesi, penalità, refresh attorno al robot, Dijkstra |
| `spotGrid.py` | Griglia locale e sue correzioni, pendenza e rugosità, mappe globali, soglie e andature |
| `arcVerification.py` | Fronte sicuro: filtro sugli ostacoli, fin dove si può avanzare, thread durante il movimento |
| `movements.py` | Comandi di movimento (velocità, arresto, tempo massimo), cadute e raddrizzamento |
| `navGraphUtils.py` | Registrazione e grafo GraphNav: waypoint, chiusure d'anello, scaricamento |
| `spotLogInUtils.py`, `spotUtils.py` | Collegamento al robot, E-Stop, posizione |

---

## Interruttori

Ognuno permette un confronto acceso/spento sul campo; il log li riporta nella riga `[CONFIG] INTERRUTTORI` e `mission_summary.json` nella voce `interruttori`.

| Interruttore | Default | Cosa fa |
|---|---|---|
| `easy_walk.MISSION_GRAPH_SAMPLING` | `reticolo` | Come si generano i nodi: `reticolo` o `casuale` (vedi *Grafo della missione*) |
| `easy_walk.INCREMENTAL_GRAPH_UPDATE` | `True` | A inizio cella rivaluta solo gli archi vicini ai dati nuovi (stesso risultato della ricostruzione completa) |
| `mission_graph.BATCH_SLOPES` | `True` | Pendenze a inizio cella calcolate in blocco (stesso risultato, più veloce) |
| `prm_graph.REFRESH_BATCH_SLOPES` | `True` | Pendenze del refresh calcolate in blocco (stesso risultato, più veloce) |
| `spotGrid.CLEARANCE_MAP_DECIDES` | `False` | La mappa dello spazio libero decide la percorribilità al posto della mappa binaria |
| `easy_walk.PLAN_ONLY_EXPLORED_CELLS` | `True` | I percorsi passano solo dalle celle già visitate |
| `easy_walk.GRAPHNAV_SHORTCUTS` | `True` | Scorciatoie nel grafo GraphNav prima di ogni spostamento |

---

## Cosa si salva

Ogni missione ha un nome `Mission_<giorno-mese-anno>_<ora>` e tre cartelle:

| Cartella | Contenuto | Peso tipico |
|---|---|---|
| `MissionLogs/<missione>/` | `mission_log.txt` (tutto l'output, con l'ora a ogni riga), `segments.csv` (un tratto per riga: pendenza, andatura, inclinazione reale), `trajectory.csv` (posizione 5 volte al secondo durante i movimenti), `frontier_live.csv`, `mission_summary.json` | 0.5 MB |
| `MissionMap/<missione>/` | `scans/` (un pacchetto per scansione: layer grezzi e derivati, fronte, decisione), `*_vis.npz` (viste del grafo), `*_BLOCKED_*.npz`, mappe globali `.pkl` | 70 MB |
| `graph/<missione>/` | grafo GraphNav scaricato, con le istantanee dei waypoint | 150 MB |

---

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
- **`preflight.py`**: controlla che i moduli siano coerenti fra loro.

Ogni modifica si verifica con `test_replay.py --tutte`, `test_e2e_loop.py` e `preflight.py` prima di andare sul robot.

---

## Convenzioni

- **Coordinate**: frame VISION di Spot. x, y e direzione sono riferiti alla posa all'avvio della missione (`env.set_origin`); le quote alla quota del suolo all'avvio (`mission_z0`), perché Spot le misura da dove è stato acceso.
- **Celle**: colonne in avanti, righe a sinistra; la cella di partenza è (0,0).
- **Lati di una cella**: 4 bit, `0b1000` nord, `0b0100` est, `0b0010` sud, `0b0001` ovest.
- **Waypoint GraphNav**: `wp_0` alla partenza, poi numerati in ordine (`wp_1`, `wp_2`, …).
- **Log**: ogni riga ha un'etichetta fra parentesi quadre (`[ATTEMPT]`, `[FRONTE]`, `[BLOCCO]`, `[GRAPHNAV]`, `[TEMPO]`, …), che resta stabile: i messaggi possono cambiare, le etichette no.
