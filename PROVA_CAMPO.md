# Prova sul campo: cosa controllare

Le modifiche del 26-10 (righe con quella data) non sono mai andate sul robot; quelle dal 14-10 al 22-10 sono andate sul robot il 22-10 al chiuso e all'aperto. Alcune del 13-10 erano nel codice delle missioni delle 12:14–12:40 ma non sono mai scattate, perché la situazione non si è presentata. Questa lista dice cosa guardare e cosa aspettarsi; `mission_report.py` fa gli stessi controlli da solo, nella sezione **Controlli della prova**.

---

## Prima di partire

Tutti gli interruttori e i numeri della missione si cambiano in **`parametri.py`** (dal 27-10; prima stavano in `easy_walk.py` e in `mission_graph.py`). Quelli propri delle griglie e delle mappe restano in `spotGrid.py`, quelli del movimento in `movements.py`.

1. **Copiare tutta la cartella** sul PC che comanda il robot, non solo `easy_walk.py`. Il 05-10 una copia vecchia di `spotGrid.py` ha fermato la missione appena costruito il grafo.
2. Sul PC del robot: `python3 preflight.py` → deve dire **"Tutti i file sono allineati fra loro."**
3. **Dimensione della missione.** Si sceglie in `parametri.py`: `MISSION_GRID_ROWS`, `MISSION_GRID_COLS`, `MISSION_CELL_SIZE_M = 2`, oggi 8 × 7, cioè 16 m a sinistra × 14 m davanti al robot, 56 celle (la prova del 26-10 15:11); come la prova all'aperto del 22-10 16:04 (42 minuti di missione più 6 di chiusura). Se l'area è diversa, senza toccare il codice: `python3 easy_walk.py --rows 6 --cols 6 --cell 2`. La prima riga `[CONFIG] Griglia di missione` del log la conferma.
4. **Grafo.** Default: nodi casuali poisson, 5 nodi/m², seme estratto a caso. Le righe `[CONFIG] Grafo` e `[GRAFO] Nodi casuali` del log riportano il seme e il comando per rifare lo stesso grafo.
5. Facoltativo: **un fiducial qualunque** (AprilTag 36h11, quelli di Boston Dynamics; non un ArUco) a 1–2 m dal robot alla partenza, davanti o dietro: lo vede da qualunque telecamera. Il log lo conferma all'avvio con `[FIDUCIAL] Fiducial di partenza: tag N a X m, dietro/davanti`; senza, la missione parte lo stesso. Al ritorno su wp_0 la riga `[FIDUCIAL] Al ritorno su wp_0 rivedo il tag N … si e' spostato di D m` è la deriva dell'odometria di tutta la missione: per averla il tag deve restare dov'è.

---

## Le novità da verificare

| Novità | Riga di log | Cosa aspettarsi | Se va male |
|---|---|---|---|
| **Batteria e rientro a batteria bassa** (28-10 sera, `BATTERY_RETURN_PCT = 30`) | `[BATTERIA] avvio: carica N%`, una riga `[BATTERIA] ciclo: …` a ogni punto perso; `battery.csv`; a fine missione `[BATTERIA] Ritorno a wp_0: da A% a B%` | La carica si legge fin dall'avvio (se compare `Carica non disponibile`, annotarlo: il rientro non può scattare). Consumo per minuto e per metro sensati e stabili. Se la batteria scende sotto il 30%: `[BATTERIA] RIENTRO`, poi `Fine dell'esplorazione per batteria bassa` e la chiusura normale fino a wp_0. | `BATTERY_RETURN_PCT = 0` in `parametri.py` spegne il rientro (la misura resta). |
| **Aggiornamento locale del grafo** e **pendenze in blocco** (14-10) | `[TEMPO] aggiornamento locale del grafo: X s` (oppure `costruzione completa`) | Una sola costruzione completa, al primo tentativo; poi aggiornamenti locali da **1–3 s**. Il 13-10 12:40 erano 11 costruzioni complete da 7.7 s in media. | `INCREMENTAL_GRAPH_UPDATE = False` in `parametri.py`, `BATCH_SLOPES = False` in `mission_graph.py`: stessi risultati, solo più lenti. |
| **Chiusura d'anello di Spot solo a fine missione, completa** (26-10 sera, `GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S = 45`; archi caricati da noi) | Nessuna riga `Chiusura d'anello` prima dei viaggi; una riga `(completa): N archi in X s` prima di `[ANCHORING]` | Partenza dei viaggi GraphNav 1–2 s dopo la preparazione. A fine missione circa un minuto e una quindicina di archi (al banco 17 in 45 s su un grafo senza archi veloci). Nessuna riga `tentativo N non riuscito`; se compare `riprovo facendoli scrivere a Spot`, il caricamento nostro è stato rifiutato: annotarlo. | `GRAPHNAV_SDK_LOOP_CLOSURE = False` in `parametri.py`. Per riavere la veloce: `GRAPHNAV_SDK_LOOP_BUDGET_S = 3` (ma allora la completa finale non trova niente). |
| **Deriva dell'odometria** (26-10 sera, solo misura) | `[DERIVA] …: localizzato su wp_N, creato X min fa -> l'odometria (VISION) e' scostata di D m` a ogni arrivo GraphNav; `drift.csv` | Una riga per arrivo; la deriva cresce con l'età del waypoint (all'arrivo su wp_0 il 26-10: fino a 0.66 m dopo 25 minuti, tolleranza d'arrivo inclusa). Serve a decidere se e come correggere le mappe globali. | — |
| **Layer in più nelle scansioni** (26-10, `SAVE_EXTRA_LAYERS`) | nessuna: dopo, `intensity` e `no_step` nei pacchetti `scans/*.npz` | Presenti nei pacchetti; tempo di `scansione` nel rapporto quasi invariato. | `SAVE_EXTRA_LAYERS = ()`. |
| **Scelta dell'obiettivo** (26-10: raggiungibile, 0.33 m, 0.30 m dal bordo, rifiutati esclusi) | `[TARGET] Cella (r,c): …`, `[TARGET] Obiettivo … escluso per il resto della missione`, `NESSUN punto della cella e' raggiungibile` | Pochi "GRAFO DISCONNESSO" all'inizio del tentativo; mai lo stesso obiettivo riscelto dopo l'esclusione (il rapporto li conta). | Annotare cella e ora; `test_obiettivi.py --missioni <nome> --dettaglio`. |
| **Arrivo appena fuori dalla cella** (26-10, fino a 0.35 m) | `[OBIETTIVO] Il robot e' fuori dalla cella … vale come arrivo` | Pochi `ancora fuori dalla cella` (il 22-10 erano 44). | `ARRIVAL_OUTSIDE_CELL_M = 0` in `parametri.py` torna alla regola del 22-10. |
| **Ritorno senza movimento** (26-10) | `[RITORNO] Torno a wp_N, a X m dal robot`, `… non mi muovo, ripianifico da qui` | Nessun ritorno GraphNav di 0 m. | — |
| **Chiusura di emergenza** (28-10 sera) | `[EMERGENZA] Missione interrotta …`, `Robot seduto e motori spenti.`, `Mappa GraphNav scaricata.` | Una prova voluta, breve e in un posto sicuro: Ctrl+C a metà di una missione corta; il robot si siede e la cartella `graph/` della missione ha la mappa. | Se non si siede: E-Stop dal tablet. |
| **Avanzamento** (28-10) | `[PROGRESSO] N/56 celle chiuse (X%) … area vista Y% … ~Z min alla fine … batteria W%` | Una riga a ogni cella chiusa; la stima del tempo si stabilizza dopo qualche cella; batteria che scende in modo regolare. Alla fine `Fine esplorazione` e la riga `Avanzamento:` nel rapporto. | Solo log: se una riga è strana, annotarla. |
| **Controllo sulla posa d'arrivo** (28-10 pomeriggio: la zona non ancora vista non conta come muro) | `[ARRIVO] Il passo di … non ci sono uscite` | Rarissimo, e mai tre volte di fila sullo stesso passo (il 22-10 erano 10, il 26-10 5, al chiuso alle 12:16 6). | Annotare cella e ora: con il visualizzatore si guarda il punto d'arrivo. |
| **Obiettivo a ridosso di un ostacolo** (28-10: nuovo obiettivo alla prima fermata del fronte vicino o oltre l'obiettivo; passo finale sotto 0.40 m se basta per arrivare; nuovo obiettivo dopo un blocco vicino all'obiettivo) | `[TARGET] Il fronte si ferma … e' l'obiettivo a non andare`, `[PASSO FINALE]`, `[TARGET] Blocco a …` | Molti meno `[SUBITO]` (il 22-10 erano 58, 45 con l'ostacolo a ridosso dell'obiettivo; il 26-10 22, 18 a ridosso) e meno blocchi; mai `[REPLAN] Dopo il blocco` subito dopo `escluso per il resto della missione`. | Annotare cella e ora. Per tornare a prima: `FINAL_STEP_MIN_M = 1.0` (niente passo finale); la regola dell'obiettivo non ha interruttore. |
| **Arrivo GraphNav girato verso la cella** (28-10 sera, `GRAPHNAV_ARRIVAL_HEADING`) | `[GRAPHNAV] …: arrivo girato verso (x, y), imbardata N gradi`; nel rapporto "Rotazione dopo gli arrivi GraphNav" | Rotazione media dopo l'arrivo molto sotto i 79 gradi del 26-10 (attesa ~20–30), meno dei 38 s sul posto. Se GraphNav si gira da fermo alla fine, il guadagno è minore: guardare la traiettoria. Nessun viaggio fallito per la posa chiesta. | `GRAPHNAV_ARRIVAL_HEADING = 'waypoint'` in `parametri.py`. |
| **Ripianificazione fallita: nuovo obiettivo subito** (28-10 sera, `REPICK_ON_REPLAN_FAIL`) | `[TARGET] Ripianificazione fallita: nessun percorso da N all'obiettivo … e' isolato l'obiettivo. Ne cerco un altro`, poi `[REPLAN] Verso il nuovo obiettivo` | Molti meno `[REPLAN FALLITO] … Mantengo il percorso precedente` (il 28-10 14:23 erano 20) e meno blocchi confermati con tante direzioni libere ("Direzioni utilizzabili da qui: 21/24" seguito da "Nessuna alternativa"). Il robot non deve rinunciare a celle che prima raggiungeva. | `REPICK_ON_REPLAN_FAIL = False` in `parametri.py`. |
| **Ritorno a wp_0 corretto con il fiducial** (29-10 pomeriggio, `WP0_FIDUCIAL_CORRECTION`) | `[WP0] Secondo il tag il robot e' a D m dal punto di partenza`; se D > 0.30: `[WP0] CORREZIONE 1/2 ...`, `[GRAPHNAV] correzione 1 del ritorno a wp_0: arrivato`, `[WP0] Dopo la correzione 1: a D m dalla partenza (prima D0 m)`. Nel visualizzatore dal vivo: FASE "CORREZIONE del ritorno a wp_0" e tratto rosso | Il tag deve essere visibile dalla partenza (1-2 m). A fine missione il robot si ferma entro ~0.3 m dalla partenza, verificato a vista. | `WP0_FIDUCIAL_CORRECTION = False` in `parametri.py` (resta la misura). |
| **Tratto non concluso ma arrivato** (28-10 sera, `MOVE_NEAR_GOAL_M = 0.10`) | `[QUASI] Comando verso (x, y) non concluso, ma il robot e' a D m dal punto` | Rare, soprattutto in salita o in HARD; dopo, il robot prosegue dal nodo senza tornare indietro. Annotare se dopo un `[QUASI]` il tratto successivo parte male (segno che 10 cm sono troppi). | `MOVE_NEAR_GOAL_M = 0` in `parametri.py`. |
| **Ritorno a wp_0 fermo vicino** (28-10 sera, `GRAPHNAV_RETURN_NEAR_M = 1.0`) | `Ritorno a wp_0 NON riuscito ma vicino: fermo a D m da wp_0 (motivo: …)` | Solo se GraphNav si blocca accanto alla partenza: fiducial e deriva misurati lo stesso, ritorno comunque segnato come non riuscito. | — |
| **Controllo prima del GraphNav** (22-10 sera, `GRAPHNAV_PRECHECK`) | `[PRECHECK] Cella (r,c): dal waypoint …` | Nessuno spostamento GraphNav seguito da un fallimento immediato con "GRAFO DISCONNESSO". | `GRAPHNAV_PRECHECK = False` in `parametri.py`. |
| **Pianificazione a 0.28 m** (22-10 sera, `PLAN_CLEARANCE_EXTRA_M = 0.03`) | `[FRONTE] … ostacolo a 0.2x m (minimo 0.25 m)` | Molte meno fermate del fronte a 0.23–0.24 m (alle 15:24 erano 18 giri su 25). | `PLAN_CLEARANCE_EXTRA_M = 0` in `spotGrid.py`. |
| **Nodi casuali poisson** (22-10, prima volta sul robot) | `[GRAFO] Nodi casuali …`, `[TEMPO] … grafo …` | Grafo da ~2100 nodi e ~21 mila archi sulla 10 × 10 da 2 m, costruito in meno di un secondo; aggiornamenti locali molto più brevi di quelli del reticolo a 0.25 m. Qualche passaggio stretto (sotto ~1 m) può non avere nodi: il robot rinuncia a quel lato e prova la cella da un altro. | `--grafo reticolo --spacing 0.5` (o `0.25` per i passaggi stretti, più lento). |
| **Celle da 2 m** (22-10) | `[ATTEMPT]`, `[TARGET]` | Tentativi più corti; obiettivo quasi sempre il centro. | `--cell 4` con meno righe e colonne. |
| **Margine attorno al corpo 0.25 m** (22-10, `ROBOT_CLEARANCE_AIR_M = 0`) | `[FRONTE]`, `[BLOCCO]`, fermate di Spot vicino agli ostacoli | Il robot passa più vicino agli ostacoli (linea a 0.25 m invece di 0.30) senza toccarli; nessuna fermata di Spot stesso nei passaggi stretti. | `ROBOT_CLEARANCE_AIR_M = 0.05` in `spotGrid.py`. Annotare dove ha sfiorato o si è fermato. |
| **Refresh con le pendenze in blocco** (21-10) | `[TEMPO] scansione … \| PRM X \| …` a ogni giro | PRM sotto ~0.5 s per giro (il 13-10 12:40 erano 1.35 s di media). | `REFRESH_BATCH_SLOPES = False` in `prm_graph.py`: stessi risultati, solo più lento. |
| **Obiettivo al centro della cella** (14-10) | `[TARGET] Cella (r,c): uso il centro geometrico …` oppure `centro …, uso il punto … a D m dal centro` | Quasi sempre il centro; ripieghi entro ~1 m dal centro; nessun arrivo fuori dalla cella. | Annotare cella e ora: si guarda con `mission_viewer.py`. |
| **Pendenza col corpo** nel veto e nell'andatura (14-10) | `[GAIT] Terrain tier: …`, `[PENDENZE] costruzione: …` | Ogni tratto in cui il robot si inclina di più di 18 gradi è **HARD** (il rapporto li conta). Qualche arco vetato in più vicino ai gradini. | `ARC_BODY_EXTENSION_M` in `spotGrid.py`. |
| **Scorciatoie GraphNav sulla strada** (14-10) | `[GRAPHNAV] …: strada verso wp_X A m -> B m con N scorciatoie` | Ritorno a wp_0 molto più corto: il 13-10 12:40 erano 46.7 m e 115 s; rigiocato col codice di oggi, 18.5 m. Nessuna riga `Scorciatoia … rifiutata`. | `GRAPHNAV_SHORTCUTS = False` in `parametri.py` (si torna alla sola catena). |
| **Mappa del terreno senza celle ostacolo** (14-10) | nessuna riga: si controlla dopo, col rigioco | `test_replay.py --missioni <nome>` → invariante 2 OK (pendenza della mappa globale uguale al vivo). | — |
| **Riaggancio di GraphNav** (13-10, mai scattato) | `[LOCALIZZAZIONE] …: PERSO`, `[LOCALIZZAZIONE] Riaggancio …: riuscito su wp_N` | Se si perde, il riaggancio riesce e lo spostamento continua. Il 13-10 non si è mai perso. | Annotare dove: `[LOCALIZZAZIONE]` riporta andatura e beccheggio. |
| **Ritorni con memoria e recupero** (13-10, scattato una volta) | `[RITORNO] Arco a->b fuori dal piano …`, `[RECOVERY] L'obiettivo … è quello del tentativo appena fallito` | Mai lo stesso giro trappola → ritorno → stesso percorso più di due volte. | — |

---

## Durante la prova

- Per seguire la missione: in un secondo terminale `python3 visualizzatore_live.py` (l'ultima missione, si aggiorna da sola; `f` segue il robot o mostra tutto). Avviarlo dopo che la missione è partita. Se si chiude o si blocca, la missione non ne risente.
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

- **CRAWL in MODERATE**: il rapporto dà il beccheggio per andatura. Se i tratti MODERATE restano tranquilli (sotto ~8 gradi), si può tornare all'andatura normale (`HINT_AUTO` in `select_mobility_params`), che è più veloce.
- **Fluidità del movimento**: oggi il robot cammina in media a 0.14 m/s (13-10 12:40), perché ogni tratto aspetta che il corpo sia fermo e ruota a parte. Il rapporto lo misura nella riga **Fluidita'** (velocità media, tempo fermo a ruotare o a partire). Se la modifica viene preparata prima della prova, va confrontata qui, con l'interruttore acceso e spento.

---

## Taratura dell'andatura e dei pesi del costo

Due strumenti: **`taratura.py`** (al computer: legge i `segments.csv` delle missioni e le prove su tracciato, stampa cosa dicono i dati) e **`prova_andatura.py`** (sul robot: lo stesso tratto rettilineo percorso più volte con un'andatura fissa, una riga per passaggio in `ProveAndatura/prove.csv`). `prova_andatura.py` non è mai stato provato sul robot: la prima volta al chiuso, su 2 m in piano.

### Cosa dicono già i log (713 tratti, 14 missioni fino al 26-10)

- **Pesi del costo**: con il livello d'andatura separato, la pendenza longitudinale vale ~2 m di strada per unità (oggi β = 2: va bene); contando anche il CRAWL che fa scattare, ~5. La pendenza laterale vale solo ~0.6 in tempo (oggi 2: è un margine di sicurezza, non un costo di tempo). La rugosità, su un arco ruvido (0.03), vale circa mezzo metro: γ = 0 cambia poco.
- **Fallimenti**: 56 dei 96 sono tratti con 0 m percorsi (rifiuti, blocchi), che non dipendono dal terreno. Quelli con il robot in movimento (5.6% dei tratti) crescono sul terreno difficile: 14% sopra pendenza 0.3 contro 5% sotto, 12% con rugosità ≥ 0.02 contro 5%. Pochi casi (58 e 104 tratti): da ricontrollare con più missioni prima di metterli nel costo.
- **Soglie**: le soglie di rugosità (0.05 e 0.10) non scattano quasi mai (95° percentile 0.029). La soglia di HARD a 0.25 prende solo la metà dei tratti con beccheggio > 12°; a 0.20 il 72%.
- **Velocità**: reale 0.17 / 0.13 / 0.11 m/s contro limiti 0.5 / 0.35 / 0.25: i limiti non frenano niente.

### Al chiuso (prima di uscire)

| Cosa | Comando | Cosa aspettarsi |
|---|---|---|
| Lo script funziona | `python3 prova_andatura.py --terreno pavimento --config auto_mu08 --dist 2 --ripetizioni 1 --ritorno` | Il robot va avanti 2 m, si gira, torna, si gira; due righe in `ProveAndatura/prove.csv` con tutte le colonne piene (`slip_*`, `mu_est_*`, `energy_*` comprese). Se una colonna è vuota, annotarlo: quel dato il robot non lo dà. |
| Zero della stima | `python3 prova_andatura.py --terreno pavimento --pendenza-vera 0 --solo-stima --ripetizioni 5` | Pendenza stimata vicina a 0 (sotto 0.05, cioè 3°). Quello che resta è l'errore della stima in piano. |
| Stima su una rampa (se c'è: rampa d'accesso, pedana di carico, tavola appoggiata) | robot fermo ai piedi della rampa, rivolto verso di essa: `--terreno rampa_interna --pendenza-vera <gradi> --solo-stima` | Pendenza stimata coerente con l'inclinometro (qualche grado sopra, perché è un 90° percentile). Poi sopra la rampa: il beccheggio del corpo da fermo deve essere vicino alla pendenza vera. |
| Limite di velocità | `--terreno pavimento --config auto_mu08 --dist 5 --ripetizioni 2`, poi lo stesso con `--vel 0.3` | Se i tempi sono quasi uguali, il limite non è ciò che rallenta il robot (conferma dei log). |
| Attrito stimato | due passaggi su fondi diversi (piastrelle, tappeto di gomma) | `mu_est_mean` diverso fra i due fondi: dà un'idea dei valori che Spot stima. |

Al chiuso non si prova: erba, zolle, fondo bagnato, pendenze vere oltre 10°.

### Fuori: materiale

- Inclinometro (app del telefono va bene) e un'asta o una tavola dritta di circa 1 m da appoggiare a terra.
- Metro a nastro, 4 coni o picchetti, nastro colorato.
- Due batterie se possibile: il programma completo richiede circa 2 ore di robot acceso.
- Tablet con E-Stop sempre in mano: in `prova_andatura.py` non c'è il fronte sicuro, il robot va dritto.
- Foglio per le note (la scheda qui sotto).

### Fuori: i tracciati

Cinque tracciati rettilinei da **5 m**, puliti da sassi e rami, con un metro libero in più alla fine e ai lati (il robot si gira sul posto alla fine). In ordine di importanza:

| # | Nome (`--terreno`) | Dove | Perché |
|---|---|---|---|
| 1 | `rampa_media` | pendenza costante di **10–15°** in salita | È dove si decide MODERATE contro HARD (0.15 = 8.5°, 0.20 = 11.3°, 0.25 = 14°). |
| 2 | `prato` | prato in piano | È il terreno di quasi tutta la missione: se CRAWL non serve qui, la missione è molto più veloce. |
| 3 | `laterale` | lo stesso pendio, percorso di traverso (pendenza laterale) | Rollio: giustifica o meno β_lat = 2. |
| 4 | `rampa_ripida` | **18–25°**, se c'è | Il limite: dove l'andatura normale smette di andare bene. |
| 5 | `zolle` | terreno sconnesso, zolle, radici | Rugosità vera: serve per sapere che valori dà la stima. |

Per ogni tracciato, **prima di tutto**:
1. Misurare la pendenza con l'inclinometro in 3 punti (inizio, metà, fine), con la tavola appoggiata lungo il tracciato; scrivere la media. Per `laterale`, misurare di traverso.
2. Segnare inizio e fine (5 m) con i coni; robot all'inizio, muso verso la fine.
3. `python3 prova_andatura.py --terreno <nome> --pendenza-vera <gradi> --solo-stima --ripetizioni 3` (circa 30 s): pendenza stimata contro misurata.

### Fuori: le configurazioni, passo per passo

Su ogni tracciato, tre configurazioni, **3 ripetizioni ciascuna con `--ritorno`** (sulla rampa il ritorno è la discesa: è un dato in più). Circa 15 minuti per tracciato.

```
python3 prova_andatura.py --terreno <nome> --pendenza-vera <gradi> --config auto_mu08 --ripetizioni 3 --ritorno
python3 prova_andatura.py --terreno <nome> --pendenza-vera <gradi> --config crawl_mu06 --ripetizioni 3 --ritorno
python3 prova_andatura.py --terreno <nome> --pendenza-vera <gradi> --config auto_alto_mu06 --ripetizioni 3 --ritorno
```

- `auto_mu08` = PLAIN di oggi; `crawl_mu06` = MODERATE di oggi; `auto_alto_mu06` = MODERATE senza crawl: dice se il crawl serve davvero.
- Su `rampa_ripida` aggiungere `crawl_mu04` (= HARD di oggi) e `auto_alto_mu04`.
- **Ordine**: cambiare l'ordine delle configurazioni da un tracciato all'altro (prima `crawl` su un tracciato, prima `auto` sul successivo): il terreno si segna con i passaggi e la batteria cala.
- **Sicurezza**: su `rampa_ripida` fare prima un solo passaggio con `crawl_mu04` (`--ripetizioni 1`); se il robot scivola o si inclina oltre 25°, non provare le altre su quel tracciato. Se il robot inciampa o scivola in un passaggio, annotarlo e passare alla configurazione successiva.
- Se la batteria scende sotto il 35%, saltare `zolle` e tenere `rampa_media`, `prato` e `laterale`.

Al termine, se resta batteria: una **missione normale breve** (`python3 easy_walk.py --rows 4 --cols 4`) sul terreno più vario disponibile, per avere altri tratti per la regressione del costo.

### Scheda da compilare (una riga per tracciato)

| Tracciato | Pendenza misurata (°) | Stima (°) | Note sul fondo (asciutto, bagnato, erba alta…) | Passaggi con scivolate o inciampi (config, ora) | Impressione a vista (quale andatura sembra più sicura) |
|---|---|---|---|---|---|
| rampa_media | | | | | |
| prato | | | | | |
| laterale | | | | | |
| rampa_ripida | | | | | |
| zolle | | | | | |

### Dopo: come si decide

1. `python3 taratura.py --prove ProveAndatura/prove.csv`: sezione 5.
2. **Stima della pendenza**: se la stima è sistematicamente più alta o più bassa della misura, le soglie vanno lette con quella correzione (non si corregge la stima al buio).
3. **Crawl in MODERATE**: su `rampa_media` e `prato`, se `auto_alto_mu06` riesce sempre con RMS di beccheggio e rollio e slittamento simili a `crawl_mu06`, il crawl in MODERATE non serve (`HINT_AUTO` in `select_mobility_params`): missione più veloce.
4. **Soglia di HARD**: la pendenza (stimata) a cui l'andatura normale comincia a dare RMS o slittamenti chiaramente più alti, o a non riuscire, è la soglia `GAIT_MODERATE_SLOPE_MAX`. I log indicano 0.20 al posto di 0.25.
5. **`ground_mu_hint`**: confrontarlo con `mu_est_mean` (l'attrito che Spot stima sotto i piedi) per fondo: va scelto in base al fondo (erba asciutta, bagnata), non alla pendenza.
6. **Rugosità**: le soglie vanno messe sui valori stimati su `zolle` contro `prato` (oggi non scattano mai).
7. **Pesi del costo**: dopo aver cambiato l'andatura, rifare `python3 taratura.py --dal <data>` sulle missioni nuove (sezione 2): il costo della pendenza dipende da come il robot la percorre.
8. Ogni modifica: `test_replay.py`, poi CHANGELOG con i numeri della prova.
