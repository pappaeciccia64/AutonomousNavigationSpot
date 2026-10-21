# Storia delle modifiche

Le modifiche più recenti in alto. Per ognuna: cosa è cambiato, perché, e il dato di campo che l'ha motivata. Il codice descrive solo come funziona oggi; da qui si capisce come ci si è arrivati. Il dettaglio riga per riga è nella storia di git.

Le missioni sono indicate come `giorno-mese ora` (per esempio `13-10 12:40`); i loro dati stanno in `MissionLogs/` e `MissionMap/`.

---

## 2026-10-21

### Pianificatore (`prm_graph.py`)
- **Pendenze del refresh calcolate in blocco** (`REFRESH_BATCH_SLOPES`, come `mission_graph.BATCH_SLOPES` a inizio cella). Dopo il 14-10 il refresh a ogni scansione era diventato la parte più lenta del grafo: rigiocando la missione 13-10 12:40 col codice di oggi, 1.08 s per scansione contro 1.38 s per inizio cella. Grafo identico bit per bit al calcolo arco per arco (archi, pesi, validità, pendenze, cache della pendenza globale) a ogni scansione di cinque missioni, 340 su 340. Tempi sullo stesso rigioco (59 scansioni, 11 inizi di cella, una variante alla volta):

  | variante | inizio cella | refresh | totale |
  |---|---|---|---|
  | tutto arco per arco, ricostruzione completa | 76.8 s | 62.7 s | 141 s |
  | aggiornamento locale | 73.3 s | 62.6 s | 138 s |
  | + pendenze in blocco a inizio cella (14-10) | 15.2 s | 63.5 s | 81 s |
  | **+ pendenze in blocco nel refresh** | **15.2 s** | **17.4 s** | **34 s** |

  In missione il refresh si fa a robot fermo fra un passo e l'altro: da ~1.1 a ~0.3 s di sosta per scansione.

### Mappe e terreno (`spotGrid.py`)
- **Lunghezza dell'arco calcolata con `x*x` invece di `**2`** nel profilo di pendenza arco per arco (dal vivo e dalla mappa globale). Su un numero singolo `**2` a volte differisce all'ultima cifra dall'operazione che numpy fa sugli array, e da lì passo e pendenza: sulla missione 12-10 14:47 sette archi avevano pendenze diverse di 1e-16 fra calcolo arco per arco e in blocco. Ora i due coincidono bit per bit anche lì (il 14-10 il confronto era stato fatto solo sulle missioni del 13-10).

### Strumenti
- **Messaggi di log in italiano** in tutti i moduli (`easy_walk.py`, `environmentMap.py`, `navGraphUtils.py`, `spotGrid.py`, `spotLogInUtils.py`). Le etichette fra parentesi quadre restano le stesse, così le ricerche nei log continuano a funzionare. `mission_report.py` e `test_replay.py` leggono sia i messaggi nuovi sia quelli in inglese delle missioni registrate prima.
- **Controlli della prova in `mission_report.py`**, con la lista `PROVA_CAMPO.md`: per ogni novità mai provata sul robot, se è scattata e se i numeri sono quelli attesi.
- **README riscritto** in italiano e al presente: avvio, funzionamento di una missione, moduli, interruttori, dati salvati, strumenti. Il precedente (20-05) descriveva il progetto di allora.
- Default della missione in `mission_graph.py` scritto in chiaro: 3 × 4 celle da 4 m (prima `3 #None`, con il commento del vecchio default 2 × 4 da 5 m).
- **Interruttori nel log e nel riepilogo**: riga `[CONFIG] INTERRUTTORI` all'avvio e voce `interruttori` in `mission_summary.json` (aggiornamento locale, pendenze in blocco a inizio cella e nel refresh, mappa dello spazio libero, scorciatoie GraphNav, …); il rapporto li mostra. Serve ai confronti acceso/spento sul campo.
- **Riga "Fluidita'" nel rapporto**: velocità media e tempo fermo a ruotare o a partire. Missione 13-10 12:40: 0.14 m/s su 356 s di movimento, di cui 93 s (26%) fermo a ruotare o a partire.

---

## 2026-10-20

### Movimenti (`movements.py`)
- Tolto il parametro `stairs` di `relative_move`: non veniva mai letto e nessuno lo passava, quindi faceva credere a una modalità scale che non esiste.

---

## 2026-10-14

### Fronte sicuro (`arcVerification.py`)
- Tolta `AsyncArcVerificationTracker`, mai usata (chiamava un servizio che non esiste).

### Pianificatore (`prm_graph.py`)
- **Il termine di pendenza del costo resta fisso per arco, non per metro** (decisione dell'utente: il pianificatore deve preferire archi lunghi, e il termine per arco lo fa). Rigiocando le missioni del 13-10 (36 partenze da cella), un costo per metro cambiava 32 percorsi su 36: esposizione alla pendenza −9%, ma archi +34% (da 144 a 193). Se un giorno il terreno diventa davvero ripido, la forma concordata è pendenza per metro più una penalità esplicita per arco.
- **Pendenze di molti archi in un colpo solo** (`precompute_arc_slopes`, con `spotGrid.arc_slope_profiles_batch`). Numeri identici bit per bit su 421 mila archi delle missioni del 13-10, circa 13 volte più veloce. Prima l'aggiornamento del grafo passava il 90% del tempo in decine di migliaia di chiamate su array di poche decine di numeri.

### Mappe e terreno (`spotGrid.py`)
- **`SLOPE_THRESHOLD` letto anche come limite sui gradini.** La pendenza si misura su una base di 0.48 m, quindi un gradino netto di altezza H vale H / 0.48, qualunque sia la lunghezza dell'arco: 0.6 corrisponde a circa 29 cm, il limite di Spot (~30 cm) senza margine. Sul campo (13-10) il gradino più alto superato è stato di ~25 cm (pendenza 0.54). Le soglie delle andature, riespresse così: PLAIN sotto ~7 cm (0.15), HARD da ~12 cm (0.25); il gradino del 13-10 12:47:20 (~17–19 cm) è HARD.
- **L'allungamento di mezzo corpo (`ARC_BODY_EXTENSION_M`) vale anche per il veto del PRM**: il robot non deve potersi fermare su un nodo con le zampe su un pendio oltre soglia. Sulle mappe del 13-10 12:40: archi scartati dallo 0.77% all'1.19%, nodi isolati da 1 a 5. Il fronte resta sull'arco fra i nodi, quindi è più permissivo del PRM e non può bloccare un arco che il PRM ha accettato per pendenza.
- Corretto un commento che dava la base della pendenza a ~25 cm: la differenza è centrata, 8 celle avanti e 8 indietro, cioè ~0.48 m (vedi anche 06-10 sera).

### Navigazione (`easy_walk.py`)
- **Scorciatoie GraphNav scelte sulla strada** (`plan_graphnav_shortcuts`). Prima si aggiungevano fino a 30 scorciatoie fra waypoint a meno di 2 m, le più corte per prime: con celle da 4 m e il giro a serpentina, fra una riga di celle e l'altra quasi nessuna coppia stava entro 2 m, e il ritorno a wp_0 del 13-10 12:40 ha ripercorso tutta la missione: 46.7 m nel grafo (linea d'aria 13.1 m), 115 s. Ora si calcola la strada più corta verso la destinazione con tutte le scorciatoie possibili fino a 3 m e si creano solo quelle che la strada usa. Rigiocato sui grafi scaricati del 13-10 (`test_replay.py`, controllo 5): ritorno delle 12:40 da 46.7 a 18.5 m, 12:14 da 18.8 a 17.3 m, 12:27 da 18.0 a 15.8 m. Margine dalle celle occupate portato a 0.30 m (0.45 m dall'ostacolo), perché gli archi sono più lunghi.
- **Un solo punto per mettere una scansione nelle mappe globali** (`fold_scan_into_global_maps`), usato dalla missione e da `test_replay.py`, così il rigioco verifica il codice vero. La mappa del terreno riceve la stessa maschera del calcolo dal vivo: prima salvava anche le quote delle celle ostacolo, e tronchi e muri diventavano pendenze. Rigiocando le scansioni del 13-10 12:40, sulle stesse 1885 coppie, archi oltre 0.6: 0.0% dal vivo contro 10.9% dalla mappa globale.

---

## 2026-10-13

### Pianificatore (`prm_graph.py`)
- **Scarto preliminare sulla scansione dal vivo** (`LIVE_PREFILTER_OD_M` = 0.20 m). Prima un arco si scartava per occupazione solo dalla mappa globale, che conferma un ostacolo dopo 2 osservazioni (4 dove il robot ha camminato) e non scrive l'anello attorno al corpo: alla prima scansione di un posto gli archi che lo attraversavano restavano tutti validi. Missione 13-10 11:22, inizio del tentativo in cella (0,1): 3160 archi validi su 9038 nella finestra passavano dentro un ostacolo (`obstacle_distance` < 0) e 4575 sotto 0.20 m; ai giri successivi 20–110. 0.20 m è mezza larghezza del corpo meno 5 cm di rumore. Sugli 85 passi eseguiti nelle missioni del 12-10 14:23 e 17:32 e del 13-10 11:22 nessuno sarebbe stato scartato, nemmeno a 0.25.
- Lo stesso scarto si applica agli archi validi della finestra che stanno fuori dal raggio di refresh (`live_prune_valid_edges`): prima un arco a 1.3–1.9 m dal robot che attraversava un ostacolo appena visto restava valido finché il robot non si avvicinava.
- **Aggiornamento locale del grafo.** All'inizio di ogni cella non si rivalutano più tutti i ~94 mila archi, solo quelli vicini alle zone in cui sono arrivati dati nuovi (`dirty_rects`, le finestre delle scansioni).
- **Log delle pendenze riassunto.** Le righe `[SLOPE-VETO]` e `[SLOPE-FLIP]` erano 1338 su 3155 nella missione del 12-10 17:32, il 42% del log. Ora c'è una riga `[PENDENZE]` per costruzione o refresh; i valori arco per arco restano nei pacchetti `_vis.npz`.

### Mappe e terreno (`spotGrid.py`)
- **Pendenza "col corpo"** (`ARC_BODY_EXTENSION_M`): per costo e andatura l'arco si allunga di mezzo corpo a ogni estremo, perché il robot fermo su un nodo ha le zampe 0.55 m avanti e indietro. Missione 13-10 12:40, ore 12:47:20: un gradino di ~17 cm stava 20 cm oltre la fine dell'arco, l'arco misurava 0.17 (MODERATE) e il robot è rimasto fermo 9 s col muso a 27 gradi, zampe anteriori sul gradino. `_body_slope` legge solo i pezzi aggiunti: ricampionare l'arco allungato da capo raddoppiava la costruzione del grafo.
- **90° percentile calcolato a mano**, identico a `np.percentile` bit per bit. `np.percentile` costava ~130 µs di solo contorno per chiamata: 121 mila chiamate, 15.7 s su 35 della costruzione del grafo.

### Movimenti (`movements.py`)
- Messaggio esplicito dopo una rotazione sul posto. Prima diceva "Arrived at the goal (traveled 0.03m)", e i 3 cm (il centro del corpo che si sposta girando) sembravano un avanzamento minuscolo.

### Navigazione (`easy_walk.py`)
- **Localizzazione di GraphNav.** Missioni del 12-10 17:16 e 17:32, all'aperto: al primo uso di GraphNav il robot era già perso ("Cannot issue a navigation request when the robot is already lost"); nella prima questo ha chiuso l'esplorazione (tre spostamenti rifiutati). Sui grafi scaricati la mappa era sana: odometria delle zampe e visione differivano al massimo di 13–14 cm su 34–49 m (come nella missione delle 14:47, che è tornata), e le nostre scorciatoie coincidevano con la visione entro 6 cm. A perdersi è il localizzatore dal vivo. Aggiunti la riga `[LOCALIZZAZIONE]`, il riaggancio sul waypoint più vicino con stima dall'odometria (senza dichiarare l'arrivo), il rilevatore di perdita permissivo, e il controllo del localizzatore prima di partire, invece di farsi rifiutare il comando tre volte.
- **Ritorni con memoria per il tentativo** (`_trap_backtrack`, `MAX_BACKTRACKS_PER_ATTEMPT` = 2). Missione 13-10 11:22, cella (1,2): dopo una trappola il robot tornava con GraphNav alla briciola precedente, ritrovava identico lo stesso percorso (nessun arco scartato, contatori azzerati) e rientrava nella stessa trappola: 6 giri prima che lo fermassimo a mano.
- **Recupero: niente secondo tentativo sullo stesso obiettivo** (`RECOVERY_SAME_GOAL_M`). Stessa missione: il recupero ritrovava l'obiettivo appena fallito, raggiungibile "con 1 arco" (proprio quello della trappola), e rientrava subito nel tentativo.
- **Regola del centro** (`find_best_point_in_cell`): il centro geometrico è l'obiettivo, libero o ignoto; si cambia solo se visto occupato o già scartato dal vivo. Prima bastava un punto già visto libero per battere il centro ignoto: era l'ultimo lembo scansionato dalla cella precedente, sul bordo, a ~2 m dal centro. Missioni del 13-10 12:14, 12:27 e 12:40: 20 ingressi su 38, con tre arrivi fuori dalla cella.
- **Riscelta dell'obiettivo ad almeno 0.30 m dagli ostacoli**, guardando anche la scansione dal vivo e lo spazio libero imparato; a parità di distanza dal centro vince il punto più vicino al robot. 13-10 12:40: le riscelte cadevano a 0.35–0.50 m dal punto scartato, ancora dentro la siepe (`obstacle_distance` −0.04, 0.06, 0.27); in cella (2,0) c'erano due punti a 0.50 m dal centro, uno oltre la siepe e l'altro dal lato del robot, quello da cui poi è entrato.
- La scansione entra nello scarto preliminare degli archi (`set_live_obstacles`): grezza all'inizio del tentativo, filtrata nel ciclo, tolta dopo un ritorno con GraphNav (era stata fatta da un altro punto).

---

## 2026-10-12

### Fronte sicuro (`arcVerification.py`)
- **Celle sotto il corpo mai ostacolo** (`_under_body`). Missione 12-10 10:36: il fronte si è fermato due volte, a metà movimento e da fermo, su un punto a 8–9 cm dal centro del robot mentre passava rasente a un tavolo (`obstacle_distance` sotto il robot 0.21 m contro 0.25 di mezza larghezza).
- **Due soglie: 0.30 m per partire, 0.25 m per fermarsi a metà movimento** (`air_m`, `FRONTIER_ABORT_AIR_M`). Richiesta dell'utente: un tratto dichiarato libero si percorre; a movimento in corso ci si ferma solo per un'intrusione vera.
- **Nel log la soglia effettiva e se ha deciso l'isteresi.** Missione 12-10 14:47: 23 fermate su 33 dicevano "ostacolo a 0.32 m (minimo 0.30 m)", che sembrava una contraddizione.
- Regola di uscita: sotto il corpo il riferimento è la distanza del robot adesso, non le letture sotto la pancia.

### Pianificatore (`prm_graph.py`)
- **Refresh a piena fedeltà solo entro 1.25 m dal robot** (`REFRESH_RADIUS_M`). Il refresh rivalutava ~8500 archi a giro, 3.65 s: era la voce di tempo più grossa della missione (37.5%), perché sul prato il 53% delle letture di pendenza trova dati veri contro l'8.6% al chiuso. Il robot nel passo successivo ne percorre uno, e avanza di ~1.2 m per volta. Missione 12-10 14:47 (1 costruzione + 27 refresh), confronto con la valutazione piena:

  | raggio | tempo | verdetti diversi |
  |---|---|---|
  | 2.00 m | 0.80 s | 1 |
  | 1.50 m | 0.58 s | 79 |
  | **1.25 m** | **0.49 s** | **80** |
  | 1.00 m | 0.38 s | 654 |

  1.25 m è anche `PRM_MAX_EDGE_M`: l'arco che il robot sta per percorrere ricade sempre per intero nella zona rivalutata.
- **Costruzione del grafo più veloce, risultati identici.** Missione 12-10 10:36: dei 16 s di costruzione, 7 erano il controllo d'occupazione (un milione di chiamate a `is_occupied`, quasi tutte su terreno mai visto) e l'altra metà la pendenza dalla mappa globale. Ora l'occupazione si legge in blocco (`_occupancy_blocked_many`), gli archi certamente senza dati di pendenza vanno in cache subito (`_prefill_global_slope_nodata`), e gli archi in zona senza dati prendono direttamente il costo di ripiego, con la stessa formula (3 archi su 4 del reticolo).
- Con `gamma` = 0 la rugosità non si campiona per il costo: era il 10–15% del tempo di costruzione.
- Il centro del raggio di refresh è il nodo del robot (se non è noto, il centro della finestra).
- Per l'occupazione si calcolano sia la mappa binaria sia la mappa dello spazio libero; decide `spotGrid.CLEARANCE_MAP_DECIDES`, l'altra serve a misurare il disaccordo (riga `[SPAZIO]`).

### Mappe e terreno (`spotGrid.py`)
- **Celle su cui il robot è passato: servono più conferme per dichiararle occupate** (`GLOBAL_OCC_CONFIRM_WALKED`). Due osservazioni di ostacolo battevano qualunque numero di osservazioni di spazio libero. Missione 12-10 14:47, cella (1,1): una cella su cui il robot era rimasto fermo per 16 scansioni, con 0.35–0.38 m di spazio sotto di sé, è stata dichiarata occupata da due letture prese da 1.8 e 2.5 m (0.121 e 0.109 m). In 16 scansioni su 72 la mappa dichiarava occupata la cella in cui stava il robot; 35 scansioni e 5 blocchi su quella cella. Rigiocando le 72 scansioni contro le viste da meno di 0.8 m: falsi ostacoli da 274 a 193 (−30%), ostacoli non visti da 377 a 384, "il robot era qui ed è occupata" da 16 a 0. L'alternativa (non azzerare mai le prove di libero) dava 173 falsi ma 490 non visti: scartata.
- **Mappa dello spazio libero** (`GlobalGrid` impara la distanza dagli ostacoli per cella). La mappa d'occupazione sa solo "ostacolo sì/no": un varco da 0.26 m per lei è prato aperto, mentre il fronte pretende 0.30. Missione 12-10 14:47, cella (1,1): il pianificatore ha riproposto lo stesso varco otto volte, un arco accanto all'altro; 336 scarti di nodi per 88 nodi davvero cattivi. Statistica scelta: media delle osservazioni vicine (`CLEARANCE_NEAR_M`), altrimenti di tutte. Sulle due missioni del 12-10 (fermate del fronte riconosciute / punti percorsi che verrebbero vietati):

  | criterio | esterno | interno |
  |---|---|---|
  | bit + margine (attuale) | 39% / 6.1% | 43% / 16.0% |
  | **vicine, se no tutte** | **52% / 1.9%** | **86% / 16.5%** |
  | media di tutte | 52% / 1.3% | 21% / 16.0% |

  Per ora `CLEARANCE_MAP_DECIDES = False`: la mappa impara e si misura il disaccordo, decide ancora la mappa binaria. Si commuta dopo una missione di prova.
- **Soglia HARD dell'andatura da 0.35 a 0.25**, sui numeri della prima missione in esterno. Con 0.35 HARD non è mai scattato (48 passi su 48 fra PLAIN e MODERATE): il gradiente massimo sul prato è stato 0.330. Sui 48 archi percorsi: p25 0.132, mediana 0.171, p75 0.203, p90 0.248. Con 0.25: PLAIN 29%, MODERATE 60%, HARD 10%, cioè HARD sul decimo più ripido, come voleva l'utente.
- `occupied_many` e `certainly_no_slope_data`: le versioni in blocco di `is_occupied` e del controllo "nessun dato" della mappa globale, con risultati identici punto per punto (il 98% delle chiamate in costruzione finiva in "nessun dato").

### GraphNav (`navGraphUtils.py`)
- **Niente scaricamenti del grafo solo per un nome.** Il numero del waypoint successivo si ricava dai waypoint creati da noi, e nome e id dell'ultimo arrivano dalla risposta del robot (`last_created_waypoint`). Missione 12-10 14:47: 41 scaricamenti completi su 60 servivano solo a questo, con un costo che cresce con la mappa (70 waypoint a fine missione).

### Navigazione (`easy_walk.py`)
- **Tempi di GraphNav e della chiusura nel riepilogo `[TEMPO]`.** Missione 12-10 14:47: 212 s su 806 di orologio (26%) non finivano in nessuna fase misurata; la fetta più grossa era GraphNav (83 s il solo ritorno a wp_0), il resto la chiusura dopo il riepilogo.
- **Anche MODERATE in CRAWL** (era `HINT_AUTO`), richiesta dell'utente dopo la prima missione in esterno. Sul prato MODERATE prende ~60% dei passi, quindi il crawl diventa l'andatura prevalente e la missione rallenta.
- **Nodi negli ostacoli giudicati anche sulla mappa dello spazio libero** (`prune_prm_nodes_in_obstacles`, quando `CLEARANCE_MAP_DECIDES`). La finestra della scansione contiene ~235 nodi su 3321 (7%), e lo scarto è soft: al refresh successivo gli archi tornavano, 336 scarti per 88 nodi davvero cattivi.
- **Nei pacchetti di scansione la `obstacle_distance` filtrata e il latch**: senza, un rigioco offline non riproduce i blocchi che nascono dalle letture del thread durante il movimento (scansione 13 del 12-10 10:36).

---

## 2026-10-08

### Pianificatore (`prm_graph.py`)
- **Strato dei costi al posto della rimozione di nodi e archi.** Decisione dell'utente, con i professori: "non modificare il grafo a livello di nodi; cosa cambia invece sono i costi". Ciò che si impara in missione diventa una penalità che Dijkstra somma al costo geometrico; impercorribile vuol dire penalità infinita, ma l'arco resta nel grafo. Nelle missioni del 07-10 la rimozione aveva condannato per sempre 8 archi attorno a un nodo per un ostacolo marginale a 9 cm, e il nodo è rimasto isolato per il resto della missione. Penalità persistenti (blocco confermato, fallimento del movimento, trappola) e transitorie (ricalcolate a ogni scansione).
- **Tetto sul numero di archi di un percorso.** Richiesta dell'utente: "non può fare percorsi troppo lunghi sulle celle già visitate". Si verifica sul percorso trovato: se il minimo è oltre il tetto, si rinuncia alla cella.
- **Tetto alla deviazione.** Missione 08-10 15:09: con il robot a 2.4 m dall'obiettivo il pianificatore aveva proposto un giro di ~15 m.
- Il costo del piano in corso si calcola con gli stessi costi di Dijkstra, penalità comprese.

### Grafo di missione (`mission_graph.py`)
- **Reticolo regolare al posto del campionamento casuale** (`global_sampler.GlobalSampler`, ora tolto). Prima: 5 punti/m², ~3125 nodi su 25×25 m, nessun seed, grafo diverso a ogni missione, ~85 600 archi da valutare (~10 s per costruzione, 3–4 costruzioni per missione). Il reticolo a passo 0.714 m con due anelli ne ha ~4700, 18 volte meno, ed è riproducibile bit per bit.
- **Rettangolare, non triangolare.** Il triangolare non può avere i centri delle celle come nodi (servirebbe che 2n/√3 fosse un intero pari). Le alternative erano spostare l'obiettivo sul nodo più vicino (fino a 50 cm) o inserire i centri e togliere i nodi vicini, che lasciava il reticolo irregolare attorno a 20 celle su 25, con archi da 0.57 a 1.44 m.
- **Passo da 0.5 a 0.25 m, archi dal secondo anello (≥ 0.5 m).** Missione 08-10 15:40: il corridoio di partenza aveva ~45 cm di linee centrali ammesse e le due file del reticolo a passo 0.5 cadevano entrambe fuori, quindi nel grafo il corridoio non esisteva. A passo 0.25 ci sono due file dentro. Gli anelli sono stati ridefiniti come cerchio di raggio k·s·√2, per coerenza con il refresh del PRM.

### GraphNav (`navGraphUtils.py`)
- **Chiusure d'anello durante la missione** (`close_loops_checked`), con fiducial e odometria e il controllo di collisione di Spot sugli archi nuovi. Prima si chiudevano solo a fine missione e solo con i fiducial: il grafo restava una catena e ogni ritorno ripercorreva tutta la strada fatta.

### Controllo pre-missione (`preflight.py`)
- Tabella aggiornata: `MAX_MANEUVERS_SAME_EDGE` non esiste più da quando è stata tolta la marcia di traverso (07-10), e `preflight` bocciava una copia corretta di `easy_walk.py`.

### Mappe e terreno (`spotGrid.py`)
- **Tolleranza sulla rotazione da 0.05 a 0.15 m.** Richiesta dell'utente: "il padding dall'ostacolo non deve bloccarlo nella rotazione, a volte lo spazio ci sarebbe, con GraphNav riesce". Missione 08-10 10:47, 46 scansioni, direzioni del giro insieme libere e raggiungibili girandosi:

  | tolleranza | direzioni usabili (mediana) | scansioni con zero |
  |---|---|---|
  | 0.00 | 3 | 7/46 |
  | 0.05 | 6 | 6/46 |
  | 0.10 | 6 | 6/46 |
  | 0.15 | 6 | 0/46 |

  Le sei scansioni in trappola avevano le direzioni libere rifiutate a −0.12/−0.15 m. A 0.15 si accetta che il corpo entri di 15 cm nella nostra mappa: l'errore di registrazione della griglia è 5–10 cm, e GraphNav in quei punti ruota e passa. Se in prova si vedono sfioramenti durante le rotazioni, è il primo numero da riportare a 0.10.
- (sera, dopo la revisione) **Rotazione: si misura la penetrazione nel corpo, non il minimo di `obstacle_distance`.** Dentro un ostacolo `obstacle_distance` non scende sotto metà del suo spessore, quindi con la tolleranza a 0.15 qualunque ostacolo più sottile di ~30 cm che entrava nel corpo veniva approvato (pali da 6, 10, 20 e 30 cm in prova sintetica). Le celle sotto il corpo adesso si escludono. Sui 47 scan della missione 08-10 16:35, 24 rotazioni per scan: stesso verdetto nel 97.8%, i 24 cambiati tutti da approvata a rifiutata, quasi tutti al limite (−0.150 → −0.151).

### Navigazione (`easy_walk.py`)
- **Archi fra 0.5 e 1.25 m** (`rings=None`, `PRM_MAX_EDGE_M` = 1.25), regola scelta dall'utente dopo la missione 08-10 16:35. Con il tetto a 0.707 m un ingresso cella diventava una catena di 16–19 waypoint, ognuno una rotazione, nel corridoio da 1.5 m in cui ruotare era proprio ciò che non si poteva fare (8 giri di ciclo in 3.2 s senza muoversi). Costo misurato sulla mappa globale vera di quella missione, 3321 nodi:

  | archi | coppie | ammessi | costruzione | refresh |
  |---|---|---|---|---|
  | 0.5–0.707 m | 25 120 | 18 853 | 0.92 s | 0.12 s |
  | **0.5–1.25 m** | **109 474** | **79 229** | **4.72 s** | **0.43 s** |

  Sulla missione delle 16:35 (4 costruzioni, 47 scansioni) sono ~39 s invece di ~9 s: prezzo accettato, in parte restituito dal refresh a eventi. `PRM_MAX_EDGE_M` segue il reticolo e non resta a 2.0, perché il refresh ricostruisce le coppie con il `max_edge_length` del PRM e con 2.0 si riempirebbe di archi lunghi solo attorno al robot.
- **Percorsi solo nelle celle visitate** (`PLAN_ONLY_EXPLORED_CELLS`). Missione 08-10 15:09: per entrare in (0,2) il pianificatore passava dalla riga 1, mai esplorata.
- **Guardie anti-stallo.** Missione 08-10 16:35, iterazione 2: otto giri in 3.2 s con il robot immobile. Rigiocando: la rotazione chiedeva quasi 180 gradi (l'obiettivo stava dietro un muro), il settore la rifiutava (−0.17/−0.19 m), il cono non trovava direzioni, e le direzioni utilizzabili erano 3 su 24, quindi niente "trappola". `MAX_ROTATION_DISCARDS` conta per tentativo: `MAX_ROTATION_FAILS_SAME_EDGE`, indicizzato sulla coppia di nodi, ripartiva da 1 a ogni ripianificazione e non è mai scattato. `MAX_IDLE_LOOPS` è la rete generica. (sera, dopo la revisione) La prima versione diceva 12 "sopra ~8" ed era sotto la sequenza legittima più lunga a robot fermo: ricalcolata sui limiti del ciclo, 26 giri, quindi 30.
- (sera) **Rifiuti di rotazione azzerati solo sui progressi** (20 cm più vicini all'obiettivo). Con l'azzeramento a ogni spostamento, nel banco di prova S11 (corridoio da 0.80 m con l'obiettivo dietro) il robot è andato un nodo a est dopo l'altro: 35 scansioni e 4.5 m allontanandosi, con il contatore sempre a zero.
- **Avanzamento multinodo.** Il fronte percorre tutto il percorso, ma si guardava solo il primo nodo e il robot si fermava a riscansionare a ogni nodo. Missione 08-10 16:35: il fronte confermava 1.9–3.2 m con archi da 0.5–0.7 m, tre o quattro nodi avanti ignorati. (sera) Dopo ogni gamba si rilegge la posa vera: con l'orientamento della scansione iniziale il banco di prova misurava 18 cm di scarto su un solo movimento a 2 nodi.
- **Refresh a eventi.** Prima il refresh degli archi girava a ogni giro di ciclo. Missione 08-10 16:35: 22 scansioni su 46 (48%) con uno spostamento sotto i 25 cm, ~9.5 s ricalcolati per riottenere gli stessi numeri.
- **Il cono punta al percorso, non all'obiettivo** (`CONE_PATH_LOOKAHEAD_M` = 1.5). Missione 08-10 16:11, scansione `it001_cell0_2_007`: il percorso girava a ovest attorno a un blocco, l'obiettivo era a est oltre un vicolo cieco, e "avvicinarsi" all'obiettivo ha portato il robot 1.36 m dentro il vicolo cieco. Nella scansione 008 il bordo dei dati si è accorciato durante una rotazione (1.29 m su 1.38) senza nessun ostacolo: da qui `ABORT_IGNORE_DATA_EDGE_UNTIL_M`.
- **Punto d'arrivo senza uscite e nessuna alternativa: si rinuncia al lato.** Prima si andava comunque. Missione 08-10 16:11, a (60.05, −31.06): 0/24 uscite viste in anticipo, il robot ci è entrato, si è trovato in trappola ed è tornato indietro con GraphNav per 3 waypoint.
- **Zona attorno al corpo nella mappa globale** (nucleo libero, anello senza ostacoli). Missione 08-10 15:09, attorno a (58.6, −32.8): le stesse celle lette da 1–2 m avevano `obstacle_distance` 0.18–0.24 (libere), lette con il robot fermo lì accanto 0.09–0.15 (ostacolo), a 0.23–0.45 m dall'asse, all'altezza delle zampe posteriori. Due scansioni da fermo le confermavano, e il PRM scartava l'arco su cui il robot era appena passato.
- **Scorciatoie nel grafo GraphNav** (`graphnav_add_shortcuts`). Il grafo era una catena e gli anelli si chiudevano solo a fine missione e solo con i fiducial ("Created 0 new edge(s)" nel log del 08-10 15:40). Missione 08-10 16:11: Spot accetta archi nuovi solo mentre registra ("NotRecordingError"), la chiusura d'anello va fatta a registrazione ferma e può rispondere "MapModifiedError" subito dopo.
- **Cella bloccata subito se non resta un punto dove stare** (`cell_has_reachable_target`). Missione 08-10 16:35: la griglia 2×4 da 5 m non stava nello spazio (corridoio esplorato 12×5 m); classificando i centri contro la mappa finale, (0,1) aveva 144 celle occupate su 169 e (0,2) 123 su 130, due obiettivi dentro il muro, e tutta la riga 1 non era mai stata vista: tre blocchi su quattro tentativi venivano da lì. Con ~400 punti candidati per cella il blocco immediato non sarebbe scattato per nessuna delle otto celle.
- Nodi di partenza e arrivo agganciati a nodi esistenti: lo scarto delle rotazioni rifiutate vale finché il robot è dove la rotazione è stata rifiutata, e sul primo tratto, se il robot non è sul nodo, un blocco confermato scarta l'arco solo per ora.

---

## 2026-10-07

### Fronte sicuro (`arcVerification.py`)
- **Filtro temporale su `obstacle_distance`: mediana su 3 scansioni e isteresi di 4 cm.** Missione 07-10 14:0x, robot fermo e percorso identico: una cella ha letto 0.296 e poi 0.336 su una soglia di 0.300, e il fronte è passato da "ostacolo" a "libero" senza che niente si muovesse. Rumore misurato fra due scansioni consecutive: 1.5 cm in media, 7.3 cm di punta.
- Lo scorrimento fra scansioni si misura fra griglie consecutive e non da un'origine assoluta: `cell_size` arriva come float32 (0.029999999329447746) e la cronologia si azzerava a ogni scansione.
- **"Cella vista" confrontata con `TERRAIN_VALID_MIN` e non con 0.** Dopo scala e offset lo 0 di `terrain_valid` diventa ±5.5e-17 con segno variabile: il vecchio `> 0.0` dichiarava vista ogni cella in 10 scansioni su 16.

### Mappe e terreno (`spotGrid.py`)
- **`TERRAIN_VALID_MIN` = 0.5 in `correct_terrain`** (stesso errore del `> 0.0`). Sulle 16 scansioni del 07-10, in 10 ogni cella risultava valida. Le celle non attendibili, comprese quelle mai scritte, entravano nel terreno come dati buoni: in media 276 veti di sola rugosità per scansione, fino a 1138, e in una scansione il robot risultava in piedi su una cella ostacolo.
- **`_cells_never_written` riscritta.** La versione precedente non era mai scattata: zero celle in 16 scansioni, mentre le celle mai scritte erano 1845–3710 (11–23% della griglia). Due errori indipendenti: la quota attesa era nel frame della griglia locale e le quote in VISION (0.26–0.35 m di differenza contro 0.25 mm di tolleranza), e quelle celle non stanno al grezzo 0 ma a un intero che cambia a ogni scansione. Ora la quota si ricava dai dati: la più frequente fra le celle che Spot dichiara non attendibili (98.2–100% di quelle mai scritte).
- **Le celle che `obstacle_distance` chiama già ostacolo non entrano nella pendenza.** Missione 07-10 13:58, stanza con mobili: 200 archi vetati per pendenza, mediana longitudinale 1.487 contro soglia 0.600, massimo 2.092, e la ripianificazione falliva da ogni nodo (225, 227, 228) nonostante 22–28 collegamenti ciascuno. La faccia di un mobile è un ostacolo, già gestito dal veto d'occupazione e dal fronte. Quegli archi diventano "senza dati di pendenza", entrano nel grafo con il costo di ripiego e li verifica il fronte.
- **Rotazione: si controlla il settore che il corpo spazza davvero, non tutto il cerchio di 0.60 m.** Missione 07-10, iterazione 2, cella (1,0): 9 rotazioni rifiutate su 10, di cui 8 sicure; in tre casi (scansioni 8–10) il rifiuto ha prodotto un blocco su un percorso libero per 1.39 m.
- **Aria sulla rotazione a 0** (scelta esplicita). Un falso "non posso ruotare" era costato 3.5 m di arretramento in catena, e 0.05 m era meno del rumore di `obstacle_distance`. Sugli 11 rifiuti della missione 07-10 14:0x sblocca un solo caso: gli altri sono veri (corpo dentro l'ostacolo di 4–19 cm in corridoi da ~60 cm). Non è la correzione del problema.
- **Tolleranza sul rumore nella rotazione** (`ROTATION_NOISE_TOLERANCE_M`). Missione 07-10 16:50: dalla stessa posa, in cinque scansioni consecutive, `obstacle_distance` in un punto a 50 cm oscillava fra −0.03 e +0.15 m (+0.45 visto da 2 m); 5 rifiuti su 17 stavano dentro quel rumore. Nei passaggi fra i tavoli il margine vero è −0.16/−0.19.

### Navigazione (`easy_walk.py`)
- **Niente marcia di traverso** (decisione dell'utente): il robot punta sempre il muso verso il punto da raggiungere. Di traverso Spot è largo 1.10 m invece di 0.50 e il fronte pretende 0.59 m invece di 0.30: sei blocchi su percorsi che col muso in avanti erano liberi per 1.3–1.4 m sono nati solo così. Quando non può ruotare il robot arretra dritto lungo l'asse del corpo.
- **Arretramenti contati per sequenza** (`MAX_STRAIGHT_RETREATS_IN_A_ROW`). Il limite per arco ripartiva da 1 perché ogni arretramento creava un nodo di sosta nuovo: nel log "tentativo 1/2" sette volte di fila. Sono nati così 3.5 m all'indietro.
- **Cono di rotazione possibile** (`plan_cone_move`). Missione 07-10 14:0x, file `iteration_0_cell_0_1_replan_12_vis.npz`, robot in (−2.28, 1.84), obiettivo a 1.26 m: andare dritti, senza ruotare, portava a 0.50 m dall'obiettivo (−0.76); la rotazione voluta (+22 gradi) era impossibile; il codice ha arretrato sette volte di fila, 3.5 m, e da ogni punto d'arrivo la rotazione restava impossibile (margine −0.09 m). Corridoio con ~60 cm di corsia libera: girarsi richiede 1.21 m di diametro.
- **Arretrare solo se serve** (`retreat_would_help`): nei due momenti esaminati la rotazione dopo l'arretramento restava impossibile (margine −0.09 m a (−2.28, 1.84), −0.00 m a (−3.91, 1.74)). L'arretramento dritto era invece disponibile in tutte e 12 le scansioni dell'iterazione 1, da 1.08 a 1.37 m.
- **Ventaglio a 360 gradi per riconoscere la trappola** (`scan_escape_directions`, `is_trapped`). Missione 07-10 14:5x, file `iteration_2_cell_1_0_BLOCKED_vis.npz`, robot in (−1.69, 1.87), `obstacle_distance` sotto di lui 0.402: le due corsie aperte (1.30 m a −180 gradi, 1.09 m a 0) chiedevano rotazioni di −73 e +107 gradi, impossibili; le altre 21 direzioni avevano 0–0.24 m. Il codice ci era arrivato scartando otto archi uno per uno in una ventina di scansioni; lo stesso schema al nodo 281 e, con rotazioni di ±179 gradi, in cella (0,2), dove quattro archi sono stati condannati per sempre.
- **Controllo della posa d'arrivo** (`arrival_has_exit`). Stessa missione: il robot è entrato in un angolo senza uscite con un ultimo passo di 23 cm, ricostruito dalla ritirata ((0.40, 1.76) → (−0.71, 1.70) → (−1.47, 1.79) → (−1.69, 1.87)).
- **Ritorni con GraphNav invece che a zampe all'indietro** (decisione dell'utente): briciole (`graphnav_drop_breadcrumb`) ogni 0.75 m e all'ingresso di ogni cella, ritorno di un waypoint alla volta riprovando a pianificare, ritirata a zampe solo come ultima rete. Una briciola per ogni nodo di sosta sarebbe stata troppo: nella missione 14:5x erano 37, molti a 20–60 cm uno dall'altro.
- **Niente ritorni inutili** (`backtracking_cannot_help`). Missione 07-10 16:50: 6 ritorni su 9 (celle (0,2) e (0,3)) per scoprire tre volte di fila che da lì il grafo non offriva percorsi; l'obiettivo era in un'altra componente già da diversi giri.
- **Nodi PRM dentro gli ostacoli scartati** (`prune_prm_nodes_in_obstacles`). Missione 07-10 14:5x: il nodo obiettivo della cella (0,1) aveva `obstacle_distance` −0.21 m e i due ripieghi 0.12 m, i tre "obiettivi riscelti" della missione.
- **Scarto soft per [SUBITO] e per le rotazioni rifiutate** (`mark_edge_blocked_soft`). Missione 07-10 14:5x: otto archi attorno al nodo 3164 sono finiti fuori dal grafo per tutto il resto della missione per un ostacolo marginale a 9 cm dal robot, e il nodo è rimasto isolato.
- **Obiettivo riscelto se irraggiungibile sui dati freschi** (`goal_still_reachable`, parametro `exclude`). Missione 07-10, iterazione 1, cella (0,1): l'obiettivo aveva `obstacle_distance` 0.27 contro 0.30 richiesti, era stato scelto da 3 m con la zona ancora ignota, e il robot si è bloccato a 63 cm da esso dopo tre scansioni ferme, con 808 celle valide entro 80 cm.
- **Tolleranza d'arrivo legata al fronte** (`GOAL_REACHED_TOLERANCE_M` = 0.65 invece di 0.60): con 0.60 restava una banda morta di 5 cm. Stessa iterazione: robot fermo con `path_len` 0.63–0.64 m, tre scansioni e blocco.
- **Ripianificazione alla prima cella bloccante** ([SUBITO]). Prima si riguardava da fermo fino alla conferma, ma fra la scansione 003 e la 004, robot fermo e percorso identico, la stessa cella ha letto 0.296 e poi 0.336 su soglia 0.300: aspettare aggiungeva rumore. Ora il rumore è filtrato in `arcVerification`.
- **Scorciatoie che rispettano gli archi scartati** (`shortcut_index`): ~40 giri identici sul nodo 225; nella missione 16:39 la scorciatoia ripuntava il nodo 218 subito dopo lo scarto di 3169→218 per rotazione.
- **Archi rifiutati per rotazione fuori dal piano finché il robot non si sposta.** Missione 07-10 16:39, cella (1,1): robot fermo su 3169 alternando 3169→142 e 3169→218 per 112 giri ("tentativo 56/2"), senza mai arrivare alla ritirata.
- **Pacchetto di scansione salvato prima del blocco rotazione.** Missione 07-10 16:50: nessuno dei 17 rifiuti di rotazione aveva lasciato una scansione su disco.
- **Celle ostacolo escluse dalla pendenza** (`slope_usable_mask`, vedi anche Mappe e terreno). Rimisurato sulle 12 scansioni della missione 14:0x, 3607 archi nella finestra:

  | | prima | dopo |
  |---|---|---|
  | archi vetati per pendenza | 40.3% | 0.1% |
  | archi senza dati di pendenza | 4.9% | 42.5% |
  | archi ammessi nel grafo | 59.7% | 99.9% |
  | pendenza longitudinale massima | 2.2–3.3 | 0.03–0.10 |

- **GraphNav: un errore RPC non abbandona più lo spostamento.** Il `return False` stava fuori dall'`if`: un solo errore transitorio abbandonava lo spostamento mentre il messaggio diceva "(1/3)".
- **Diagnostica dell'impronta (questione A).** Sui dati del 07-10 si misuravano 190–191 celle ostacolo nel nucleo del corpo, tutte per rugosità: celle mai scritte alla quota del plateau, con un salto di ~0.93 m dal pavimento. Con `_cells_never_written` corretta scendono a zero, quindi la maschera dell'impronta resta una tantum.
- Traiettoria reale campionata durante ogni movimento (`trajectory.csv`). Tolta la riga "[INIT] Initializing Velodyne client...": il Velodyne non è in uso e la riga faceva credere a un sensore in più.

---

## 2026-10-06

### Fronte sicuro (`arcVerification.py`)
- **Il fronte sicuro sostituisce la verifica arco per arco con verdetti permanenti.** Prima missione in esterno, 05-10 16:05: cinque ritirate da terreno libero. Gli archi venivano giudicati mentre il robot camminava, da lontano, troncati sul bordo della finestra, e il verdetto non veniva più riesaminato.
- Celle mai scritte dai sensori trattate come non viste: il fronte si ferma lì con `fine_dati`.
- (sera, dopo la revisione) **Regola di uscita da uno spazio stretto corretta.** La soglia abbassata nel primo metro valeva per qualunque ostacolo: un palo a 15 cm dal fianco, 70 cm più avanti, veniva accettato perché il robot partiva vicino a un altro oggetto. Ora vale "non avvicinarsi più di quanto già si sia".
- (sera) Tolleranza della regola di uscita portata a 1.5 celle: con 2 cm, lungo un muro a 5 gradi dal percorso, 257 blocchi su 360 posizioni (prima 13).

### Pianificatore (`prm_graph.py`)
- **Occupazione campionata ogni 10 cm, con un'unica costante** (`OCCUPANCY_SAMPLES_PER_M`). Prima la costruzione usava 3 campioni al metro e il refresh 5: lo stesso arco poteva essere ammesso e poi rifiutato solo per la densità. A 5 al metro, a metà fra due campioni il disco del margine (0.15 m) copriva solo 0.11 m, e uno spigolo poteva arrivare a 0.26 m dalla linea centrale, meno dei 0.30 del fronte: il robot si bloccava su piani "validi". (sera) Il numero di campioni si arrotonda per eccesso: `int()` dava fino a 0.147 m fra due campioni.
- **Il refresh rivaluta anche gli archi con un solo estremo nella finestra.** Un arco da un nodo vicino attraverso un muro appena visto verso un nodo lontano non veniva riesaminato, e con il fronte sicuro produceva blocchi a ripetizione sullo stesso muro (`test_e2e_loop.py`, scenario S2).
- **`block_edge` la chiama solo il ciclo principale**, dopo un blocco confermato da vicino. Prima la chiamava il vecchio verificatore su verdetti presi da lontano, causa delle ritirate del 05-10.
- `_evaluate_and_add_edge` estratta da `build_graph`, per collegare con la stessa logica anche i nodi di sosta.
- (sera) **Cache della pendenza globale per arco.** Era il costo dominante di `build_graph`: ~80 mila archi per ~500 letture di dizionario.
- (sera) **Primo tratto diretto nel costo del piano** (`allow_direct_first_hop`). Dopo una scorciatoia il primo tratto non è un arco del grafo e il piano risultava "rotto": si ripianificava a ogni scansione, il conteggio di conferma di un blocco ripartiva da zero e un blocco vero non veniva mai dichiarato.

### Mappe e terreno (`spotGrid.py`)
- **Filtro sull'altezza del suolo** (`GROUND_HEIGHT_TOLERANCE_M` = 1.2 m, riferito al suolo sotto il robot). Missione 05-10 16:05: il robot era stato acceso un piano più su, le celle mai scritte stavano 3.226 m sopra il suolo, e la loro rugosità (1.585 contro soglia 0.15) ha prodotto il 100% degli ostacoli dei blocchi. Asimmetrico apposta: sotto il suolo può esserci un vuoto vero, e non si tocca.
- **La pendenza si misura solo sulle celle attendibili** (`valid_2d`): un arco visto per meno di metà torna "senza dati" e si passa alla mappa globale.
- **Mappa globale degli ostacoli con conferma** (`GLOBAL_OCC_CONFIRM_OBS`). Prima bastava una sola osservazione di ostacolo, anche su una cella vista libera quattro volte. (sera) Le celle non attendibili in una scansione non contano più come "viste libere"; il disco attorno a una caduta non torna più libero (prima spariva alla quarta scansione).
- Disco occupato attorno al punto di una caduta, senza aspettare conferme dei sensori.
- **Rotazione sul posto controllata**: gli spigoli del corpo descrivono un cerchio di 0.60 m e prima nessuno controllava che fosse libero.
- Margine del PRM dagli ostacoli da 0.05 a 0.15 m, per dare la stessa distanza del fronte. Prefiltro a grana grossa in `is_occupied`: con 81 celle per controllo, senza prefiltro la costruzione passerebbe da ~2 s a ~12 s.
- Velocità massime per livello di andatura: prima nessun limite (Spot fino a ~1.6 m/s).
- (sera) **Uscita rapida nel profilo di pendenza** per gli archi che non toccano la griglia: prima ognuno pagava il campionamento intero, ~30 s per costruzione.
- (sera) **Download dei layer più robusto**: niente risultato se manca un layer (prima `KeyError` e missione terminata per un errore di rete) o se i layer non sono allineati fra loro.
- (sera, revisione) La differenza della pendenza è centrata: la base reale è ~0.48 m, non 0.25. Un gradino di 25 cm dà ~0.52 di pendenza e ~0.125 di rugosità, entrambe sotto soglia. Non cambiato.

### Movimenti (`movements.py`)
- **Limite di velocità su ogni comando.** Prima nessuno lo impostava e Spot camminava fino a ~1.6 m/s: nella missione del 05-10 la media sui tratti era ~0.3 m/s, ma con picchi a metà tratto troppo alti per l'esterno. Il limite è sempre simmetrico (`min_vel` = −`max_vel`): con il solo massimo il robot non può più andare indietro né a sinistra, e la ritirata smetterebbe di funzionare senza errori.
- **Arresto a metà movimento** (`should_abort`) e feedback ogni 0.2 s invece di 1.0 (a 0.5 m/s, mezzo metro fra due controlli). Prima il comando era bloccante fino a fine traiettoria: nella missione del 05-10 13:12 il verificatore aveva visto un ostacolo a movimento in corso, ma nessuno poteva fermare il robot.
- Rilevazione delle cadute e raddrizzamento automatico (`check_fall`, `recover_from_fall`).
- (sera, dopo la revisione) **Tempo massimo per movimento.** Il comando durava 6000 s: se Spot non riusciva ad arrivare, restava a provare fino a 100 minuti. (sera) **Errori di rete**: prima uscivano come eccezione e chiudevano la missione senza fermare il robot, che finiva la traiettoria da solo; ora si riprova 3 volte, poi si ferma il robot.

### Navigazione (`easy_walk.py`)
- **Spostamenti con GraphNav con tempo massimo** (`_graphnav_navigate_to`, sera, dopo la revisione). `navigate_to_waypoint` e `navigate_to_first_waypoint` di `navGraphUtils.py` non avevano tempo massimo (ripetevano all'infinito anche con il robot caduto, e la missione non arrivava mai a sedersi), usavano `TravelParams` vuoti (velocità di default di Spot) e su `STATUS_LOST` forzavano la localizzazione sulla destinazione e dichiaravano "arrivato".
- (sera, revisione) **Ritirata solo in continuità** (`RETREAT_CONTINUITY_M`): dopo uno spostamento con GraphNav l'ultimo tratto registrato poteva stare a molti metri, e il robot ci andava camminando all'indietro, alla cieca.
- **Scorciatoie del percorso** (`shortcut_index`), controllate dal fronte e dalla mappa globale (dove sta, per esempio, il punto di una caduta).
- **Andature tarate per l'esterno**: PLAIN aveva `SWING_HEIGHT_LOW` (passo da pavimento) e nessun livello aveva un limite di velocità.
- **Quota di riferimento della missione** (`mission_z0`): Spot misura le quote da dove è stato acceso, e portandolo alla zona di missione per le scale tutte le quote risultavano spostate (−3.2 m il 05-10).
- Margine del punto obiettivo uguale a quello del PRM e del fronte (era 0.05 m).

---

## Prima del 2026-10-06

### Pianificatore (`prm_graph.py`)
- **Costo con pendenza longitudinale e laterale separate.** Prima la pendenza era un valore unico, e un arco di traverso su un pendio costava quanto uno piano con lo stesso gradiente.
- Il profilo di pendenza dell'arco si calcola una volta sola e passa dal veto al costo: prima due volte per arco, con gli stessi argomenti.
- Gli archi percorsi dal robot non tornano più a forza nel grafo se dopo sono stati bloccati o passano sopra il punto di una caduta (`_traversed_edge_ok`): prima un arco percorso all'andata e poi bloccato tornava alla ricostruzione successiva.
- `find_path_dijkstra` restituisce il piano in corso quando la stabilità lo tiene; il chiamante deve distinguerlo da `None`. Nelle missioni del 05-10 `[REPLAN WARNING]` scambiava un caso per l'altro.

### Mappe e terreno (`spotGrid.py`)
- **Pendenza d'arco su una base fisica** (`SLOPE_BASELINE_M`) e non sul passo della cella (~3 cm): la faccia di un armadio da 80 cm dava 13 m/m e un'anta da 1.45 m 24, esattamente i massimi nei log del 05-10.
- **Fetta laterale larga quanto il robot** (±0.30 m), non quanto l'arco: prima un arco da 2 m raccoglieva terreno fino a 1 m per lato, e lo stesso punto era percorribile con un arco da 1.5 m e vietato con uno da 2 m.
- **`has_data` onesto.** Prima era sempre vero: un arco fuori dalla finestra risultava una misura dal vivo di pendenza 0, e la mappa globale non veniva mai usata (`global_map: 0.0%` in ogni riepilogo). La mappa globale ora usa la stessa geometria e la stessa base della griglia locale.
- **Uscita anticipata sulla mappa globale.** Missione 05-10 13:12: il 98% delle letture non trovava nulla dopo ~500 accessi per arco, circa 50 s per costruzione del grafo.
- **Chiavi della mappa globale con `np.floor`, non `np.round`** (occupazione e altezze): con i centri delle celle l'arrotondamento al pari faceva collassare celle diverse sulla stessa chiave e lasciava chiavi mai scritte, lette come "libere".
- **Rugosità calcolata solo su celle valide.** Missione 05-10, iterazione 6: 29 celle mai scritte hanno prodotto 83 falsi ostacoli attorno a sé; con le statistiche solo sulle valide quei veti vanno a 0 (rugosità massima da 1.585 a 0.029) e i 54 veti veri restano.
- `LATERAL_SLOPE_COST_MULTIPLIER` da 1.5 a 1.0: 1.5 era una supposizione, e la stima della pendenza di allora sovrastimava le componenti piccole fino a ~1.9 volte.

### Controllo pre-missione (`preflight.py`)
- **Nato dalla missione del 05-10 15:55**, interrotta appena costruito il grafo da `AttributeError: 'GlobalGrid' object has no attribute 'is_known'`: `easy_walk.py` era aggiornato, `spotGrid.py` sul robot era una copia vecchia. Nessun rischio (prima di qualunque movimento), ma robot già acceso e in piedi.

### Navigazione (`easy_walk.py`)
- **Punto obiettivo scelto sui dati.** Prima `find_best_point_in_cell` restituiva sempre il centro geometrico, ignorando la scansione: in una stanza arredata il centro cadeva dentro o a ridosso di un mobile. Missione 05-10 13:12: percorso 176 → 211 → 3127, oltre il bersaglio di più di un metro e poi indietro, fino a una tasca stretta. (Dal 13-10 il centro torna a essere l'obiettivo, ma si cambia quando è visto occupato.)
- **Ritirata all'indietro sui propri passi** (`retreat_along_traveled_path`). Missione 05-10 13:12: prima `[REPLAN FALLITO]`, poi l'arco successivo bloccato, in uno spazio in cui girarsi era impossibile.
- **`navigate_to` restituisce il booleano**, non la tupla di `relative_move`: una tupla non vuota è sempre vera, e un movimento fallito risultava riuscito. Era impossibile accorgersi che il robot era bloccato.
- **Il grafo non si costruisce più all'avvio**: senza dati di terreno né di occupazione era privo di informazione, e il primo tentativo di cella lo ricostruiva da zero prima di qualunque uso.
- **Una cella fallita da un lato non si ritenta subito né si blocca** (`finalize_or_defer_blocked_cell`): resta candidata e si ritenta quando tocca a lei, dal lato non ancora provato.
- Il lato provato è quello della cella da cui si tenta l'ingresso, non quella in cui il robot si trova dopo il tentativo. Senza waypoint da cui avvicinarsi la cella si salta (prima un indice su `None` terminava la missione).
