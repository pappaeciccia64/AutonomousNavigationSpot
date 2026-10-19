# Storia delle modifiche

Le modifiche più recenti in alto. Per ognuna: cosa è cambiato, perché, e il dato di campo che l'ha motivata. Il codice descrive solo come funziona oggi; da qui si capisce come ci si è arrivati. Il dettaglio riga per riga è nella storia di git.

Le missioni sono indicate come `giorno-mese ora` (per esempio `13-10 12:40`); i loro dati stanno in `MissionLogs/` e `MissionMap/`.

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

---

## Prima del 2026-10-06

### Pianificatore (`prm_graph.py`)
- **Costo con pendenza longitudinale e laterale separate.** Prima la pendenza era un valore unico, e un arco di traverso su un pendio costava quanto uno piano con lo stesso gradiente.
- Il profilo di pendenza dell'arco si calcola una volta sola e passa dal veto al costo: prima due volte per arco, con gli stessi argomenti.
- `find_path_dijkstra` restituisce il piano in corso quando la stabilità lo tiene; il chiamante deve distinguerlo da `None`. Nelle missioni del 05-10 `[REPLAN WARNING]` scambiava un caso per l'altro.

### Mappe e terreno (`spotGrid.py`)
- **Pendenza d'arco su una base fisica** (`SLOPE_BASELINE_M`) e non sul passo della cella (~3 cm): la faccia di un armadio da 80 cm dava 13 m/m e un'anta da 1.45 m 24, esattamente i massimi nei log del 05-10.
- **Fetta laterale larga quanto il robot** (±0.30 m), non quanto l'arco: prima un arco da 2 m raccoglieva terreno fino a 1 m per lato, e lo stesso punto era percorribile con un arco da 1.5 m e vietato con uno da 2 m.
- **`has_data` onesto.** Prima era sempre vero: un arco fuori dalla finestra risultava una misura dal vivo di pendenza 0, e la mappa globale non veniva mai usata (`global_map: 0.0%` in ogni riepilogo). La mappa globale ora usa la stessa geometria e la stessa base della griglia locale.
- **Uscita anticipata sulla mappa globale.** Missione 05-10 13:12: il 98% delle letture non trovava nulla dopo ~500 accessi per arco, circa 50 s per costruzione del grafo.
- **Chiavi della mappa globale con `np.floor`, non `np.round`** (occupazione e altezze): con i centri delle celle l'arrotondamento al pari faceva collassare celle diverse sulla stessa chiave e lasciava chiavi mai scritte, lette come "libere".
- **Rugosità calcolata solo su celle valide.** Missione 05-10, iterazione 6: 29 celle mai scritte hanno prodotto 83 falsi ostacoli attorno a sé; con le statistiche solo sulle valide quei veti vanno a 0 (rugosità massima da 1.585 a 0.029) e i 54 veti veri restano.
- `LATERAL_SLOPE_COST_MULTIPLIER` da 1.5 a 1.0: 1.5 era una supposizione, e la stima della pendenza di allora sovrastimava le componenti piccole fino a ~1.9 volte.
