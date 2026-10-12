# 31. Archi 0.5-1.25, avanzamento multinodo, refresh a eventi, guardie anti-stallo (2026-10-08 sera)

Da inserire nel CONTEXT.md come punti 31 e 32. Non l'ho fatto io perche' la copia di
CONTEXT.md che vedo e' precedente al branch AdL-version e riscrivendola perderei le voci 30+.

Come applicare: `git apply revisione_2026-10-08.patch` sul branch AdL-version (8b8a4ad).
Verificato su una copia pulita del branch: si applica senza conflitti e i file risultanti
sono identici a quelli su cui sono girate le prove. Per leggere il diff di easy_walk.py
senza il rumore del rientro: `git diff -w`.

---

# 32. Revisione completa (2026-10-08, sera tardi)

Rilettura del codice scritto oggi e del ciclo, con il banco di prova reso piu' fedele.
Quattro difetti veri trovati e corretti, uno dei quali introdotto da me poche ore prima.

## a) GRAVE, introdotto oggi: le gambe del multinodo ruotavano dell'angolo sbagliato

`navigate_to` calcola l'angolo di rotazione dall'orientamento che riceve, e
`relative_move` ruota RISPETTO ALLA POSA ATTUALE (legge la trasformata dal robot). Nel
ciclo delle gambe passavo a tutte l'orientamento della scansione iniziale, cioe' quello
PRIMA della gamba 0. Dalla seconda gamba il robot ruotava di (direzione voluta -
orientamento vecchio) partendo dall'orientamento nuovo, e camminava fuori dal percorso
verificato: 30 gradi di errore su 1 m sono 50 cm di lato.

Il banco di prova non poteva vederlo: il suo `navigate_to` finto ignorava quell'argomento e
metteva il robot dritto verso il bersaglio. Reso fedele (rotazione relativa calcolata come
nel vero, camminata lungo il muso REALE), lo scenario nuovo S12 misurava **18 cm di scarto**
fra punto comandato e punto raggiunto su un solo movimento a 2 nodi. Corretto passando a
ogni gamba la posa letta dal robot (`spotUtils.getPosition`): scarto **0.000 m**.

## b) Le rotazioni ai nodi intermedi non erano controllate

La decisione iniziale controlla solo la rotazione della prima gamba; ai nodi intermedi il
robot si ferma e ruota senza alcun controllo. Ora prima di ogni gamba successiva:
`rotation_plan` sulla fotografia della scansione (in coordinate mondo, quindi valida dopo
lo spostamento) e `arrival_has_exit` con le stesse condizioni della prima gamba. Se la
rotazione non si puo' fare, o il cerchio spazzato esce dai dati della scansione
(`rotation_is_clear` ignora le celle fuori finestra), ci si ferma e si riguarda: la
rotazione la gestisce il ciclo, con cono, arretramento e ventaglio. Contatore:
`multinode_cut` in [MISSION STATS].

## c) Guardie anti-stallo mal tarate (scritte oggi)

- `MAX_IDLE_LOOPS` era 12 "sopra la sequenza legittima di ~8": era sotto. Calcolata sui
  limiti del ciclo ([SUBITO] 4 + obiettivi riscelti 3 + conferme di blocco 3x4 + rifiuti
  di rotazione 4 + rifiuti d'arrivo 3 = 26), ora **30**. Con 12 avrebbe tagliato via proprio
  i ritorni con GraphNav che quella sequenza deve raggiungere.
- `MAX_ROTATION_DISCARDS`: una prima correzione azzerava il contatore a ogni spostamento.
  Il banco di prova (S11) ha mostrato il guaio: non potendo girarsi verso ovest, il
  pianificatore portava il robot un nodo a EST e riprovava -- **35 scansioni e 4.5 m
  percorsi allontanandosi dall'obiettivo**, contatore sempre a zero. Ora si azzera solo
  quando il robot si AVVICINA davvero all'obiettivo (20 cm meglio del meglio raggiunto), e
  dopo i ritorni con GraphNav. Muoversi non e' fare progressi.

## d) Il controllo di rotazione non vedeva gli ostacoli sottili

`spotGrid.rotation_is_clear` confrontava con -0.15 il MINIMO di obstacle_distance sulle
celle spazzate. Dentro un ostacolo obstacle_distance non scende sotto meta' del suo
spessore, quindi con la tolleranza a 0.15 qualunque ostacolo piu' sottile di ~30 cm che
entra nel corpo durante la rotazione veniva approvato. Misurato con distanze col segno
realistiche (palo in (0.15, 0.45), rotazione di 90 gradi che lo porta dentro il corpo):

| palo | prima | dopo |
|---|---|---|
| 10 cm | approvata (-0.042) | rifiutata (-0.167) |
| 20 cm | approvata (-0.085) | rifiutata (-0.200) |
| 30 cm | approvata (-0.134) | rifiutata (-0.249) |
| 40 cm | rifiutata (-0.182) | rifiutata (-0.297) |

Ora si misura la PENETRAZIONE (profondita' della cella dentro il corpo meno
obstacle_distance), escludendo le celle sotto il corpo ADESSO (il robot ci sta sopra:
libere per definizione -- e' proprio l'errore di registrazione che la tolleranza doveva
assorbire). Per i muri da' lo stesso numero di prima; non e' mai piu' permissiva.
**Verificata sui 47 scan veri del 16:35, 1081 rotazioni: stesso verdetto nel 97.8%; i 24
che cambiano vanno tutti verso la prudenza** (quasi tutti al limite, -0.150 -> -0.151;
l'eccezione e' lo scan 12 della cella (0,2), punto strettissimo, -0.085 -> -0.17).
Interruttore: `spotGrid.ROTATION_CHECK_PENETRATION = False` torna alla metrica vecchia.
La tolleranza resta 0.15, come deciso.

## e) Il banco di prova non provava quello che diceva

- `navigate_to` finto non fedele (vedi a).
- `obstacle_distance` dentro gli ostacoli fisso a -0.01: il controllo di rotazione vedeva
  "1 cm dentro il muro" anche quando il corpo ci entrava per 15. Ora distanza con segno.
- Il filtro temporale degli ostacoli (allineato al mondo, giusto sul robot) sopravviveva
  da uno scenario all'altro: stesse coordinate, mondo diverso. S11 passava o falliva a
  seconda dell'ordine. Ora azzerato a ogni scenario.
- S11 dichiarava un corridoio da 0.80 m ed era da 0.90, e il robot partiva gia' dentro la
  cella obiettivo: passava senza provare niente. Ora esercita davvero la guardia: rotazione
  rifiutata 5 volte, il robot non ruota mai, tentativo chiuso in 6 giri.
- I controlli `ok is True` fallivano con `np.True_` (il codice vero restituisce un bool di
  numpy; il chiamante fa `if check:` e funziona).

## Cosa resta APERTO, in ordine di importanza

1. **Tolleranza di rotazione a 0.15 m** (scelta documentata sul branch). Con la metrica
   nuova vale allo stesso modo per muri e pali: il corpo puo' entrare di 15 cm in qualunque
   cosa. Un palo da 6 cm che finisce 13 cm dentro il corpo e' ancora approvato; la prova
   unitaria "anello di ostacoli a 0.45 m" (il corpo ci entra di 14 cm) ora fallisce per
   questo. Se in prova si vedono sfioramenti durante le rotazioni: 0.10.
2. **La griglia di missione che non sta nello spazio** (tre blocchi su quattro al 16:35).
   Non e' codice: griglia ridotta e robot allineato al corridoio.
3. **In un corridoio piu' stretto di ~1.2 m il robot non puo' raggiungere un obiettivo che
   ha alle spalle**: puo' solo andare avanti o arretrare per far spazio, mai camminare
   all'indietro VERSO l'obiettivo. Le guardie ora chiudono il tentativo in pochi giri invece
   di farlo girare a vuoto, ma la cella non viene raggiunta. E' una scelta di progetto da
   rivedere, non un difetto.
4. `test_offline.py` del repository importa un modulo `PRM` che non esiste piu': non gira e
   non verifica il codice attuale.
5. Le guardie anti-stallo e il taglio del multinodo ai nodi intermedi sono verificati solo
   nel banco di prova.

## Da dove nasce: la missione del 08-10 16:35

Quattro tentativi, tre blocchi. Diagnosi dai dati (47 pacchetti di scansione, mappa
globale, file di blocco; nessun mission_log.txt disponibile):

**Causa principale, che NON e' nel codice: la griglia di missione non stava nello spazio.**
Il robot ha esplorato un corridoio di 12 x 5 m con una fascia libera di 1-1.5 m; la
missione chiedeva 2x4 celle da 5 m, cioe' 20 x 10 m. Ha visto 46.8 m2 su 200 (23%), e di
quelli il 53% occupato. Classificando i centri cella contro la mappa finale:

| cella | centro | giudizio |
|---|---|---|
| (0,0) | 55.46, -32.65 | libera (169 celle viste, 0 occupate) |
| (0,1) | 60.43, -32.10 | DENTRO un ostacolo (144 su 169) |
| (0,2) | 65.40, -31.55 | DENTRO un ostacolo (123 su 130) |
| (0,3) e tutta la riga 1 | — | mai viste, zero celle in mappa |

In piu' la griglia e' allineata all'imbardata di accensione (6.4 gradi) mentre il corridoio
corre lungo x: su 10 m la linea dei centri si sposta di 10*tan(6.4) = 1.1 m, piu' della
mezza larghezza del corridoio. Ecco perche' (0,0) e' nel libero, (0,1) al bordo del muro e
(0,2) dentro. **Accendere il robot allineato alla direzione di esplorazione, in un
corridoio, conta piu' di qualunque parametro.**

**Secondo difetto, questo nel codice: 8 giri di ciclo in 3.2 s con il robot immobile**
(iterazione 2, cella (1,1)): stessa posizione, stessa imbardata, stesso nodo 595, scansioni
identiche, target che rimbalzava fra ovest, nord ed est. Rigiocando quelle scansioni con
`rotation_plan`, `plan_cone_move` e `scan_escape_directions` del branch:

| scan | rotazione richiesta | ruota? | margine settore | cono? | direzioni utilizzabili |
|---|---|---|---|---|---|
| 001 | -147 gradi | no | -0.19 m | no | 3/24 |
| 005 | +153 gradi | no | -0.17 m | no | 3/24 |
| 007 | +88 gradi | no | -0.17 m | no | 3/24 |
| 008 | +29 gradi | si | +0.03 m | no | 3/24 |

Il percorso chiedeva una rotazione di quasi 180 gradi (l'obiettivo stava dietro il muro a
nord, quindi Dijkstra faceva tornare indietro), nel corridoio non si poteva fare, il cono
non trovava direzioni che avvicinassero di 20 cm, e le direzioni utilizzabili erano 3 su 24
-- **non zero, quindi non "trappola"**. Il codice scartava il tratto, ripianificava e
`continue`: senza muoversi. `no_progress` non lo intercettava perche' la decisione era
`move`, e `MAX_ROTATION_FAILS_SAME_EDGE` nemmeno, perche' e' indicizzato sulla coppia di
nodi, che cambia a ogni ripianificazione.

## Cosa e' stato cambiato

**a) Archi 0.5-1.25 m** (prima 0.5-0.707). Reticolo invariato a passo 0.25 m. Si ottiene
con `MISSION_LATTICE_RINGS = None` e `PRM_MAX_EDGE_M = 1.25`: il limite inferiore era gia'
`min_edge_length = 0.5`, che non va abbassato perche' `compute_arc_slope_profile` misura la
pendenza su una base di ~0.48 m e un arco piu' corto la misurerebbe su terreno che non
percorre.

Costo misurato sulla mappa globale VERA del 16:35, 3321 nodi:

| regola | coppie | archi ammessi | costruzione | refresh locale |
|---|---|---|---|---|
| 0.5-0.707 (prima) | 25 120 | 18 853 | 0.92 s | 0.12 s |
| 0.5-1.25 (ora) | 109 474 | 79 229 | 4.72 s | 0.43 s |

Sulla missione del 16:35 (4 costruzioni, 47 scansioni) sono ~39 s invece di ~9 s. Prezzo
noto e accettato. Provata anche la selezione "un arco per direzione" (16 direzioni x 2
lunghezze, 52 840 coppie, 2.42 s): **scartata**, perche' `refresh_local_edge_weights`
ricostruisce da se' le coppie candidate nella finestra usando `max_edge_length` del PRM e
quindi disferebbe il filtro proprio attorno al robot, dove conta.

**b) Avanzamento multinodo** (`decide_next_move`, nuovo campo `nodes`). Il fronte sicuro
percorre TUTTO il percorso pianificato e dice fin dove e' confermato libero; prima quel
dato veniva calcolato e buttato via, perche' si guardava solo il primo nodo. Ora si contano
tutti i nodi dentro l'avanzamento consentito e il ciclo li percorre uno dopo l'altro senza
scansione, refresh ne' ripianificazione in mezzo. Si cammina **sempre sulla polilinea**, un
tratto per volta: non si taglia la corda fra due nodi, che passerebbe su terreno non
verificato (quello lo fa `shortcut_index`, ma solo dopo avere verificato la corda col
fronte). Il controllo non diventa un verdetto ricordato: il thread ricalcola il fronte a
~5 Hz sul percorso rimanente e ferma il robot se si accorcia.

**c) Refresh a eventi.** `refresh_local_edge_weights` non si chiama piu' a ogni giro, ma
solo: primo giro del tentativo, fine di un movimento (completato, interrotto o fallito,
anche se ha attraversato piu' nodi), arco scartato, blocco confermato, obiettivo riscelto,
ritorno con GraphNav. Implementato con una `need_refresh` locale piu' una flag
`needs_refresh` sul PRM, messa dentro `mark_edge_blocked_soft`: copre in un punto solo i
sette siti in cui un arco viene scartato. Nella missione del 16:35, 22 scansioni su 46
(48%) avevano uno spostamento sotto i 25 cm: a 0.43 s l'una sono ~9.5 s risparmiati.
NON tocca la sicurezza: la rete non sono i veti del grafo ma il fronte sicuro, che resta a
ogni scansione (0.5 ms) e a 5 Hz nel thread.

**d) Guardie anti-stallo.** `MAX_ROTATION_DISCARDS = 4` scarti per rifiuto di rotazione per
TENTATIVO (non per coppia di nodi): superati, si tratta il posto come una trappola e si
torna indietro con GraphNav. Piu' `MAX_IDLE_LOOPS` giri consecutivi senza spostarsi di
`IDLE_MOVE_M = 0.05`: rete generica, qualunque sia il ramo che gira a vuoto. *Valori e
azzeramento corretti dalla revisione del punto 32 (30 giri, azzeramento solo sui progressi).*

**e) Diagnostica sulle celle** (`cell_has_reachable_target`): dopo un tentativo fallito si
stampa quanti punti candidati della cella sono liberi, mai visti o dentro un ostacolo. E'
la riga che al 16:35 avrebbe detto subito che la griglia non stava nello spazio.
*Nato come blocco immediato della cella, declassato a diagnostica dopo averlo misurato: con
il reticolo a 0.25 m i candidati per cella sono ~400 e ce n'e' sempre qualcuno libero o
ignoto, quindi non sarebbe scattato per nessuna delle otto celle del 16:35. Il blocco resta
solo per il caso estremo in cui non ne resta nemmeno uno.*

## Verifica fatta FUORI dal robot

`test_e2e_loop.py` e' stato rimesso in piedi sul branch (era rimasto alla versione del
2026-10-06 e non girava piu'): pilota il ciclo VERO
`attempt_enter_cell_from_position` contro un robot finto. Undici scenari, tutti verdi:
campo libero con celle fantasma, muro con varco, persona che compare durante il movimento,
cella chiusa, partenza in spazio stretto, caduta recuperabile con secondo tentativo, caduta
irrecuperabile, reticolo vero su campo libero, reticolo vero con percorso che curva,
corridoio da 0.8 m con obiettivo dietro, e un confronto diretto fra le due regole d'arco.

Adattamenti all'interfaccia di oggi: `FakeEnv` ha ora `is_cell_visited`,
`mark_cell_visited`, `get_cell_from_world`, `start_cell`, `origin_x/y`, perche'
`_restrict_planning_to_explored_cells` li usa; e **le celle obiettivo degli scenari sono
state spostate da (1,1) a (0,1)**, perche' con `PLAN_ONLY_EXPLORED_CELLS` una cella
diagonale rispetto alle visitate non e' pianificabile e tutti gli scenari fallivano prima
di muovere il robot.

Difetto trovato nel test stesso: il trigger dell'ostacolo dinamico voleva 0.9 m di tratto
rimanente e piazzava l'ostacolo a 0.85 m, soglie tarate su archi da 2 m. Con archi da
0.5-1.25 non scattava piu' e lo scenario passava **senza provare nulla**. Corretto a 0.45 m
di tratto rimanente, ostacolo sempre a 0.85 m dal centro del robot (a 0.45 m sarebbe sotto
il muso, mezzo corpo = 0.55: proverebbe l'impossibile). Con il trigger giusto l'arresto a
meta' movimento funziona: distanza minima del centro dall'ostacolo 0.28 m contro i 0.30
richiesti -- i 2 cm sono la conferma su due fronti consecutivi a 5 Hz.

## Il risultato piu' importante: i due cambiamenti si annullano in parte

Il tetto all'avanzamento **non e' il grafo, e' la finestra dei dati**: ~1.92 m di raggio
meno mezzo corpo e margine (0.65) fanno ~1.27 m confermati per volta. Misurato sui 32
movimenti veri del 16:35: avanzamento mediano 1.35 m, massimo 2.57 m.

Quindi con archi da 1.25 m **dentro un avanzamento ci sta un solo arco**, e il multinodo
non scatta quasi mai (zero volte in tutti gli scenari del banco di prova). Con gli archi
corti scattava nel 44% dei movimenti veri del 16:35 (mediana 1 nodo, massimo 4).

Confronto diretto, stesso mondo con percorso che curva:

| regola | comandi | metri | m/comando | rotazioni > 10 gradi | scansioni |
|---|---|---|---|---|---|
| archi 0.5-0.707 + multinodo | 7 | 7.3 | 1.04 | 4-7 (rumoroso fra una corsa e l'altra) | 5-7 |
| archi 0.5-1.25 | 7 | 7.2 | 1.03 | 3 | 7 |

Stessa distanza per comando; gli archi lunghi fanno **meno rotazioni**, che e' esattamente
cio' che era andato storto il 16:35. Il multinodo resta utile dove il pianificatore
scegliera' archi corti (attorno agli ostacoli, dove i nodi vengono scartati), ed e' gratis
quando non serve.

## Cosa resta da verificare SOL SUL ROBOT

1. Il costo vero di `[TEMPO] costruzione completa del grafo` con 79 000 archi e la mappa
   che cresce: la stima e' 4.7 s, misurata su una mappa di 52 000 celle. Con una mappa piu'
   grande puo' peggiorare.
2. *Superato dal punto 32:* il limite sui rifiuti di rotazione ora e' esercitato dal
   banco di prova (S11). Quello generico sui giri a vuoto no.
3. L'avanzamento multinodo non e' mai stato esercitato con archi da 1.25 m (non scatta) ne'
   su hardware. Il ramo si attiva solo con `MISSION_LATTICE_RINGS` basso o in un percorso
   fatto di archi corti.
4. `IDLE_MOVE_M = 0.05` e' un numero scelto, non misurato; `MAX_IDLE_LOOPS = 30` e'
   calcolato sui limiti del ciclo (punto 32).
