# Da fare

Idee, problemi aperti e decisioni rimandate, con i dati che servono per prenderli. Quando
una cosa è fatta o una decisione è presa si toglie da qui e va nel `CHANGELOG.md`, con il
perché: lì si leggono i progressi, qui solo ciò che manca.

---

## Per le missioni del 29-10 e la versione su `main` (entro fine settimana)

Dalle prove all'aperto del 28-10 (13:50, 14:17, 14:19, 14:23). Correzioni piccole, ognuna con
la sua prova senza robot, prima di portare il branch su `main`.

**Codice**: fatto il 28-10 sera (vedi CHANGELOG): `[SUBITO]` ripetuti sullo stesso tratto, ritorno a wp_0 con tre esiti, tratto non concluso ma arrivato (`[QUASI]`), codice superfluo. Accensione con `KeepaliveMotorsOffError`: non si fa, il traceback va bene e si riavvia a mano. Resta facoltativo spostare gli strumenti da riga di comando di `mission_graph.py` in `prova_grafo.py`.

**Prima di `main`**: una giornata di missioni con il codice di oggi (scelta delle celle corretta, arrivo GraphNav girato, `[FINE]`, le correzioni sopra) su terreno senza erba alta; se vanno come la 14:23, il branch va su `main`. Da aggiungere alle validazioni su GitHub la missione del 28-10 14:23.

**Sul campo** (finché l'erba non è sistemata)

- Partire su terreno basso o nudo, mai dentro l'erba alta: 13:50 (ritorno bloccato), 14:17 (nessuna cella raggiungibile, 21 s), 14:19 (3 celle). Ora il log lo dice: `[FINE] Nessuna cella raggiungibile dalla partenza`.
- Batteria piena: la 14:23 è partita al 66% ed è finita al 37%, a pochi minuti dal rientro automatico al 30% (consumo 1.15–1.19 %/min, 0.18–0.19 %/m).
- Marker a 1–2 m dalla partenza, fermo per tutta la missione.

**Missione di validazione del 29-10 10:57** (8 x 5 celle da 2 m, prato in salita di ~8 gradi, cespugli e una siepe lungo il lato sinistro): 40/40 celle al primo tentativo, nessun blocco, stallo o errore; il nuovo obiettivo dopo una ripianificazione fallita e' scattato una volta e ha funzionato. Ctrl+C di emergenza ancora da provare.

**Ritorno a wp_0 spostato di ~1.2 m (29-10 10:57)**, da sistemare prima o subito dopo `main`. GraphNav ha detto "arrivato", ma il tag di partenza visto all'arrivo (1.59 m, +1.03 m a sinistra, contro 1.08 m e -0.13 m alla partenza) dice che il robot era ~1.2 m alla destra della partenza; confermato a vista. Deriva dell'odometria 2.82 m in 15 min (salita, erba); GraphNav ne ha corretti 1.86 m, il residuo (~1.3 m) e' tutto lungo la siepe, l'unico riferimento lungo e dritto: l'ICP sa la distanza dalla siepe ma non la posizione lungo di lei. Il grafo (archi di odometria, comprese le 9 scorciatoie nostre) porta con se' tutta la deriva da wp_153 a wp_0. Il tag era nell'istantanea di wp_0 e visibile all'arrivo, ma GraphNav non l'ha usato per correggere, e noi lo usiamo solo per misurare. Proposta: dopo l'arrivo, se lo scarto dal tag supera 0.3 m, rilocalizzazione col fiducial (come all'avvio) e secondo navigate_to a wp_0, poi ricontrollo; dietro un interruttore, prova con le missioni finte e una missione sul campo. Anomalia da capire: subito dopo l'inizializzazione sul fiducial il rilevatore di perdita di GraphNav diceva PERSO fino al riaggancio delle 11:06 (nessun effetto sull'esplorazione).

---

## Decisioni con i dati del 28-10

| Questione | Dati del 28-10 | Decisione |
|---|---|---|
| **Chiusura d'anello finale** (solo completa a fine missione, limite 45 s) | 16 archi in 55.5 s (13:50), 20 in 65.6 s (14:23); caricati con `upload_graph` senza errori né secondo tentativo | Resta così. Spot sfora il limite di 10–20 s, crescendo con il grafo: da tenere d'occhio, eventualmente limite più basso. |
| **Rientri senza chiusura veloce** | Tutti i viaggi GraphNav riusciti (3 e 8), scorciatoie 116 -> 27 m e 132 -> 70 m | Resta spenta (`GRAPHNAV_SDK_LOOP_BUDGET_S = 0`). |
| **Deriva dell'odometria** | 14:23: **1.09 m in 26 min sul tag**, 0.92 m su GraphNav; 13:50: 0.19 m dopo 4 min | Supera di molto la soglia di 0.3 m: la correzione delle mappe globali va fatta (lavori principali, punto 3). |
| **Soglia di rientro a batteria bassa** (30%) | 1.15–1.19 %/min, 0.18–0.19 %/m; ritorno a wp_0 di 14–21 m costato 1 punto | Il ritorno costa poco: 30% ha margine ampio, si potrebbe scendere a 20–25%. Da decidere con missioni più grandi. |
| **`CLEARANCE_MAP_DECIDES`** (oggi `False`) | Disaccordo mediano 7–9% degli archi; nessun varco riproposto più volte | Resta spento. |
| **Andature** | HARD su tratti tranquilli (sotto 8°): 6 (13:50), 9 (14:23) | Poco tempo perso; da rivedere insieme al movimento fluido. |
| **Briciole GraphNav ogni 0.75 m** | Scaricamento della mappa 76–94 s a fine missione | Da provare 1.0–1.5 m quando si lavora sul movimento. |
| **Missione nelle validazioni** | 14:23: 56/56 celle, ritorno e fiducial riusciti | Da aggiungere alle missioni di validazione su GitHub. |

---

## Lavori principali, dopo la versione su `main`

1. **Erba e ostacoli che "si muovono"** di scansione in scansione. Il problema principale del 28-10: la vegetazione è vista come muro, nella nostra mappa e nell'evitamento ostacoli di GraphNav.
   - *Dati etichettati del 28-10* (il robot sapeva dov'era l'erba alta):
     - 13:50, partenza in una chiazza d'erba e celle (7,0)-(7,1) d'erba. Ostacoli attorno alla partenza: altezza mediana 0.11 m sul suolo (95% sotto 0.48), intensità 19 contro 54 del suolo libero. Nell'angolo 0.06 m e intensità 19 anche sul "libero" (tutto prato). Nel resto della missione 0.18 m (95% sotto 0.99), intensità 21 contro 70. Primo tentativo bloccato accanto alla partenza (122 celle su 800 sotto il corpo diventate ostacolo appena il robot si è girato).
     - 14:17, partenza in mezzo all'erba: ostacoli sul 54–67% delle celle viste attorno al robot, alti 4–15 cm, intensità uguale al suolo; nessuna cella raggiungibile.
     - 14:19, erba davanti (cella (0,3): ostacoli 34%, 0.12 m di mediana) e cespugli a sinistra ((1,0)-(1,1): 58–76%, 0.5–0.7 m di mediana, 90% oltre 1.1 m). Sulla mappa finale, con 0.25 m di margine (quello del fronte) nessun passaggio verso la (0,3); con 0.15 m il 74% della cella diventa raggiungibile. Non era la densità del grafo.
   - *Obiettivo*: la vegetazione bassa (pochi cm, intensità bassa) attraversabile o con margine ridotto; i cespugli sopra ~0.5 m restano ostacoli. Una regola dietro un interruttore, provata sul rigioco di queste missioni. Anche il 26-10: il 24% degli ostacoli bassi aveva l'intensità del suolo, contro il 5% di quelli alti.
2. **Movimento più fluido e ciclo non bloccante.**
   - *Dati del 28-10* (13:50, 14:23, da `trajectory.csv`): durante i tratti il robot avanza il 74–75% del tempo, ruota sul posto l'8%, ed è **fermo il 17–18% (~165 s a missione)**, in pezzetti brevi: ~0.6 s a tratto a metà avanzamento, in tutte le andature, e ~0.45 s a tratto alla fine, ad aspettare `BODY_STATUS_SETTLED` (i passi di assestamento: il "camminare sul posto"). I tratti sono di ~0.9 m: la velocità media per tratto è 0.17 m/s in PLAIN con limite 0.5, 0.11 m/s in MODERATE e HARD. Il robot non arriva mai in crociera: si gira, accelera, rallenta, si assesta, si ferma.
   - *Primo stadio*: percorsi continui invece di punti con fermata (traiettorie a più punti, girarsi camminando), niente assestamento sui tratti intermedi.
   - *Secondo stadio*: movimento, scansioni più frequenti, aggiornamento dei pesi e del grafo che girano mentre il robot cammina, invece del ciclo sequenziale di oggi (l'unica cosa in parallelo è il fronte sicuro). Cambia la struttura del ciclo: dopo il primo stadio, con il test end-to-end a confronto riga per riga. Entrambi dietro un interruttore, poi poche celle sul campo prima di lasciarli accesi.
   - Nella stessa occasione: il pianificatore ignora le rotazioni (inversioni a U impossibili nei corridoi stretti, 28-10 11:36 e scenario "imbuto" del test end-to-end).
3. **Correzione della deriva nelle mappe globali**, con `drift.csv` e il fiducial di partenza (1.09 m in 26 minuti il 28-10): riallineare la mappa globale con le localizzazioni di GraphNav, oppure dimenticare più in fretta le osservazioni vecchie quando il robot ci ripassa.
4. **Servono ancora le celle?** Vedi l'idea 5 sotto. Il 28-10 hanno pesato: l'arrivo "appena fuori" rende ambiguo dove sia il robot (celle (7,0)-(7,1) mai tentate alle 13:50, corretto); i lati fanno saltare tentativi; un angolo d'erba resta "da fare" solo perché la griglia lo divide in due celle.

---

## Idee da valutare (28-10)

Non ancora decise: prima serve dire cosa fanno esattamente e come si prova che funzionano.

1. ~~**Percentuale di esplorazione completata.**~~ Fatta il 28-10 (vedi CHANGELOG): riga `[PROGRESSO]` con celle chiuse sul totale, area vista, stima del tempo che manca e batteria; nel rapporto e in `mission_summary.json`.
2. ~~**Rientro con GraphNav: evitare la rotazione inutile.**~~ Fatta il 28-10 (vedi CHANGELOG): all'arrivo il robot è girato verso la cella da tentare (`GRAPHNAV_ARRIVAL_HEADING`). Da confermare sul campo con la riga "Rotazione dopo gli arrivi GraphNav" del rapporto. Il ritorno finale a wp_0 tiene ancora l'orientamento del waypoint (14:23: arrivato girato di 90°, tag visto dalla telecamera di destra): per wp_0 è giusto, è la posa di partenza.
3. **Obiettivo scelto da noi sul grafo globale, a esplorazione finita.** Oggi l'obiettivo è sempre il punto di una cella da esplorare. Serve un modo per dire al robot «vai lì» su una mappa già costruita, usando il grafo e le mappe globali invece di rifare l'esplorazione. È un modo d'uso nuovo, non una modifica al ciclo attuale: probabilmente uno script a parte.
4. **Infittire il grafo nelle zone strette, in modo locale e dinamico.** Oggi la densità dei nodi è decisa una volta per tutta l'area (`MISSION_RANDOM_DENSITY`). Nei varchi stretti servono più nodi, altrove sono tempo sprecato. Da definire come si riconosce «zona stretta» (lo spazio libero misurato è già disponibile) e se i nodi nuovi si aggiungono al grafo esistente o se si ricostruisce la zona. Si prova sul rigioco prima che sul robot. (Il 28-10 14:19 il collo di bottiglia era l'erba, non la densità.) Prova a tappeto del 28-10 sera, caso 65: varco di 0.7 m fra due muri senza nodi sulla sua linea, cella mancata. Caso 88: robot fermo sotto lo spigolo di un ostacolo, da li' nessun arco del grafo verso la cella (3 direzioni utilizzabili su 24), cella mancata pur essendo raggiungibile all'87%. Con il nuovo obiettivo dopo una ripianificazione fallita (28-10 sera) i casi 65 e 88 riescono. Prova a tappeto delle missioni intere (40 mondi difficili): le celle lasciate aperte erano tutte irraggiungibili con 0.30 m di margine, tranne quelle del caso 8: robot in un corridoio di 0.65 m fra due muri, i nodi della griglia ogni 0.5 m cadono troppo vicini ai muri e il grafo non ne esce (tre celle mancate).
5. **Ricontrollare `find_best_point_in_cell`**, e poi la domanda grossa: **servono ancora le celle?** Se l'obiettivo diventa una traiettoria decisa prima, la griglia di celle e tutta la logica di tentativo/frontiera/rank diventano un'impalcatura di cui si può fare a meno. È il cambiamento più grande dell'elenco: prima il controllo della funzione (c'è già `test_obiettivi.py`), poi una valutazione scritta di cosa si guadagna e cosa si perde, poi semmai il codice.

---

## Ristrutturazione del codice

Fatto (vedi CHANGELOG, 27-10): parametri in `parametri.py`, registro in `registro.py`, scansione e mappe in `spotGrid.py`, movimento in `movements.py`, grafo in `mission_graph.py`, celle in `environmentMap.py`, decisioni in `arcVerification.py`, GraphNav in `navGraphUtils.py`; in `easy_walk.py` le classi `TentativoCella` e `Missione`.

Ancora possibile, non urgente:
- in `TentativoCella` i circa 20 attributi della scansione (`pts_up`, `terrain_up`, `grad_up`, …) raccolti in un solo oggetto `Scan`;
- `mission_switches` e il riepilogo di fine missione restano in `easy_walk.py`: in `registro.py` creerebbero un ciclo di import (`movements` importa `registro`);
- salvare in `scans/` anche le scansioni d'inizio tentativo, del recupero e finale come pacchetti completi (oggi il visualizzatore le ricostruisce dalle viste PRM).
