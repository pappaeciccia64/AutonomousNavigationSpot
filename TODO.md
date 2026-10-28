# Da fare

Idee, problemi aperti e decisioni rimandate, con i dati che servono per prenderli. Quando
una cosa è fatta o una decisione è presa si toglie da qui e va nel `CHANGELOG.md`, con il
perché: lì si leggono i progressi, qui solo ciò che manca.

---

## Dopo la prova all'aperto di mercoledì 28-10

Tutte si leggono dal rapporto (`python3 mission_report.py`) e dai file della missione.

| Questione | Cosa guardare | Come si decide |
|---|---|---|
| **Chiusura d'anello finale** (solo completa a fine missione, limite 45 s; la veloce prima dei viaggi è spenta) | Riga `Chiusura d'anello di Spot (completa): N archi in X s`; controllo "Attesa prima di GraphNav" | Se dà una quindicina di archi in circa un minuto e nessun tentativo fallito, resta così. Se ne dà pochi, si valuta se vale il minuto. Se compare `riprovo facendoli scrivere a Spot`, il caricamento con `upload_graph` non funziona sul robot e va sistemato. |
| **Rientri senza chiusura veloce** | Controllo "Scorciatoie GraphNav sulla strada" (metri prima/dopo le scorciatoie), viaggi riusciti/falliti, tempi di viaggio | Stima sul 26-10: senza, +9 m su 56 (~25 s in tutta la missione). Se i viaggi restano tutti riusciti e le strade circa uguali, resta spenta (`GRAPHNAV_SDK_LOOP_BUDGET_S = 0`). |
| **Deriva dell'odometria** | `drift.csv`, controllo "Deriva dell'odometria" | Curva della deriva nel tempo. Se supera ~0.3 m sulle zone viste da più di 10 minuti, si progetta la correzione delle mappe globali (sotto). |
| **`CLEARANCE_MAP_DECIDES`** (`spotGrid.py`, oggi `False`) | Riga DECISIONE "Spazio libero" | Da accendere se il pianificatore ripropone lo stesso varco stretto più volte. Si prova prima sul rigioco. |
| **Andature** | Riga "Andature" del rapporto: tratti HARD con beccheggio sotto 8 gradi (il 26-10 erano 9), beccheggio dei tratti MODERATE | HARD su tratti tranquilli = tempo perso; se MODERATE resta sotto ~8 gradi si può tornare all'andatura normale invece di CRAWL. |
| **Briciole GraphNav ogni 0.75 m** (`GRAPHNAV_WAYPOINT_MIN_SPACING_M`) | Numero di waypoint, tempo di scaricamento della mappa, distanza dei ritorni (controllo "Ritorni con GraphNav") | Portarle a 1.0–1.5 m alleggerisce grafo e scaricamento; da vedere se i ritorni diventano troppo lunghi. |
| **Soglia di rientro a batteria bassa** (`BATTERY_RETURN_PCT`, oggi 30% provvisorio) | `battery.csv`, riga "Batteria" del rapporto: %/min, %/m, punti spesi nel ritorno a wp_0 | La soglia deve coprire il ritorno più lungo possibile (diagonale dell'area con i metri GraphNav reali × %/m) più un margine. Se il ritorno costa pochi punti, si può abbassare. |
| **Missione nelle validazioni** | — | Se è buona, si aggiunge alle missioni di validazione su GitHub (log, scansioni, mappe, viste, GraphNav). |

---

## Lavori già decisi, da fare dopo la ristrutturazione

1. **Erba e ostacoli che "si muovono"** di scansione in scansione. Idea: misurare sui pacchetti di scansione quanto ogni cella sfarfalla fra ostacolo e libero, e incrociarlo con la quota e con `intensity` (il 26-10 il 24% degli ostacoli bassi aveva l'intensità del suolo, contro il 5% di quelli alti). Se il segnale c'è, una regola dietro un interruttore, provata sul rigioco. Per dati etichettati: una prova breve con il robot fermo davanti a erba alta, un cespuglio e un oggetto solido, con foto.
   **28-10 13:50, dati etichettati gratis:** partenza dentro una chiazza di erba alta, e erba alta nelle celle (7,0)-(7,1). Ostacoli attorno alla partenza: altezza mediana 0.11 m sul suolo (95% sotto 0.48), intensità 19 contro 54 del suolo libero; nell'angolo 0.06 m e intensità 19 anche sul "libero" (è tutto prato). Resto della missione: 0.18 m, 95% sotto 0.99 m, intensità 21 contro 70. Effetti: primo tentativo bloccato accanto alla partenza (122 celle su 800 sotto il corpo diventate ostacolo appena il robot si è girato), (7,0) senza strada nel grafo, e il ritorno a wp_0 fermato da Spot (`STUCK`) a 0.63 m dall'arrivo: l'erba blocca anche l'evitamento ostacoli di GraphNav, non solo la nostra mappa.
4. **Ritorno a wp_0 quasi arrivato.** Se GraphNav si ferma (`STUCK`) entro ~1 m da wp_0, contarlo come arrivato e fare comunque il controllo del fiducial (13:50: fermo a 0.63 m, niente misura della deriva). In pratica, intanto: partire su terreno basso o nudo, non dentro l'erba alta.
2. **Movimento più fluido**, in due stadi. *Primo*: girarsi camminando e non fermarsi fra un tratto e l'altro (oggi 0.12 m/s di media). *Secondo, più ambizioso*: rendere movimento, scansione e ripianificazione non bloccanti fra loro, così il robot non si ferma ad aspettare la scansione o il grafo. Oggi il ciclo è sequenziale e l'unica cosa che gira in parallelo è il fronte sicuro. Il secondo stadio cambia la struttura del ciclo, quindi va fatto dopo il primo e con il test end-to-end a confronto riga per riga. Entrambi dietro un interruttore, poi poche celle sul campo prima di lasciarli accesi.
3. **Correzione della deriva nelle mappe globali**, con i dati di `drift.csv`: riallineare la mappa globale con le localizzazioni di GraphNav, oppure dimenticare più in fretta le osservazioni vecchie quando il robot ci ripassa.

---

## Idee da valutare (28-10)

Non ancora decise: prima serve dire cosa fanno esattamente e come si prova che funzionano.

1. ~~**Percentuale di esplorazione completata.**~~ Fatta il 28-10 (vedi CHANGELOG): riga `[PROGRESSO]` con celle chiuse sul totale, area vista, stima del tempo che manca e batteria; nel rapporto e in `mission_summary.json`.
2. ~~**Rientro con GraphNav: evitare la rotazione inutile.**~~ Fatta il 28-10 (vedi CHANGELOG): all'arrivo il robot è girato verso la cella da tentare (`GRAPHNAV_ARRIVAL_HEADING`). Da confermare sul campo con la riga "Rotazione dopo gli arrivi GraphNav" del rapporto.
3. **Obiettivo scelto da noi sul grafo globale, a esplorazione finita.** Oggi l'obiettivo è sempre il punto di una cella da esplorare. Serve un modo per dire al robot «vai lì» su una mappa già costruita, usando il grafo e le mappe globali invece di rifare l'esplorazione. È un modo d'uso nuovo, non una modifica al ciclo attuale: probabilmente uno script a parte.
4. **Infittire il grafo nelle zone strette, in modo locale e dinamico.** Oggi la densità dei nodi è decisa una volta per tutta l'area (`MISSION_RANDOM_DENSITY`). Nei varchi stretti servono più nodi, altrove sono tempo sprecato. Da definire come si riconosce «zona stretta» (lo spazio libero misurato è già disponibile) e se i nodi nuovi si aggiungono al grafo esistente o se si ricostruisce la zona. Si prova sul rigioco prima che sul robot.
5. **Ricontrollare `find_best_point_in_cell`**, e poi la domanda grossa: **servono ancora le celle?** Se l'obiettivo diventa una traiettoria decisa prima, la griglia di celle e tutta la logica di tentativo/frontiera/rank diventano un'impalcatura di cui si può fare a meno. È il cambiamento più grande dell'elenco: prima il controllo della funzione (c'è già `test_obiettivi.py`), poi una valutazione scritta di cosa si guadagna e cosa si perde, poi semmai il codice.

---

## Ristrutturazione del codice

Fatto (vedi CHANGELOG, 27-10): parametri in `parametri.py`, registro in `registro.py`, scansione e mappe in `spotGrid.py`, movimento in `movements.py`, grafo in `mission_graph.py`, celle in `environmentMap.py`, decisioni in `arcVerification.py`, GraphNav in `navGraphUtils.py`; in `easy_walk.py` le classi `TentativoCella` e `Missione`.

Ancora possibile, non urgente:
- in `TentativoCella` i circa 20 attributi della scansione (`pts_up`, `terrain_up`, `grad_up`, …) raccolti in un solo oggetto `Scan`;
- `mission_switches` e il riepilogo di fine missione restano in `easy_walk.py`: in `registro.py` creerebbero un ciclo di import (`movements` importa `registro`).
