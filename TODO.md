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
| **Missione nelle validazioni** | — | Se è buona, si aggiunge alle missioni di validazione su GitHub (log, scansioni, mappe, viste, GraphNav). |

---

## Lavori già decisi, da fare dopo la ristrutturazione

1. **Erba e ostacoli che "si muovono"** di scansione in scansione. Idea: misurare sui pacchetti di scansione quanto ogni cella sfarfalla fra ostacolo e libero, e incrociarlo con la quota e con `intensity` (il 26-10 il 24% degli ostacoli bassi aveva l'intensità del suolo, contro il 5% di quelli alti). Se il segnale c'è, una regola dietro un interruttore, provata sul rigioco. Per dati etichettati: una prova breve con il robot fermo davanti a erba alta, un cespuglio e un oggetto solido, con foto.
2. **Movimento più fluido**: girarsi camminando e non fermarsi fra un tratto e l'altro (oggi 0.12 m/s di media). Dietro un interruttore, provato con il test end-to-end, poi poche celle sul campo prima di lasciarlo acceso.
3. **Correzione della deriva nelle mappe globali**, con i dati di `drift.csv`: riallineare la mappa globale con le localizzazioni di GraphNav, oppure dimenticare più in fretta le osservazioni vecchie quando il robot ci ripassa.

---

## Ristrutturazione del codice

Fatto (vedi CHANGELOG, 27-10): parametri in `parametri.py`, registro in `registro.py`, scansione e mappe in `spotGrid.py`, movimento in `movements.py`, grafo in `mission_graph.py`, celle in `environmentMap.py`, decisioni in `arcVerification.py`, GraphNav in `navGraphUtils.py`; in `easy_walk.py` le classi `TentativoCella` e `Missione`.

Ancora possibile, non urgente:
- in `TentativoCella` i circa 20 attributi della scansione (`pts_up`, `terrain_up`, `grad_up`, …) raccolti in un solo oggetto `Scan`;
- `mission_switches` e il riepilogo di fine missione restano in `easy_walk.py`: in `registro.py` creerebbero un ciclo di import (`movements` importa `registro`).
