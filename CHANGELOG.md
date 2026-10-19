# Storia delle modifiche

Le modifiche più recenti in alto. Per ognuna: cosa è cambiato, perché, e il dato di campo che l'ha motivata. Il codice descrive solo come funziona oggi; da qui si capisce come ci si è arrivati. Il dettaglio riga per riga è nella storia di git.

Le missioni sono indicate come `giorno-mese ora` (per esempio `13-10 12:40`); i loro dati stanno in `MissionLogs/` e `MissionMap/`.

---

## 2026-10-14

### Fronte sicuro (`arcVerification.py`)
- Tolta `AsyncArcVerificationTracker`, mai usata (chiamava un servizio che non esiste).

---

## 2026-10-12

### Fronte sicuro (`arcVerification.py`)
- **Celle sotto il corpo mai ostacolo** (`_under_body`). Missione 12-10 10:36: il fronte si è fermato due volte, a metà movimento e da fermo, su un punto a 8–9 cm dal centro del robot mentre passava rasente a un tavolo (`obstacle_distance` sotto il robot 0.21 m contro 0.25 di mezza larghezza).
- **Due soglie: 0.30 m per partire, 0.25 m per fermarsi a metà movimento** (`air_m`, `FRONTIER_ABORT_AIR_M`). Richiesta dell'utente: un tratto dichiarato libero si percorre; a movimento in corso ci si ferma solo per un'intrusione vera.
- **Nel log la soglia effettiva e se ha deciso l'isteresi.** Missione 12-10 14:47: 23 fermate su 33 dicevano "ostacolo a 0.32 m (minimo 0.30 m)", che sembrava una contraddizione.
- Regola di uscita: sotto il corpo il riferimento è la distanza del robot adesso, non le letture sotto la pancia.

---

## 2026-10-07

### Fronte sicuro (`arcVerification.py`)
- **Filtro temporale su `obstacle_distance`: mediana su 3 scansioni e isteresi di 4 cm.** Missione 07-10 14:0x, robot fermo e percorso identico: una cella ha letto 0.296 e poi 0.336 su una soglia di 0.300, e il fronte è passato da "ostacolo" a "libero" senza che niente si muovesse. Rumore misurato fra due scansioni consecutive: 1.5 cm in media, 7.3 cm di punta.
- Lo scorrimento fra scansioni si misura fra griglie consecutive e non da un'origine assoluta: `cell_size` arriva come float32 (0.029999999329447746) e la cronologia si azzerava a ogni scansione.
- **"Cella vista" confrontata con `TERRAIN_VALID_MIN` e non con 0.** Dopo scala e offset lo 0 di `terrain_valid` diventa ±5.5e-17 con segno variabile: il vecchio `> 0.0` dichiarava vista ogni cella in 10 scansioni su 16.

---

## 2026-10-06

### Fronte sicuro (`arcVerification.py`)
- **Il fronte sicuro sostituisce la verifica arco per arco con verdetti permanenti.** Prima missione in esterno, 05-10 16:05: cinque ritirate da terreno libero. Gli archi venivano giudicati mentre il robot camminava, da lontano, troncati sul bordo della finestra, e il verdetto non veniva più riesaminato.
- Celle mai scritte dai sensori trattate come non viste: il fronte si ferma lì con `fine_dati`.
- (sera, dopo la revisione) **Regola di uscita da uno spazio stretto corretta.** La soglia abbassata nel primo metro valeva per qualunque ostacolo: un palo a 15 cm dal fianco, 70 cm più avanti, veniva accettato perché il robot partiva vicino a un altro oggetto. Ora vale "non avvicinarsi più di quanto già si sia".
- (sera) Tolleranza della regola di uscita portata a 1.5 celle: con 2 cm, lungo un muro a 5 gradi dal percorso, 257 blocchi su 360 posizioni (prima 13).
