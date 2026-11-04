# Missione a bordo della CORE I/O: cosa fare

**Obiettivo.** Far girare `easy_walk.py` sulla CORE I/O di Boston Dynamics (la Jetson del payload lidar), così tutti i calcoli si fanno a bordo. Il PC serve solo per copiare il codice, avviare la missione e scaricare i log. Il lavoro sta nel branch `core-io-a-bordo`: `AdL-version` non va toccato.

## Cosa c'è già nel branch (mai provato su un robot vero)
- In `easy_walk.py`:
  - `--robot IP` (oppure la variabile `SPOT_ROBOT_IP`; default `192.168.80.3`);
  - `--estop esterno`: la missione non registra il suo E-Stop, lo tiene il tablet.
- `a_bordo.sh` (si lancia dal PC): `installa`, `copia`, `avvia [argomenti]`, `guarda`, `log`, `scarica`, `ssh`.
  - Copia la cartella con `rsync` sulla porta SSH 20022.
  - Con `avvia` lancia `preflight.py`, poi `easy_walk.py --estop esterno` dentro `tmux`, con `SPOT_ROBOT_IP=192.168.50.3` e le credenziali prese da `BOSDYN_CLIENT_USERNAME` e `BOSDYN_CLIENT_PASSWORD`.
- Sezione "Missione a bordo" nel `README.md`.

## Rete
| Da | A | Indirizzo |
|---|---|---|
| PC (Wi-Fi di Spot) | robot | `192.168.80.3` (via cavo: `10.0.0.3`) |
| PC | SSH della CORE I/O | `ssh -p 20022 spot@192.168.80.3` |
| PC | portale della CORE I/O | `https://192.168.80.3:21443` |
| CORE I/O | robot | `192.168.50.3` (la CORE I/O è `192.168.50.5`) |

## Step
1. **Portale** (`https://192.168.80.3:21443`): entrare con l'utente del robot e scegliere la password dell'utente `spot` della CORE I/O.
2. **SSH:**
   ```bash
   ssh -p 20022 spot@192.168.80.3
   ```
   Poi controllare:
   - `python3 --version` deve dire 3.8;
   - `which tmux rsync pip3` deve trovare tutti e tre;
   - `ping 192.168.50.3` (il robot) deve rispondere;
   - `ping 8.8.8.8` dice se la Jetson è su internet.
3. **Chiave SSH** (consigliato):
   ```bash
   ssh-copy-id -p 20022 spot@192.168.80.3
   ```
4. **Programmi e librerie:**
   - Se mancano: `sudo apt install -y tmux rsync python3-pip`.
   - Installare l'SDK della **stessa versione del PC** (`pip3 show bosdyn-client`), più numpy e scipy:
     ```bash
     pip3 install --user bosdyn-client==X bosdyn-core==X bosdyn-api==X numpy scipy
     ```
   - Se la Jetson non è su internet: sul PC `pip3 download --platform manylinux2014_aarch64 --python-version 38 --only-binary=:all: ...`, poi `scp -P 20022` sulla Jetson e lì `pip3 install --no-index --find-links`.
5. **Prova del login dalla CORE I/O** (non muove il robot):
   ```bash
   python3 -m bosdyn.client --username U --password P 192.168.50.3 id
   ```
6. **Prima missione, piccola:**
   - dal tablet: E-Stop attivo e rilasciato, poi rilasciare il controllo (lease);
   - dal PC:
     ```bash
     export BOSDYN_CLIENT_USERNAME=U BOSDYN_CLIENT_PASSWORD=P
     ```
     ```bash
     ./a_bordo.sh avvia --rows 1 --cols 2 --cell 2
     ```
     ```bash
     ./a_bordo.sh log
     ```
     ```bash
     ./a_bordo.sh scarica
     ```
   - Per fermare il robot: l'E-Stop del tablet.

## Da sapere
- **E-Stop.** Se lo tiene la missione sulla CORE I/O non scatta mai, anche se si perde il collegamento con l'operatore. Per questo a bordo si usa `--estop esterno` e l'E-Stop resta sul tablet.
- **Credenziali.** Sul PC di sviluppo l'SDK è stato modificato per avere le credenziali fisse; sulla Jetson è installato da zero, quindi servono le variabili `BOSDYN_CLIENT_*`.
- **Branch `COREIOv1.0`** (di GennaiG00, giugno 2026): una vecchia prova come Extension `.spx` (Dockerfile, `docker-compose.yml`, `manifest.json`, login con le credenziali del payload). Parte da codice vecchio (`easy_walk_CORE.py`) e ha GUID e secret del payload scritti nel codice. È utile come riferimento per dopo.

## Prossimi passi possibili
- Login con le credenziali del payload (`robot.authenticate_from_payload_credentials`, file di solito in `/opt/payload_credentials/`), così non serve la password.
- Rifare l'Extension `.spx` con il codice attuale, quando sarà stabile.
