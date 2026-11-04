#!/usr/bin/env bash
# Missione A BORDO: il codice gira sulla CORE I/O (la Jetson nel payload del lidar), non sul PC.
# Il PC serve solo per copiare i file, avviare la missione e scaricare i log: dopo l'avvio
# si puo' chiudere il terminale o perdere il Wi-Fi, la missione continua (gira in tmux).
#
#   ./a_bordo.sh installa            # una volta sola: pip install dei requisiti sulla CORE I/O
#   ./a_bordo.sh copia               # copia la cartella del progetto (prima di ogni missione)
#   ./a_bordo.sh avvia [argomenti]   # copia, preflight, avvia easy_walk.py in tmux
#                                    #   es. ./a_bordo.sh avvia --rows 2 --cols 3 --cell 4
#   ./a_bordo.sh guarda              # si aggancia al terminale della missione (Ctrl-b d per staccarsi)
#   ./a_bordo.sh log                 # segue mission_log.txt dell'ultima missione
#   ./a_bordo.sh scarica             # riporta sul PC MissionLogs/, MissionMap/, graph/
#   ./a_bordo.sh ssh                 # apre una shell sulla CORE I/O
#
# Variabili (tutte facoltative):
#   CORE_HOST   indirizzo con cui il PC vede la CORE I/O  (default 192.168.80.3, Wi-Fi di Spot)
#   CORE_PORT   porta SSH                                 (default 20022: Spot la inoltra alla CORE I/O)
#   CORE_USER   utente sulla CORE I/O                     (default spot)
#   CORE_DIR    cartella del progetto sulla CORE I/O     (default ~/AutonomousNavigationSpot)
#   ROBOT_DA_BORDO  indirizzo del robot visto dalla CORE I/O (default 192.168.50.3)
#   BOSDYN_CLIENT_USERNAME / BOSDYN_CLIENT_PASSWORD  utente del ROBOT, passati alla missione
#
# Se il PC e' collegato via cavo alla porta Ethernet di Spot: CORE_HOST=10.0.0.3
set -euo pipefail

CORE_HOST="${CORE_HOST:-192.168.80.3}"
CORE_PORT="${CORE_PORT:-20022}"
CORE_USER="${CORE_USER:-spot}"
CORE_DIR="${CORE_DIR:-AutonomousNavigationSpot}"
ROBOT_DA_BORDO="${ROBOT_DA_BORDO:-192.168.50.3}"
SESSIONE="missione"

QUI="$(cd "$(dirname "$0")" && pwd)"
SSH=(ssh -p "$CORE_PORT" -o ServerAliveInterval=5 "$CORE_USER@$CORE_HOST")
RSYNC_SSH="ssh -p $CORE_PORT"

copia() {
    rsync -az --delete -e "$RSYNC_SSH" \
        --exclude '.git' --exclude '__pycache__' \
        --exclude 'MissionLogs' --exclude 'MissionMap' --exclude 'graph' \
        "$QUI/" "$CORE_USER@$CORE_HOST:$CORE_DIR/"
    echo "[a_bordo] File copiati in $CORE_USER@$CORE_HOST:$CORE_DIR"
}

case "${1:-}" in
    installa)
        copia
        "${SSH[@]}" "cd $CORE_DIR && python3 -m pip install --user -r docker-requirements.txt"
        ;;
    copia)
        copia
        ;;
    avvia)
        shift
        : "${BOSDYN_CLIENT_USERNAME:?serve BOSDYN_CLIENT_USERNAME (utente del robot)}"
        : "${BOSDYN_CLIENT_PASSWORD:?serve BOSDYN_CLIENT_PASSWORD (password del robot)}"
        copia
        "${SSH[@]}" "cd $CORE_DIR && python3 preflight.py" || {
            echo "[a_bordo] preflight fallito: missione NON avviata"; exit 1; }
        if "${SSH[@]}" "tmux has-session -t $SESSIONE 2>/dev/null"; then
            echo "[a_bordo] C'e' gia' una missione in corso (./a_bordo.sh guarda)"; exit 1
        fi
        # Le credenziali passano nell'ambiente della sessione tmux, non sulla riga di comando.
        # printf %q protegge spazi e caratteri speciali nella password e negli argomenti.
        ARGS="$(printf '%q ' "$@")"
        "${SSH[@]}" "tmux new-session -d -s $SESSIONE \
            -e BOSDYN_CLIENT_USERNAME=$(printf %q "$BOSDYN_CLIENT_USERNAME") \
            -e BOSDYN_CLIENT_PASSWORD=$(printf %q "$BOSDYN_CLIENT_PASSWORD") \
            -e SPOT_ROBOT_IP=$ROBOT_DA_BORDO \
            'cd $CORE_DIR && python3 easy_walk.py --estop esterno $ARGS; echo; echo FINE MISSIONE; sleep 3600'"
        echo "[a_bordo] Missione avviata a bordo. Segui con: ./a_bordo.sh guarda  oppure  ./a_bordo.sh log"
        ;;
    guarda)
        "${SSH[@]}" -t "tmux attach -t $SESSIONE"
        ;;
    log)
        "${SSH[@]}" -t "cd $CORE_DIR && tail -n 50 -F \"\$(ls -td MissionLogs/Mission_*/ | head -1)mission_log.txt\""
        ;;
    scarica)
        for d in MissionLogs MissionMap graph; do
            rsync -az -e "$RSYNC_SSH" "$CORE_USER@$CORE_HOST:$CORE_DIR/$d/" "$QUI/$d/" 2>/dev/null || true
        done
        echo "[a_bordo] Log e mappe scaricati in $QUI"
        ;;
    ssh)
        "${SSH[@]}" -t
        ;;
    *)
        sed -n '2,24p' "$0"; exit 1
        ;;
esac
