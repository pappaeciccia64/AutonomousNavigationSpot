"""
prova_fiducial.py -- quali fiducial vede Spot adesso, e quale sceglierebbe la missione.

    python3 prova_fiducial.py            # una lettura ogni secondo per 10 s
    python3 prova_fiducial.py 30         # per 30 s

Solo lettura: niente lease, niente accensione dei motori, il robot non si muove. Va bene
anche con il robot seduto (serve solo che sia acceso). Usa la stessa funzione della missione
(apriltags.find_fiducials) e lo stesso criterio di scelta (il piu' vicino, oppure
parametri.START_FIDUCIAL_ID se e' un numero).

Spot riconosce solo i fiducial AprilTag 36h11 (quelli di Boston Dynamics), non gli ArUco.
"""
import sys
import time
from types import SimpleNamespace

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # il pacchetto spot_explorer

from spot_explorer.graphnav import apriltags
from spot_explorer import config as P
from spot_explorer.robot import connection as spotLogInUtils


def main():
    durata = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    options = SimpleNamespace(name="provaFiducial", hostname="192.168.80.3", verbose=False,
                              recording_user_name="", recording_session_name="")
    robot, _lease, robot_state_client, _meta = spotLogInUtils.setLogInfo(options)
    print(f"Collegato. Cerco fiducial per {durata:.0f} s "
          f"({'qualunque ID' if P.START_FIDUCIAL_ID is None else f'solo ID {P.START_FIDUCIAL_ID}'}).\n")
    t_end = time.time() + durata
    visti = {}
    while time.time() < t_end:
        found = apriltags.find_fiducials(robot, robot_state_client,
                                             fiducial_id=P.START_FIDUCIAL_ID)
        stamp = time.strftime('%H:%M:%S')
        if found:
            for f in found:
                visti[f['id']] = visti.get(f['id'], 0) + 1
            print(f"{stamp}  la missione sceglierebbe: {apriltags.describe_fiducial(found[0])}")
            for f in found[1:]:
                print(f"           visto anche: {apriltags.describe_fiducial(f)}")
        else:
            print(f"{stamp}  nessun fiducial visto")
        time.sleep(1.0)
    print("\nRiepilogo: " + (", ".join(f"tag {k} in {n} letture" for k, n in sorted(visti.items()))
                             if visti else "nessun fiducial visto. Controlla che sia un AprilTag "
                                           "36h11, in luce, a 1-2 m e non troppo di taglio."))


if __name__ == '__main__':
    main()
