#!/usr/bin/env python3
"""
Confronta due registrazioni dei log del test end-to-end (E2E_LOG_DIR), ignorando cio' che
cambia da un'esecuzione all'altra senza cambiare il comportamento: tempi, cartelle
temporanee, orari. Serve a verificare che una ristrutturazione del codice non cambi niente.

    E2E_LOG_DIR=/tmp/prima python3 test_e2e_loop.py
    ... modifica ...
    E2E_LOG_DIR=/tmp/dopo  python3 test_e2e_loop.py
    python3 confronta_log.py /tmp/prima /tmp/dopo

Uscita 0 = identici, 1 = differenze (le prime per ogni scenario sono stampate).
"""
import difflib
import os
import re
import sys

_RULES = [
    (re.compile(r'\d+(\.\d+)?\s*s\b'), '<T>s'),                 # durate
    (re.compile(r'\d+(\.\d+)?\s*ms\b'), '<T>ms'),
    (re.compile(r'/tmp/[^\s\'",)]+'), '<TMP>'),                  # cartelle temporanee
    (re.compile(r'\b\d{2}:\d{2}:\d{2}(\.\d+)?\b'), '<ORA>'),     # orari
    (re.compile(r'0x[0-9a-f]+'), '<ADDR>'),                      # indirizzi di oggetti
    (re.compile(r'\d{2}-\d{2}-\d{4}_\d{2}-\d{2}-\d{2}'), '<DATA>'),   # nomi delle cartelle
    (re.compile(r'\d+(\.\d+)?\s*min\b'), '<T>min'),               # eta' dei waypoint
]


def normalize(text):
    out = []
    for line in text.splitlines():
        if line.startswith('[TEMPO]') or '[TEMPO]' in line:
            continue                                            # righe di soli tempi
        for rx, rep in _RULES:
            line = rx.sub(rep, line)
        if re.search(r'<T>s\s+\(\s*[\d.]+%\)', line):
            continue                                            # ripartizione dei tempi
        out.append(line)
    return out


def main(a, b):
    names = sorted(set(os.listdir(a)) | set(os.listdir(b)))
    bad = 0
    for n in names:
        pa, pb = os.path.join(a, n), os.path.join(b, n)
        if not (os.path.exists(pa) and os.path.exists(pb)):
            print(f"[MANCA] {n}: solo in {'prima' if os.path.exists(pa) else 'dopo'}")
            bad += 1
            continue
        la, lb = normalize(open(pa).read()), normalize(open(pb).read())
        if la != lb:
            bad += 1
            diff = list(difflib.unified_diff(la, lb, 'prima', 'dopo', n=1, lineterm=''))
            print(f"[DIVERSO] {n}: {sum(1 for d in diff if d[:1] in '+-') - 2} righe")
            print("\n".join(diff[:30]))
        else:
            print(f"[UGUALE] {n} ({len(la)} righe)")
    print("\nIDENTICI" if not bad else f"\n{bad} SCENARI DIVERSI")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
