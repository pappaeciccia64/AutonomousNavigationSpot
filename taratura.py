#!/usr/bin/env python3
"""
TARATURA dai dati: pesi del costo del PRM e soglie dell'andatura, misurati sui log.

    python3 taratura.py                          # tutte le missioni in MissionLogs/
    python3 taratura.py --missioni Mission_26-10-2026_15-11-21 Mission_22-10-2026_16-04-22
    python3 taratura.py --dal 2026-10-22         # solo le missioni da quel giorno
    python3 taratura.py --beccheggio 12          # soglia del "tratto inclinato" per le soglie
    python3 taratura.py --prove ProveAndatura/prove.csv   # anche le prove su tracciato

Non tocca il robot e non cambia nessun parametro: stampa cosa dicono i dati. Le decisioni si
prendono leggendo, e vanno nel CHANGELOG con i numeri che le giustificano.

Cosa stampa:
  1. Tratti per livello d'andatura: quanti, riusciti, beccheggio e rollio, velocita' reale
     contro il limite impostato (se la reale e' molto sotto, il limite non conta).
  2. COSTO DEL PRM. Regressione del tempo di un tratto riuscito su lunghezza, pendenza
     longitudinale, pendenza laterale, rugosita':
        durata = a*lunghezza + b*pend_long + c*pend_lat + d*rugosita' + k
     Dijkstra guarda solo i rapporti fra i pesi, quindi i pesi suggeriti sono b/a, c/a, d/a
     (con alpha = 1): "quanti metri di strada vale un'unita' di pendenza", in tempo.
     Due versioni: senza il livello d'andatura (effetto totale: la pendenza costa anche
     perche' fa passare al CRAWL) e con il livello (effetto diretto, piu' il costo in metri
     equivalenti di ogni cambio di livello). Intervalli al 90% col bootstrap.
  3. FALLIMENTI per fasce di pendenza e rugosita', separando i tratti con 0 m percorsi
     (rifiuti, blocchi del fronte: non dipendono dal terreno) dagli altri.
  4. SOGLIE DELL'ANDATURA come classificatore: per ogni soglia di pendenza, quanti tratti
     "inclinati" (beccheggio di picco > --beccheggio gradi) cadono sopra soglia, quanti tratti
     tranquilli finiscono sopra soglia per niente, e che quota dei tratti va sopra. Poi
     quanto spesso le soglie di rugosita' scattano davvero.
  5. PROVE SU TRACCIATO (prova_andatura.py), se ci sono: per terreno e configurazione,
     riuscita, tempo, velocita', oscillazioni (RMS di beccheggio e rollio attorno alla media,
     che su una rampa e' la pendenza stessa), slittamento dei piedi, attrito stimato da Spot,
     energia per metro; e la pendenza stimata contro quella misurata con l'inclinometro.

Avvertenze: i dati delle missioni non sono un esperimento controllato (l'andatura dipende
dalla pendenza, quindi i due effetti si confondono: e' il motivo delle due regressioni) e il
beccheggio alto su una rampa e' normale, non e' instabilita'. Per la scelta dell'andatura
fanno fede le prove su tracciato.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

import spotGrid

TIERS = ('PLAIN', 'MODERATE', 'HARD')
CAP_VEL = {'PLAIN': spotGrid.GAIT_VEL_PLAIN, 'MODERATE': spotGrid.GAIT_VEL_MODERATE,
           'HARD': spotGrid.GAIT_VEL_HARD}


# -----------------------------------------------------------------------------------------
# Lettura
# -----------------------------------------------------------------------------------------
def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def _bool(v):
    return str(v).strip().lower() in ('true', '1', 'si', 'yes')


def _data_missione(nome):
    """'Mission_26-10-2026_15-11-21' -> '2026-10-26' (None se il nome e' diverso)."""
    try:
        d = nome.split('_')[1]
        g, m, a = d.split('-')
        return f"{a}-{m}-{g}"
    except Exception:
        return None


def carica_segmenti(cartella, missioni=None, dal=None):
    righe = []
    for path in sorted(glob.glob(os.path.join(cartella, '*', 'segments.csv'))):
        nome = os.path.basename(os.path.dirname(path))
        if missioni and nome not in missioni:
            continue
        if dal and (_data_missione(nome) or '') < dal:
            continue
        with open(path, newline='', encoding='utf-8') as f:
            for r in csv.DictReader(f):
                r['_missione'] = nome
                righe.append(r)
    if not righe:
        return None
    col = lambda k: np.array([_num(r.get(k)) for r in righe])
    return {
        'missione': np.array([r['_missione'] for r in righe]),
        'tier': np.array([r.get('tier', '') for r in righe]),
        'success': np.array([_bool(r.get('success')) for r in righe]),
        'override': np.array([_bool(r.get('gait_override')) for r in righe]),
        'dist': col('dist_m'),
        'dist_trav': col('dist_traveled_m'),
        'dist_cmd': col('dist_commanded_m'),
        'durata': col('duration_s'),
        'long': np.abs(col('long_slope')),
        'lat': np.abs(col('lat_slope')),
        'rough': col('roughness'),
        'pitch': col('pitch_peak_abs_deg'),
        'roll': col('roll_peak_abs_deg'),
    }


# -----------------------------------------------------------------------------------------
# Strumenti
# -----------------------------------------------------------------------------------------
def regressione(X, y):
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1.0 - float(((y - pred) ** 2).sum()) / ss_tot if ss_tot > 0 else float('nan')
    return coef, r2


def bootstrap_rapporti(X, y, idx_num, idx_den=0, n=300, seme=0):
    """Intervallo al 90% di coef[idx_num]/coef[idx_den], ricampionando le righe."""
    rng = np.random.default_rng(seme)
    out = {i: [] for i in idx_num}
    for _ in range(n):
        s = rng.integers(0, len(y), len(y))
        try:
            c, *_ = np.linalg.lstsq(X[s], y[s], rcond=None)
        except np.linalg.LinAlgError:
            continue
        if c[idx_den] <= 0:
            continue
        for i in idx_num:
            out[i].append(c[i] / c[idx_den])
    return {i: (np.percentile(v, 5), np.percentile(v, 95)) if v else (np.nan, np.nan)
            for i, v in out.items()}


def titolo(t):
    print("\n" + "=" * 92 + f"\n{t}\n" + "=" * 92)


# -----------------------------------------------------------------------------------------
# Sezioni
# -----------------------------------------------------------------------------------------
def sezione_livelli(S):
    titolo("1. TRATTI PER LIVELLO D'ANDATURA")
    v = S['dist_trav'] / S['durata']
    print(f"{'livello':9s} {'tratti':>6s} {'riusciti':>9s} {'becch.med':>9s} {'becch.p95':>9s} "
          f"{'rollio.med':>10s} {'vel.med':>8s} {'limite':>7s}")
    for t in TIERS:
        m = S['tier'] == t
        if not m.any():
            continue
        ok = m & S['success'] & (S['durata'] > 0)
        print(f"{t:9s} {m.sum():6d} {S['success'][m].mean():9.0%} {np.nanmedian(S['pitch'][m]):8.1f}° "
              f"{np.nanpercentile(S['pitch'][m], 95):8.1f}° {np.nanmedian(S['roll'][m]):9.1f}° "
              f"{np.nanmedian(v[ok]):7.2f}  {CAP_VEL[t]:6.2f}")
    if S['override'].any():
        print(f"(attenzione: {S['override'].sum()} tratti con andatura imposta, gait_override)")
    print("Velocita' in m/s. Se la velocita' reale e' molto sotto il limite, il limite non e' "
          "cio' che rallenta il robot.")


def sezione_costo(S):
    titolo("2. COSTO DEL PRM: quanto costa davvero la pendenza (regressione del tempo)")
    ok = S['success'] & (S['durata'] > 0) & (S['dist_trav'] > 0.05)
    for k in ('long', 'lat', 'rough'):
        ok &= np.isfinite(S[k])
    n = int(ok.sum())
    if n < 30:
        print(f"Solo {n} tratti riusciti utilizzabili: troppo pochi per una regressione.")
        return
    d, lo, la, ro = S['dist_trav'][ok], S['long'][ok], S['lat'][ok], S['rough'][ok]
    y = S['durata'][ok]
    tier = S['tier'][ok]
    uno = np.ones(n)

    X1 = np.c_[d, lo, la, ro, uno]
    c1, r21 = regressione(X1, y)
    ic1 = bootstrap_rapporti(X1, y, [1, 2, 3])
    mod, hard = (tier == 'MODERATE').astype(float), (tier == 'HARD').astype(float)
    X2 = np.c_[d, lo, la, ro, mod, hard, uno]
    c2, r22 = regressione(X2, y)
    ic2 = bootstrap_rapporti(X2, y, [1, 2, 3, 4, 5])

    print(f"{n} tratti riusciti. Tempo per metro: {c1[0]:.2f} s/m (senza livello), "
          f"{c2[0]:.2f} s/m (con livello).\n")
    print(f"{'peso (alpha = 1)':28s} {'senza livello':>22s} {'con livello':>22s}")
    nomi = [('beta  (pend. longitudinale)', 1), ('beta_lat (pend. laterale)', 2),
            ('gamma (rugosita\')', 3)]
    for nome, i in nomi:
        print(f"{nome:28s} {c1[i] / c1[0]:6.2f} [{ic1[i][0]:5.2f};{ic1[i][1]:5.2f}]  "
              f"{c2[i] / c2[0]:6.2f} [{ic2[i][0]:5.2f};{ic2[i][1]:5.2f}]")
    print(f"{'passaggio a MODERATE (m)':28s} {'':22s} {c2[4] / c2[0]:6.2f} [{ic2[4][0]:5.2f};{ic2[4][1]:5.2f}]")
    print(f"{'passaggio a HARD (m)':28s} {'':22s} {c2[5] / c2[0]:6.2f} [{ic2[5][0]:5.2f};{ic2[5][1]:5.2f}]")
    print(f"{'R² (quota di tempo spiegata)':28s} {r21:22.2f} {r22:22.2f}")
    print(f"\nOggi nel PRM: beta = 2.0, beta_lat = {2.0 * spotGrid.LATERAL_SLOPE_COST_MULTIPLIER:.1f}, "
          f"gamma = 0 (prm_graph.PRM, spotGrid.LATERAL_SLOPE_COST_MULTIPLIER).")
    rq = np.nanpercentile(S['rough'], [50, 95])
    print(f"Rugosita' tipica: mediana {rq[0]:.3f}, 95° percentile {rq[1]:.3f}: gamma * {rq[1]:.3f} "
          f"e' quanto pesa la rugosita' su un arco ruvido; se e' molto meno di un metro, gamma = 0 "
          f"non cambia i percorsi.")
    print("Lettura: 'senza livello' e' il costo totale in tempo (pendenza + CRAWL che ne segue); "
          "'con livello' separa i due. I valori fra parentesi sono l'intervallo al 90%: se "
          "contiene il valore di oggi, i dati non chiedono di cambiarlo. Il tempo non e' "
          "l'unico criterio: beta_lat alto puo' restare come margine di sicurezza sul rollio.")


def _fasce(nome, x, fallito, soglie):
    parti = []
    for q in soglie:
        sotto, sopra = x < q, x >= q
        if sopra.sum() == 0:
            parti.append(f"<{q:g}: {fallito[sotto].mean():.0%} (n{sotto.sum()}) | >=: nessuno")
            continue
        parti.append(f"<{q:g}: {fallito[sotto].mean():.0%} (n{sotto.sum()}) | "
                     f">=: {fallito[sopra].mean():.0%} (n{sopra.sum()})")
    print(f"  {nome}:")
    for p in parti:
        print(f"     {p}")


def sezione_fallimenti(S):
    titolo("3. FALLIMENTI: dipendono dal terreno?")
    fall = ~S['success']
    fermo = fall & (np.nan_to_num(S['dist_trav']) < 0.05)
    print(f"Tratti falliti: {fall.sum()} su {len(fall)} ({fall.mean():.1%}); di questi {fermo.sum()} "
          f"con 0 m percorsi (rifiuti, blocchi: non e' il terreno).")
    fisico = fall & ~fermo
    print(f"Fallimenti con il robot in movimento: {fisico.sum()} ({fisico.mean():.1%} dei tratti). "
          f"Tasso per fascia:")
    _fasce("pendenza longitudinale", S['long'], fisico, [0.1, 0.2, 0.3])
    _fasce("pendenza laterale", S['lat'], fisico, [0.05, 0.1, 0.2])
    _fasce("rugosita'", S['rough'], fisico, [0.02, 0.04])
    print("Se il tasso sopra soglia non e' chiaramente piu' alto di quello sotto, il rischio non "
          "va messo nei pesi della pendenza (le penalita' sugli archi lo gestiscono gia').")


def sezione_soglie(S, beccheggio):
    titolo(f"4. SOGLIE DELL'ANDATURA (tratto 'inclinato' = beccheggio di picco > {beccheggio:g}°)")
    ok = np.isfinite(S['pitch']) & np.isfinite(S['long'])
    lab, s = S['pitch'][ok] > beccheggio, S['long'][ok]
    if lab.sum() == 0 or (~lab).sum() == 0:
        print("Servono tratti sia inclinati sia tranquilli.")
        return
    print(f"{lab.sum()} tratti inclinati su {ok.sum()}.")
    print(f"{'soglia':>7s} {'inclinati presi':>16s} {'falsi allarmi':>14s} {'tratti sopra':>13s}")
    for t in (0.10, 0.125, 0.15, 0.175, 0.20, 0.225, 0.25, 0.30):
        nota = ""
        if abs(t - spotGrid.GAIT_PLAIN_SLOPE_MAX) < 1e-9:
            nota = "  <- oggi PLAIN -> MODERATE"
        elif abs(t - spotGrid.GAIT_MODERATE_SLOPE_MAX) < 1e-9:
            nota = "  <- oggi MODERATE -> HARD"
        print(f"{t:7.3f} {(s[lab] >= t).mean():16.0%} {(s[~lab] >= t).mean():14.0%} "
              f"{(s >= t).mean():13.0%}{nota}")
    print("'inclinati presi': dei tratti inclinati, quanti sarebbero sopra soglia (piu' alto e' "
          "meglio per la soglia di HARD). 'falsi allarmi': dei tratti tranquilli, quanti sopra "
          "soglia per niente (ognuno e' un tratto in CRAWL inutile).")
    r = S['rough'][np.isfinite(S['rough'])]
    for nome, val in (('GAIT_PLAIN_ROUGH_MAX', spotGrid.GAIT_PLAIN_ROUGH_MAX),
                      ('GAIT_MODERATE_ROUGH_MAX', spotGrid.GAIT_MODERATE_ROUGH_MAX)):
        print(f"{nome} = {val:g}: superata in {(r >= val).sum()} tratti su {len(r)} "
              f"({(r >= val).mean():.1%}).")
    print(f"Rugosita' misurata: mediana {np.median(r):.3f}, 95° percentile {np.percentile(r, 95):.3f}, "
          f"massimo {r.max():.3f}. Una soglia che non scatta mai non decide niente.")


def sezione_prove(path):
    titolo(f"5. PROVE SU TRACCIATO ({path})")
    if not os.path.exists(path):
        print("Nessun file: si crea con prova_andatura.py.")
        return
    with open(path, newline='', encoding='utf-8') as f:
        righe = list(csv.DictReader(f))
    if not righe:
        print("File vuoto.")
        return
    gruppi = {}
    for r in righe:
        gruppi.setdefault((r['terreno'], r['verso'], r['config']), []).append(r)
    print(f"{'terreno':14s} {'verso':7s} {'config':14s} {'n':>2s} {'riusc':>5s} {'tempo':>6s} "
          f"{'vel':>5s} {'becchRMS':>8s} {'rollRMS':>7s} {'slitt':>6s} {'mu_est':>6s} {'J/m':>5s} "
          f"{'scarto':>6s}")
    for (ter, verso, cfg), rr in sorted(gruppi.items()):
        if cfg == 'solo_stima':
            continue
        g = lambda k: np.array([_num(r.get(k)) for r in rr])
        ok = np.array([_bool(r.get('success')) for r in rr])
        dist = g('dist_traveled_m')
        nm = lambda a: np.nanmean(a) if np.isfinite(a).any() else float('nan')
        scarto = np.abs(g('dist_commanded_m') - dist)
        print(f"{ter[:14]:14s} {verso[:7]:7s} {cfg[:14]:14s} {len(rr):2d} {ok.mean():5.0%} "
              f"{nm(g('duration_s')):6.1f} {nm(g('speed_mps')):5.2f} {nm(g('pitch_rms_deg')):8.2f} "
              f"{nm(g('roll_rms_deg')):7.2f} {nm(g('slip_peak_m')):6.3f} {nm(g('mu_est_mean')):6.2f} "
              f"{nm(g('energy_per_m_j')):5.0f} {nm(scarto):6.2f}")
    print("tempo in s, vel in m/s, RMS in gradi attorno alla media del tratto (oscillazione, non "
          "pendenza), slitt = picco dello slittamento di un piede in appoggio (m), mu_est = attrito "
          "stimato da Spot sotto i piedi, J/m = energia per metro, scarto = |comandato - percorso| m.")
    print("Per ogni terreno e verso, la configurazione migliore e' quella che riesce sempre e ha "
          "oscillazioni e slittamento bassi; a parita', la piu' veloce o la meno energivora.")

    # Stima della pendenza contro l'inclinometro
    coppie = []
    for r in righe:
        vera, stima = _num(r.get('pendenza_vera_gradi')), _num(r.get('stima_long'))
        becc = _num(r.get('pitch_mean_deg'))
        if np.isfinite(vera) and np.isfinite(stima):
            # La stima e' sempre positiva (90° percentile del modulo); al ritorno la pendenza
            # vera e' negativa (discesa): si confrontano i moduli, il beccheggio resta col segno.
            coppie.append((f"{r['terreno']} {r.get('verso', '')}", abs(vera),
                           float(np.degrees(np.arctan(stima))), becc))
    if coppie:
        print("\nPendenza stimata (scansione, col corpo, 90° percentile) contro quella misurata:")
        print(f"{'terreno e verso':22s} {'vera':>6s} {'stimata':>8s} {'beccheggio medio':>17s}")
        per = {}
        for ter, vera, st, be in coppie:
            per.setdefault(ter, []).append((vera, st, be))
        for ter, v in sorted(per.items()):
            a = np.array(v)
            print(f"{ter[:22]:22s} {np.nanmean(a[:, 0]):5.1f}° {np.nanmean(a[:, 1]):7.1f}° "
                  f"{np.nanmean(a[:, 2]):16.1f}°")
        print("La stima e' un 90° percentile, quindi di solito un po' sopra la pendenza media: "
              "conta che sia coerente fra terreni. Un errore di qualche grado sposta di molto il "
              "livello scelto (0.15 = 8.5°, 0.25 = 14°).")


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--cartella', default='MissionLogs')
    p.add_argument('--missioni', nargs='*', default=None)
    p.add_argument('--dal', default=None, help="AAAA-MM-GG: solo le missioni da quel giorno")
    p.add_argument('--beccheggio', type=float, default=12.0,
                   help="gradi di beccheggio di picco oltre cui un tratto e' 'inclinato'")
    p.add_argument('--prove', default=os.path.join('ProveAndatura', 'prove.csv'))
    a = p.parse_args()

    S = carica_segmenti(a.cartella, a.missioni, a.dal)
    if S is None:
        print(f"Nessun segments.csv trovato in {a.cartella}/ con questi filtri.")
    else:
        print(f"{len(S['tier'])} tratti da {len(set(S['missione']))} missioni.")
        sezione_livelli(S)
        sezione_costo(S)
        sezione_fallimenti(S)
        sezione_soglie(S, a.beccheggio)
    sezione_prove(a.prove)


if __name__ == '__main__':
    sys.exit(main())
