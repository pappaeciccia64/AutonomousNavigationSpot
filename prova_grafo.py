"""
prova_grafo.py -- guardare i grafi di missione possibili, senza robot, e sceglierne uno.

    python3 prova_grafo.py

Di default usa GLI STESSI parametri della missione (parametri.py): area, archi, passo del
reticolo, densita' e seme dei nodi casuali. Fa un'immagine con i metodi affiancati (stessa
area, stessi archi, stesso seme), una tabella con nodi, archi, grado medio e copertura, e la
apre. Nel terminale stampa il riepilogo [GRAFO] di ogni grafo, come nel log di missione, i
nodi per cella e, in fondo, COME USARE IN MISSIONE ciascuno dei grafi visti.

Per provare valori diversi senza toccare parametri.py, si cambia il blocco PROVA qui sotto
(None = il valore di parametri.py). Valgono solo per l'immagine.

Il grafo e' riferito alla posa del robot all'accensione: stessi parametri e stesso seme danno
lo stesso grafo, nella stessa posizione rispetto al punto di partenza.
"""
import os
import subprocess

import mission_graph
import parametri as P

# ======================================================================================
# PROVA: None = come in parametri.py
# ======================================================================================
# Area: righe (a sinistra del robot) x colonne (davanti) x lato della cella.
RIGHE = None                 # parametri.MISSION_GRID_ROWS
COLONNE = None               # parametri.MISSION_GRID_COLS
LATO_CELLA_M = None          # parametri.MISSION_CELL_SIZE_M

# Archi: tutte le coppie di nodi distanti fra ARCO_MIN_M e ARCO_MAX_M.
ARCO_MIN_M = None            # parametri.PRM_MIN_EDGE_M
ARCO_MAX_M = None            # parametri.PRM_MAX_EDGE_M

# Reticolo: passo fra i nodi, arrotondato a LATO_CELLA_M / n; anelli di vicini collegati.
PASSO_RETICOLO_M = None      # parametri.MISSION_LATTICE_SPACING_M
ANELLI = None                # parametri.MISSION_LATTICE_RINGS (None anche li' = tutti)

# Nodi casuali: nodi per m2 ('come_reticolo' = stesso numero di nodi del reticolo) e seme.
# Seme None (anche in parametri.py) = estratto a caso: e' scritto nel titolo dell'immagine e
# in fondo, per rifare in missione lo stesso grafo.
DENSITA_NODI_M2 = None       # parametri.MISSION_RANDOM_DENSITY
SEME = None                  # parametri.MISSION_RANDOM_SEED

# Quali metodi mettere a confronto: togli quelli che non vuoi vedere.
METODI = ['reticolo', 'uniforme', 'poisson', 'perturbata', 'halton']

# Riquadro da disegnare: None = tutta l'area; oppure (x_min, x_max, y_min, y_max) in metri,
# con il robot in (0, 0), x in avanti e y a sinistra. Esempio: (-2, 2, -2, 2).
ZOOM = None

# Immagine da salvare, e se aprirla alla fine.
IMMAGINE = 'grafi.png'
APRI = True
# ======================================================================================


def _scelta(valore, missione):
    return missione if valore is None else valore


def main():
    rows = _scelta(RIGHE, P.MISSION_GRID_ROWS)
    cols = _scelta(COLONNE, P.MISSION_GRID_COLS)
    cell = _scelta(LATO_CELLA_M, P.MISSION_CELL_SIZE_M)
    e_min = _scelta(ARCO_MIN_M, P.PRM_MIN_EDGE_M)
    e_max = _scelta(ARCO_MAX_M, P.PRM_MAX_EDGE_M)
    spacing = _scelta(PASSO_RETICOLO_M, P.MISSION_LATTICE_SPACING_M)
    rings = _scelta(ANELLI, P.MISSION_LATTICE_RINGS)
    density = _scelta(DENSITA_NODI_M2, P.MISSION_RANDOM_DENSITY)
    seed = _scelta(SEME, P.MISSION_RANDOM_SEED)
    if seed is None:
        seed = mission_graph.draw_seed()
    print(f"Area {rows} x {cols} celle da {cell:g} m, archi {e_min:g}-{e_max:g} m, "
          f"passo del reticolo {spacing}, anelli {rings if rings is not None else 'tutti'}, "
          f"densita' {density}, seme {seed}"
          + ("" if all(v is None for v in (RIGHE, COLONNE, LATO_CELLA_M, ARCO_MIN_M, ARCO_MAX_M,
                                           PASSO_RETICOLO_M, ANELLI, DENSITA_NODI_M2))
             else "  (alcuni valori dal blocco PROVA, non da parametri.py)"))

    path, graphs = mission_graph.compare_methods_png(
        IMMAGINE, zoom=ZOOM, seed=seed, methods=METODI, verbose=True,
        rows=rows, cols=cols, cell_size_m=cell,
        spacing_m=spacing, rings=rings,
        min_edge_length=e_min, max_edge_length=e_max,
        density_per_m2=None if density == 'come_reticolo' else density)
    # Nodi per cella, con i nodi sui confini contati a meta' (vedi nodes_per_cell).
    print("\nNodi per cella (righe dall'alto, come nell'immagine; i nodi sui confini fra "
          "celle contano a meta'):")
    for g in graphs:
        name = g.distribution if g.is_random else 'reticolo'
        counts = mission_graph.nodes_per_cell(g)
        print(f"  {name:11s} totale {g.n_nodes:5d}, per cella {counts.min():.0f}-{counts.max():.0f}")
        for r in reversed(range(counts.shape[0])):
            print("              " + " ".join(f"{v:6.1f}" for v in counts[r]))

    # Come usare in missione ciascun grafo
    area = (rows, cols, cell) != (P.MISSION_GRID_ROWS, P.MISSION_GRID_COLS, P.MISSION_CELL_SIZE_M)
    edges = (e_min, e_max) != (P.PRM_MIN_EDGE_M, P.PRM_MAX_EDGE_M)
    print("\nPER USARE IN MISSIONE UNO DI QUESTI GRAFI")
    if area or edges:
        print("  (area o archi diversi da parametri.py: vanno cambiati anche quelli)")
    for g in graphs:
        if g.is_random:
            d = g.describe()
            dens = d.get('densita_nodi_m2', density)
            righe = (f"MISSION_GRAPH_SAMPLING = 'casuale'   MISSION_RANDOM_DISTRIBUTION = "
                     f"'{g.distribution}'   MISSION_RANDOM_DENSITY = {dens:g}   "
                     f"MISSION_RANDOM_SEED = {seed}")
            cmd = (f"--grafo casuale --distribuzione {g.distribution} --densita {dens:g} "
                   f"--seme {seed}")
            name = g.distribution
        else:
            righe = (f"MISSION_GRAPH_SAMPLING = 'reticolo'   MISSION_LATTICE_SPACING_M = "
                     f"{spacing}   MISSION_LATTICE_RINGS = {rings}")
            cmd = f"--grafo reticolo --spacing {spacing}" + (f" --rings {rings}" if rings else "")
            name = 'reticolo'
        extra = ""
        if area:
            extra += f" --rows {rows} --cols {cols} --cell {cell:g}"
        if edges:
            extra += f" --min-edge {e_min:g} --max-edge {e_max:g}"
        print(f"  {name}:")
        print(f"    in parametri.py:  {righe}")
        print(f"    oppure una volta: python3 easy_walk.py {cmd}{extra}")
    if P.MISSION_RANDOM_SEED is None:
        print("  Attenzione: in parametri.py il seme e' None, cioe' nuovo a ogni missione. Per "
              "avere in missione un grafo casuale visto qui serve il suo seme.")

    print(f"\nImmagine salvata in {os.path.abspath(path)}")
    if APRI:
        try:
            subprocess.Popen(['xdg-open', path], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        except OSError:
            print("Non riesco ad aprirla da qui: aprila a mano.")


if __name__ == '__main__':
    main()
