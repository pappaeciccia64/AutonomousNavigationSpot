"""
prova_grafo.py -- guardare il grafo di missione con i parametri che si vogliono, senza robot.

Si cambiano i numeri nel blocco PARAMETRI qui sotto e si lancia, dalla cartella del progetto:

    python3 prova_grafo.py

Fa un'immagine con i metodi scelti affiancati (lo stesso pezzo di area, gli stessi archi e lo
stesso seme), una tabella con nodi, archi, grado medio e copertura, e la apre. Nel terminale
stampa il riepilogo [GRAFO] di ogni grafo, come nel log di missione.

Questi parametri valgono SOLO per la prova: la missione usa i suoi (mission_graph.py,
oppure gli argomenti di easy_walk.py). Per ripetere sul robot un grafo visto qui, la riga
[GRAFO] Nodi casuali stampa il comando da dare a easy_walk.py.
"""
import os
import subprocess

import mission_graph

# ======================================================================================
# PARAMETRI: cambia questi
# ======================================================================================
# Area della missione: righe (a sinistra del robot) x colonne (davanti) x lato della cella.
RIGHE = 10
COLONNE = 10
LATO_CELLA_M = 2.0

# Archi: si collegano tutte le coppie di nodi distanti fra ARCO_MIN_M e ARCO_MAX_M.
# In missione: 0.5 e 1.25.
ARCO_MIN_M = 0.5
ARCO_MAX_M = 1.25

# Reticolo: passo fra i nodi, arrotondato a LATO_CELLA_M / n (in missione 0.5).
# Passo 0.5 = 4 nodi/m2, come DENSITA_NODI_M2 qui sotto: a parita' di nodi.
PASSO_RETICOLO_M = 0.5

# Nodi casuali: nodi per m2 (in missione 4) e seme (None = estratto a caso, scritto
# nel titolo dell'immagine e nel terminale).
# 4 nodi/m2 = la densita' del reticolo a passo 0.5: numero di nodi simile (il reticolo ne
# ha qualcuno in piu', la fila sul bordo esterno: ~6% con 3 x 3 celle da 4 m).
# DENSITA_NODI_M2 = None: esattamente lo stesso numero di nodi del reticolo.
DENSITA_NODI_M2 = 5.0
SEME = 7

# Quali metodi mettere a confronto: togli quelli che non vuoi vedere.
METODI = ['reticolo', 'uniforme', 'poisson', 'perturbata', 'halton']

# Riquadro da disegnare: None = tutta l'area; oppure (x_min, x_max, y_min, y_max) in metri,
# con il robot in (0, 0), x in avanti e y a sinistra. Esempio: (-2, 2, -2, 2).
ZOOM = None

# Immagine da salvare, e se aprirla alla fine.
IMMAGINE = 'grafi.png'
APRI = True
# ======================================================================================


def main():
    path, graphs = mission_graph.compare_methods_png(
        IMMAGINE, zoom=ZOOM, seed=SEME, methods=METODI, verbose=True,
        rows=RIGHE, cols=COLONNE, cell_size_m=LATO_CELLA_M,
        spacing_m=PASSO_RETICOLO_M, rings=None,
        min_edge_length=ARCO_MIN_M, max_edge_length=ARCO_MAX_M,
        density_per_m2=DENSITA_NODI_M2)
    # Nodi per cella, con i nodi sui confini contati a meta' (vedi nodes_per_cell).
    print("\nNodi per cella (righe dall'alto, come nell'immagine; i nodi sui confini fra "
          "celle contano a meta'):")
    for g in graphs:
        name = g.distribution if g.is_random else 'reticolo'
        counts = mission_graph.nodes_per_cell(g)
        print(f"  {name:11s} totale {g.n_nodes:5d}, per cella {counts.min():.0f}-{counts.max():.0f}")
        for r in reversed(range(counts.shape[0])):
            print("              " + " ".join(f"{v:6.1f}" for v in counts[r]))
    print(f"\nImmagine salvata in {os.path.abspath(path)}")
    if APRI:
        try:
            subprocess.Popen(['xdg-open', path], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        except OSError:
            print("Non riesco ad aprirla da qui: aprila a mano.")


if __name__ == '__main__':
    main()
