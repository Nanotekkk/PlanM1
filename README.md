# PlanM1

Detection et verification de plans dans des nuages de points `.ply`.

Trois methodes de detection automatique, plus un outil de verification manuelle.

---

## Installation

```powershell
pip install -r requirements.txt
```

Pre-requis : Python 3.10+

---

## Structure

```
Kmeans.py                  — detection par clustering des normales
Linear.py                  — detection par regression lineaire iterative
Ransac.py                  — detection par RANSAC
checkplan.py               — verification de plans connus
utility/                   — modules partages (I/O, metriques, visualisation)
ply_files/                 — nuages de points .ply
json/                      — plans de reference .json
csv/                       — metriques exportees .csv
plans/                     — plans .txt et commandes checkplan
requirements.txt
COMMANDES.md               — reference complete des commandes
```

---

## Utilisation rapide

Les fichiers `.ply` sont charges depuis `ply_files/` si tu donnes seulement le nom.

### KMeans
```powershell
py Kmeans.py a127.ply --metric --echant
```

### Linear
```powershell
py Linear.py a127.ply --metric --echant
```

### RANSAC
```powershell
py Ransac.py a127.ply --metric --echant
```

### checkplan — verification de plans connus

Teste un ou plusieurs plans definis par leurs coefficients `ax + by + cz + d = 0`.
Les valeurs sont arrondies automatiquement a l'affichage.

```powershell
# Plan manuel
py checkplan.py a127.ply 0,0,1,0.93

# Depuis un fichier JSON
py checkplan.py a127.ply testa127.json

# Avec visualisation plan par plan
py checkplan.py a127.ply testa127.json --visualize-one-by-one

# Scanner une normale pour trouver les distances d candidates
py checkplan.py a127.ply --scan-normal 0,0,1
```

Fichiers JSON disponibles dans `json/` :
- `testa127.json` — plans de `a127.ply`
- `test_fmms_salle_152.json` — plans de `fmms_salle_152.ply`

---

## Metriques

Chaque plan detecte retourne :
- `inlier_count` : nombre de points appartenant au plan
- `inlier_ratio` : proportion du nuage couverte
- `rmse` : erreur quadratique moyenne des inliers

---

## Echantillonnage

Activation avec `--echant`. Deux modes :

```powershell
# Aleatoire
--echant --sampling-method random --sample-target-points 3000

# Voxel (un point par cellule de grille)
--echant --sampling-method voxel --voxel-size 0.05
```

---

## Export CSV

Ajouter `--export-csv` a n'importe quelle commande.
Les fichiers sont ecrits dans `csv/` par defaut.

```powershell
py Ransac.py a127.ply --metric --export-csv
py checkplan.py a127.ply testa127.json --export-csv
```

---

Voir `COMMANDES.md` pour la liste complete des options.
