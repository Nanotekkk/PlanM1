# Commandes d'execution

## Structure du projet

```
ply_files/   — fichiers .ply (nuages de points)
json/        — fichiers de plans JSON (testa127.json, test_fmms_salle_152.json, ...)
csv/         — fichiers de metriques CSV (generes par --export-csv)
plans/       — fichiers de plans .txt (plans_a127.txt, CHECKPLAN_COMMANDES.txt)
utility/     — modules Python partages
```

---

## 1) KMeans

```powershell
# Minimal
py Kmeans.py a127.ply

# Avec metriques et echantillonnage
py Kmeans.py a127.ply --metric --echant

# Echantillonnage voxel
py Kmeans.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05

# Export CSV
py Kmeans.py a127.ply --metric --echant --export-csv
```

**Options:**
| Option | Defaut | Description |
|--------|--------|-------------|
| `--n-planes` | `6` | Nombre de plans a detecter |
| `--distance-threshold` | `1% diag` | Seuil distance point-plan |
| `--no-visualize` | — | Desactive la fenetre Open3D |
| `--metric` | — | Affiche les metriques en console |
| `--echant` | — | Active l'echantillonnage |
| `--sampling-method` | `random` | Methode: `random` ou `voxel` |
| `--sample-target-points` | `4000` | Taille echantillon (random) |
| `--sample-seed` | `42` | Graine aleatoire |
| `--voxel-size` | `0.05` | Taille de voxel |
| `--export-csv` | — | Exporte les metriques en CSV |
| `--csv-path` | `csv/room_planes_metrics.csv` | Chemin du CSV |
| `--world-model` | `none` | Modele de monde : `manhattan` ou `atlanta` |
| `--world-angle-tol` | `15.0` | Tolerance angulaire (degres) pour recaler une normale |
| `--atlanta-n-horizontal` | `4` | Directions horizontales de murs (Atlanta World) |

---

## 2) Linear

```powershell
# Minimal
py Linear.py a127.ply

# Avec metriques et echantillonnage
py Linear.py a127.ply --metric --echant

# Echantillonnage voxel
py Linear.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05

# Export CSV
py Linear.py a127.ply --metric --export-csv
```

**Options:**
| Option | Defaut | Description |
|--------|--------|-------------|
| `--max-planes` | `6` | Nombre maximal de plans |
| `--inlier-percentile` | `15.0` | Percentile de residus pour les inliers |
| `--min-points` | `200` | Points minimaux par plan |
| `--no-visualize` | — | Desactive la fenetre Open3D |
| `--metric` | — | Affiche les metriques en console |
| `--echant` | — | Active l'echantillonnage |
| `--sampling-method` | `random` | Methode: `random` ou `voxel` |
| `--sample-target-points` | `3000` | Taille echantillon (random) |
| `--sample-seed` | `42` | Graine aleatoire |
| `--voxel-size` | `0.05` | Taille de voxel |
| `--export-csv` | — | Exporte les metriques en CSV |
| `--csv-path` | `csv/linear_metrics.csv` | Chemin du CSV |
| `--world-model` | `none` | Modele de monde : `manhattan` ou `atlanta` |
| `--world-angle-tol` | `15.0` | Tolerance angulaire (degres) pour recaler une normale |
| `--atlanta-n-horizontal` | `4` | Directions horizontales de murs (Atlanta World) |

---

## 3) RANSAC

```powershell
# Minimal
py Ransac.py a127.ply

# Avec metriques et echantillonnage
py Ransac.py a127.ply --metric --echant

# Echantillonnage voxel
py Ransac.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05

# Plus d'iterations
py Ransac.py a127.ply --max-trials 2000 --confidence 0.995

# Export CSV
py Ransac.py a127.ply --metric --export-csv
```

**Options:**
| Option | Defaut | Description |
|--------|--------|-------------|
| `--max-planes` | `6` | Nombre maximal de plans |
| `--min-points` | `200` | Points minimaux par plan |
| `--inlier-ratio-threshold` | `0.05` | Ratio minimal d'inliers |
| `--residual-threshold` | `1% diag` | Seuil de distance point-plan |
| `--max-trials` | `1000` | Iterations RANSAC max |
| `--confidence` | `0.99` | Confiance pour estimation theorique |
| `--no-visualize` | — | Desactive la fenetre Open3D |
| `--metric` | — | Affiche les metriques en console |
| `--echant` | — | Active l'echantillonnage |
| `--sampling-method` | `random` | Methode: `random` ou `voxel` |
| `--sample-target-points` | `3000` | Taille echantillon (random) |
| `--sample-seed` | `42` | Graine aleatoire |
| `--voxel-size` | `0.05` | Taille de voxel |
| `--export-csv` | — | Exporte les metriques en CSV |
| `--csv-path` | `csv/ransac_metrics.csv` | Chemin du CSV |
| `--world-model` | `none` | Modele de monde : `manhattan` ou `atlanta` |
| `--world-angle-tol` | `15.0` | Tolerance angulaire (degres) pour recaler une normale |
| `--atlanta-n-horizontal` | `4` | Directions horizontales de murs (Atlanta World) |

---

## 4) checkplan

Verifie un ou plusieurs plans connus sur un nuage de points.
Les coefficients suivent l'equation `ax + by + cz + d = 0`.
Les valeurs sont arrondies automatiquement a l'affichage (ex: `0.9999 -> 1`).

```powershell
# Plan manuel
py checkplan.py a127.ply 0,0,1,0.93

# Plusieurs plans depuis un fichier JSON
py checkplan.py a127.ply testa127.json

# JSON seul (demande le .ply interactivement)
py checkplan.py testa127.json

# Visualisation 3D — tous les plans ensemble
py checkplan.py a127.ply testa127.json --visualize

# Visualisation plan par plan (fermer la fenetre pour passer au suivant)
py checkplan.py a127.ply testa127.json --visualize-one-by-one

# Scanner une normale pour trouver les valeurs d candidates
py checkplan.py a127.ply --scan-normal 0,0,1
py checkplan.py a127.ply "--scan-normal=-0.38,0.92,0"

# Export CSV des metriques
py checkplan.py a127.ply testa127.json --export-csv
```

**Options:**
| Option | Defaut | Description |
|--------|--------|-------------|
| `--distance-threshold` | `1% diag` | Seuil d'appartenance d'un point au plan |
| `--smooth-epsilon` | `1e-4` | Lissage des coefficients (ex: `0.00005 -> 0`) |
| `--no-metric` | — | Desactive l'affichage des metriques |
| `--visualize` | — | Fenetre Open3D, tous les plans |
| `--visualize-one-by-one` | — | Fenetre Open3D, un plan a la fois |
| `--planes-file` | — | Fichier .txt de plans (un par ligne) |
| `--scan-normal` | — | Trouve les distances `d` candidates pour une normale |
| `--export-csv` | — | Exporte les metriques en CSV |
| `--csv-path` | `csv/checkplan_metrics.csv` | Chemin du CSV |

**Fichiers JSON disponibles** (recherche automatique dans `json/`) :
- `json/testa127.json` — plans de `a127.ply`
- `json/test_fmms_salle_152.json` — plans de `fmms_salle_152.ply`

**Format JSON:**
```json
{
  "planes": [
    [a, b, c, d],
    [a, b, c, d]
  ]
}
```
