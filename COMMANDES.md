# Commandes d'execution

### Commande minimale
```powershell
py Kmeans.py a127.ply
```
### Commande demandee (metriques + echantillonnage)
```powershell
py Kmeans.py a127.ply --metric --echant
```

### Echantillonnage voxel
```powershell
py Kmeans.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05
```

### Options disponibles
```powershell
py Kmeans.py --help
```
- `ply_path`: fichier `.ply` a charger (positionnel)
- `--n-planes`: nombre de plans a detecter (defaut `6`)
- `--distance-threshold`: seuil distance point-plan (defaut: `1%` de la diagonale)
- `--no-visualize`: desactive la fenetre Open3D
- `--metric`: affiche les metriques en console
- `--use-sampling` / `--echant`: active l'echantillonnage
- `--sampling-method` / `--echant-method`: methode d'echantillonnage (`random` ou `voxel`)
- `--sample-target-points`: nombre de points apres echantillonnage aleatoire (defaut `4000`)
- `--sample-seed`: graine aleatoire pour la methode `random` (defaut `42`)
- `--voxel-size`: taille de voxel pour la methode `voxel` (defaut `0.05`)
- `--export-csv`: exporte les metriques en CSV
- `--csv-path`: chemin du CSV (defaut `room_planes_metrics.csv`)

### Exemples utiles
```powershell
py Kmeans.py a127.ply --metric --echant --no-visualize
py Kmeans.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.03
py Kmeans.py a127.ply --metric --echant --export-csv
py Kmeans.py a127.ply --n-planes 8 --distance-threshold 0.02
```

### Metriques + echantillonnage
```powershell
py Linear.py a127.ply --metric --echant
```

### Echantillonnage voxel
```powershell
py Linear.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05
```
### Exemples utiles
```powershell
py Linear.py a127.ply --metric --echant --no-visualize
py Linear.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.03
py Linear.py a127.ply --max-planes 6 --inlier-percentile 10
py Linear.py a127.ply --metric --export-csv
```

### Options disponibles
```powershell
py Ransac.py --help
```
- `ply_path`: fichier `.ply` a charger (positionnel)
- `--max-planes`: nombre maximal de plans (defaut `6`)
- `--min-points`: points minimaux par plan (defaut `200`)
- `--inlier-ratio-threshold`: ratio minimal d'inliers (defaut `0.05`)
- `--residual-threshold`: seuil de distance point-plan (defaut: `1%` de la diagonale)
- `--max-trials`: iterations RANSAC max (defaut `1000`)
- `--confidence`: confiance pour estimation theorique (defaut `0.99`)
- `--no-visualize`: desactive la fenetre Open3D
- `--metric`: affiche les metriques en console
- `--use-sampling` / `--echant`: active l'echantillonnage
- `--sampling-method` / `--echant-method`: methode d'echantillonnage (`random` ou `voxel`)
- `--sample-target-points`: taille echantillon pour `random` (defaut `3000`)
- `--sample-seed`: graine aleatoire pour `random` (defaut `42`)
- `--voxel-size`: taille de voxel pour `voxel` (defaut `0.05`)
- `--export-csv`: exporte les metriques en CSV
- `--csv-path`: chemin du CSV (defaut `ransac_metrics.csv`)

### Exemples utiles
```powershell
py Ransac.py a127.ply --metric --echant --no-visualize
py Ransac.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.03
py Ransac.py a127.ply --max-trials 2000 --confidence 0.995
py Ransac.py a127.ply --metric --export-csv
```

## 4) Commandes rapides (resume)

```powershell
# KMeans: metriques + echantillonnage
py Kmeans.py a127.ply --metric --echant

# KMeans: metriques + echantillonnage voxel
py Kmeans.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05

# Linear: metriques + echantillonnage
py Linear.py a127.ply --metric --echant

# Linear: metriques + echantillonnage voxel
py Linear.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05

# RANSAC: metriques + echantillonnage
py Ransac.py a127.ply --metric --echant

# RANSAC: metriques + echantillonnage voxel
py Ransac.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05

# Check un plan manuel (a,b,c,d)
py checkplan.py a127.ply 1,0,5,2

# Check un plan manuel + export CSV
py checkplan.py a127.ply 1,0,5,2 --distance-threshold 0.03 --export-csv

# Check avec lissage des petits coefficients
py checkplan.py a127.ply 0.00005,0,0.99999,0.9258 --smooth-epsilon 0.0001
```

## 5) Check d'un plan manuel

Le format du plan est: `a,b,c,d` dans l'equation `ax+by+cz+d=0`.

Exemple:

```powershell
py checkplan.py a127.ply 1,0,5,2
```

Options:
- `--distance-threshold`: seuil d'appartenance d'un point au plan
- visualisation 3D: toujours active
- `--smooth-epsilon`: lisse les petits coefficients (ex: `0.00005 -> 0`)
- `--export-csv`: ajoute les metriques dans `checkplan_metrics.csv`
