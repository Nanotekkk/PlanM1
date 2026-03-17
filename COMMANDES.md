# Commandes CLI du projet

Ce document liste les commandes disponibles pour lancer les 3 scripts principaux:
- `Kmeans.py`
- `Linear.py`
- `Ransac.py`

Les exemples utilisent `py` sous Windows.

## 1) KMeans (`Kmeans.py`)

### Commande minimale
```powershell
py Kmeans.py a127.ply
```
Ce que ca fait:
- Charge `a127.ply`
- Detecte des plans avec la methode normales + KMeans
- Ouvre la visualisation Open3D

### Commande demandee (metriques + echantillonnage)
```powershell
py Kmeans.py a127.ply --metric --echant
```
Ce que ca fait:
- Active l'echantillonnage
- Calcule et affiche les metriques (inlier_ratio, rmse, score)
- Affiche aussi les parametres de chaque plan detecte (normale, distance, nombre d'inliers)
- Lance la detection de plans

### Echantillonnage voxel
```powershell
py Kmeans.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05
```
Ce que ca fait:
- Active l'echantillonnage par grille de voxels
- Garde un point representatif par voxel
- Reduit fortement le nombre de points a traiter

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

## 2) Linear (`Linear.py`)

### Commande minimale
```powershell
py Linear.py a127.ply
```
Ce que ca fait:
- Charge `a127.ply`
- Detecte des plans avec regression lineaire iterative
- Ouvre la visualisation Open3D

### Metriques + echantillonnage
```powershell
py Linear.py a127.ply --metric --echant
```
Ce que ca fait:
- Active l'echantillonnage
- Affiche les metriques du plan principal
- Affiche aussi les parametres des plans detectes

### Echantillonnage voxel
```powershell
py Linear.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05
```
Ce que ca fait:
- Active l'echantillonnage voxel
- Selectionne un point representatif par voxel

### Options disponibles
```powershell
py Linear.py --help
```
- `ply_path`: fichier `.ply` a charger (positionnel)
- `--max-planes`: nombre maximal de plans (defaut `6`)
- `--inlier-percentile`: percentile des residus pour les inliers (defaut `15.0`)
- `--min-points`: nombre minimal de points par plan (defaut `200`)
- `--no-visualize`: desactive la fenetre Open3D
- `--metric`: affiche les metriques en console
- `--use-sampling` / `--echant`: active l'echantillonnage
- `--sampling-method` / `--echant-method`: methode d'echantillonnage (`random` ou `voxel`)
- `--sample-target-points`: taille echantillon pour `random` (defaut `3000`)
- `--sample-seed`: graine aleatoire pour `random` (defaut `42`)
- `--voxel-size`: taille de voxel pour `voxel` (defaut `0.05`)
- `--export-csv`: exporte les metriques en CSV
- `--csv-path`: chemin du CSV (defaut `linear_metrics.csv`)

### Exemples utiles
```powershell
py Linear.py a127.ply --metric --echant --no-visualize
py Linear.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.03
py Linear.py a127.ply --max-planes 6 --inlier-percentile 10
py Linear.py a127.ply --metric --export-csv
```

## 3) RANSAC (`Ransac.py`)

### Commande minimale
```powershell
py Ransac.py a127.ply
```
Ce que ca fait:
- Charge `a127.ply`
- Lance la detection de plans RANSAC
- Ouvre la visualisation Open3D

### Metriques + echantillonnage
```powershell
py Ransac.py a127.ply --metric --echant
```
Ce que ca fait:
- Active l'echantillonnage
- Affiche les metriques (inlier_ratio, rmse, score)
- Affiche aussi `estimated_trials`
- Affiche aussi les parametres de chaque plan detecte

### Echantillonnage voxel
```powershell
py Ransac.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05
```
Ce que ca fait:
- Active l'echantillonnage voxel
- Reduit les points selon une grille reguliere
- Affiche les metriques sur le resultat obtenu

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
```
