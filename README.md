# PlanM1

Projet de detection de plans dans un nuage de points `.ply` avec trois approches:
- `Kmeans.py`: clustering des normales puis ajustement de plans
- `Ransac.py`: detection directe de plans par RANSAC
- `Linear.py`: ajustement de plans par regression lineaire

## Installation

Prerequis:
- Python 3.10+
- pip

Installation des dependances:

```powershell
pip install -r requirements.txt
```

## Structure du projet

```text
Kmeans.py
Linear.py
Ransac.py
utility/
ply_files/
COMMANDES.md
COMPARAISON_METHODES.md
```

## Utilisation rapide

Les fichiers `.ply` sont charges depuis `ply_files/` si tu donnes seulement le nom du fichier.

### KMeans

```powershell
py Kmeans.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05
```

### Linear

```powershell
py Linear.py a127.ply --metric --no-visualize
```

### RANSAC

```powershell
py Ransac.py a127.ply --metric --echant --sampling-method voxel --voxel-size 0.05 --inlier-ratio-threshold 0.05
```

## Sorties

Selon les options, les scripts peuvent:
- afficher les metriques en console
- afficher les parametres des plans detectes:
  - normale
  - distance
  - nombre d'inliers
- exporter les resultats en CSV
- ouvrir une visualisation Open3D

## Echantillonnage

Deux modes sont disponibles:
- `random`: echantillonnage aleatoire
- `voxel`: un point representatif par voxel

Activation:

```powershell
--echant --sampling-method random
```

ou

```powershell
--echant --sampling-method voxel --voxel-size 0.05
```

## Documentation complementaire

- Voir `COMMANDES.md` pour la liste complete des commandes
- Voir `COMPARAISON_METHODES.md` pour la comparaison entre KMeans, RANSAC et Linear

## Git

Le dossier `ply_files/` est ignore par Git via `.gitignore`.
