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

### Check Plan (plan manuel)

Tu peux tester un plan precis avec ses coefficients d'equation $ax+by+cz+d=0$:

```powershell
py checkplan.py a127.ply 1,0,5,2
```

Le script affiche toujours la visualisation 3D Open3D.
Il applique aussi un lissage des coefficients pour faciliter le test de "plan parfait"
(ex: `0.00005` devient `0` avec le seuil par defaut).

Options utiles:

```powershell
py checkplan.py a127.ply 1,0,5,2 --distance-threshold 0.03 --export-csv
py checkplan.py a127.ply 0.00005,0,0.99999,0.9258 --smooth-epsilon 0.0001
py checkplan.py a127.ply --planes-file plans_a127.txt --distance-threshold 0.03 --smooth-epsilon 0.0001
```

Fichiers utiles fournis:
- `CHECKPLAN_COMMANDES.txt`: commandes pretes a copier/coller
- `plans_a127.txt`: exemple de liste multi-plans (un plan par ligne)

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
