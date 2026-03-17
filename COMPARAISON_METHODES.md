# Comparaison des methodes de detection de plans

Ce document explique ce que font les 3 scripts du projet et dans quels cas ils sont les plus adaptes.

## 1) Resume rapide

- `Kmeans.py`:
  - calcule les normales locales des points
  - fait un KMeans sur ces normales
  - ajuste ensuite un plan sur chaque groupe
  - logique: orientation d'abord, plan ensuite

- `Ransac.py`:
  - ajuste des plans directement avec une logique RANSAC
  - retire iterativement les inliers des plans trouves
  - logique: detection directe de plans

- `Linear.py`:
  - ajuste un plan par regression lineaire
  - repete ensuite la detection sur les points restants
  - logique: approximation simple et rapide, moins robuste que RANSAC sur des cas difficiles

## 2) Ce que fait vraiment KMeans dans ce projet

Le script `Kmeans.py` ne fait pas un KMeans directement sur les coordonnees 3D pour detecter les plans.

Il fait plutot ceci:

1. estimation des normales de surface pour chaque point
2. clustering KMeans sur les normales
3. ajustement d'un plan sur chaque cluster obtenu

Donc:
- KMeans sert ici a regrouper les points qui ont une orientation similaire
- la detection du plan se fait ensuite
- ce n'est pas un clustering "direct de plans"

## 3) KMeans sur normales vs KMeans sur points

### KMeans sur normales

Avantages:
- tres utile si plusieurs surfaces ont des orientations bien distinctes
- marche bien dans une scene de type piece: murs, sol, plafond
- permet de retrouver rapidement les grandes directions dominantes

Limites:
- deux plans paralleles peuvent avoir presque la meme normale
- dans ce cas, KMeans peut les melanger au depart
- depend de la qualite de l'estimation des normales
- sensible au bruit local si les normales sont instables

### KMeans direct sur les points 3D

Avantages:
- simple a executer
- utile pour separer des groupes spatiaux compacts

Limites:
- pas adapte a la detection de plans
- KMeans regroupe par proximite geometrique, pas par structure plane
- il peut produire des paquets de points sans vrai sens geometrique pour les plans

Conclusion:
- si le but est de detecter des plans, faire KMeans directement sur `x,y,z` n'est en general pas la bonne approche
- faire KMeans sur les normales est beaucoup plus pertinent

## 4) Pourquoi RANSAC est souvent meilleur pour les plans

RANSAC cherche directement un modele de plan qui explique un grand nombre de points.

Avantages:
- tres adapte a la detection de plans
- robuste aux outliers
- plus naturel pour extraire murs, plafond, sol
- donne souvent des plans plus credibles geometriquement

Limites:
- depend du seuil de distance point-plan
- peut etre plus lent si le nuage est enorme
- l'ordre d'extraction peut influencer les plans suivants

Conclusion:
- si ton objectif principal est "trouver les vrais plans de la scene", RANSAC est souvent le meilleur choix

## 5) Place de la regression lineaire

`Linear.py` ajuste un plan avec une regression lineaire classique.

Avantages:
- simple
- rapide
- facile a comprendre

Limites:
- moins robuste aux outliers que RANSAC
- moins fiable si la scene contient beaucoup de bruit ou des surfaces melangees
- bon pour une approximation, moins bon pour une segmentation plane robuste

Conclusion:
- utile comme baseline ou methode simple
- moins solide que RANSAC si la scene est complexe

## 6) Quelle methode choisir ?

### Si tu veux detecter les vrais plans d'une piece
Choix recommande:
- `Ransac.py`

Pourquoi:
- detection directe de plans
- robuste
- plus coherent geometriquement

### Si tu veux separer rapidement les grandes orientations
Choix recommande:
- `Kmeans.py`

Pourquoi:
- les normales capturent bien les orientations dominantes
- interessant pour une scene structuree type interieur

### Si tu veux une methode simple de reference
Choix recommande:
- `Linear.py`

Pourquoi:
- implementation simple
- bon point de comparaison
- utile pour une baseline

## 7) Recommandation pratique pour ton projet

Pour un nuage de points de piece/interieur:

1. utilise `Ransac.py` si tu veux la meilleure detection de plans
2. utilise `Kmeans.py` si tu veux comparer une strategie basee sur les orientations
3. garde `Linear.py` comme baseline simple

En pratique:
- `Ransac.py` = meilleur candidat principal
- `Kmeans.py` = bon candidat secondaire pour comparaison
- `Linear.py` = reference simple mais moins robuste

## 8) Lecture simple

En une phrase:
- `Kmeans.py` regroupe d'abord les orientations, puis ajuste les plans
- `Ransac.py` cherche directement les plans
- `Linear.py` ajuste des plans plus simplement, mais avec moins de robustesse
