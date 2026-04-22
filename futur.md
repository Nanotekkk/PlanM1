# Amélioration RANSAC — éviter les plans parallèles inutiles

## Problème

- RANSAC itératif retire les inliers à chaque plan trouvé puis recommence
- Il détecte bien sol + plafond mais gaspille des slots sur des plans parasites (tables, rebords) qui ont la même normale
- Résultat : moins de vrais murs détectés

---

## Pistes

### 1. Filtrage post-détection par normale ✅ le plus simple
- Regrouper les plans dont les normales sont quasi-parallèles (`|n1·n2| > 0.95`)
- Dans chaque groupe, garder seulement les 2 avec le plus d'inliers
- Aucun changement dans la boucle RANSAC
- Limite : si 2 murs parallèles + 1 table dans la même direction, il faut choisir combien garder par direction

### 2. Rejet pendant la boucle ✅ complémentaire
- Avant d'accepter un plan candidat, vérifier s'il est parallèle à un plan déjà accepté ET s'il a beaucoup moins d'inliers (ex: < 50%)
- Si oui → rejeter et continuer RANSAC
- Limite : si la table est trouvée avant le plafond, elle serait conservée

### 3. Hypothèse Manhattan World (recherche avancée)
- Dans une salle rectangulaire : seulement 3 directions de normales orthogonales
- Estimer les 3 axes dominants par PCA ou KMeans sur les normales de surface
- Contraindre RANSAC à ne valider que les plans dont la normale colle à un axe dominant
- Élimine les plans obliques en plus des doublons
- Ref : Coughlan & Yuille (1999), *Manhattan-world stereo*

### 4. Fusion de plans trop proches
- Deux plans parallèles avec `|d1 - d2|` faible (< 20 cm) = deux faces d'un même objet mince
- Fusionner ou garder seulement le plus grand
- Utile pour les cloisons fines ou les tables épaisses

---

## Priorité d'implémentation

| # | Approche | Difficulté | À faire |
|---|----------|-----------|---------|
| 1 | Filtrage post par normale | Faible | En premier |
| 2 | Rejet dans la boucle | Faible | Avec 1 |
| 4 | Fusion plans proches | Faible | Avec 1+2 |
| 3 | Manhattan World | Élevée | Recherche |

→ Implémenter 1+2+4 dans une fonction `filter_redundant_planes()` appelée après la boucle dans `Ransac.py`

---

## Refs
- Fischler & Bolles (1981) — RANSAC
- Coughlan & Yuille (1999) — Manhattan World
- Monszpart et al. (2015) — *RAPter* (contraintes parallélisme/perpendicularité)
- Open3D `segment_plane()` — `ransac_n`, `num_iterations`, `distance_threshold`
