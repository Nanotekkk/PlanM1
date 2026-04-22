# Pistes d'amélioration — RANSAC

## Problème constaté

RANSAC itératif (plane stripping) retire les inliers après chaque plan trouvé et recommence.
Il détecte correctement le sol et le plafond, mais s'arrête parfois sur des plans parasites **parallèles à ceux déjà trouvés** : tables, bureaux, rebords de fenêtres, etc.

Ces plans ont une normale quasi-identique à un plan déjà trouvé mais sont bien moins significatifs (moins d'inliers, plus petite surface).

---

## Approche 1 — Filtrage post-détection par direction de normale

**Idée** : après avoir trouvé tous les plans, regrouper ceux dont les normales sont quasi-parallèles et ne garder que les N plus significatifs par direction.

```python
def are_parallel(n1, n2, threshold=0.05):
    """True si les normales sont quasi-parallèles (dot product proche de ±1)."""
    dot = abs(np.dot(n1, n2))
    return dot > (1.0 - threshold)
```

**Règle de sélection dans un groupe** :
- Garder les 2 plans avec le plus d'inliers (= les deux faces opposées d'un mur, ou sol + plafond)
- Écarter tous les autres (tables, etc.)

**Avantage** : simple, aucun changement dans la boucle RANSAC.
**Limite** : si une salle a deux murs strictement parallèles ET une table, il faut calibrer combien de plans par direction on autorise (2 pour les murs, 2 pour horizontal).

---

## Approche 2 — Contrainte pendant la boucle RANSAC

**Idée** : avant d'accepter un nouveau plan trouvé, vérifier s'il est quasi-parallèle à un plan déjà accepté **ET** s'il a moins d'inliers que lui. Si oui, le rejeter (ou le passer).

```python
def is_redundant(new_plane, accepted_planes, normal_threshold=0.05, ratio_threshold=0.5):
    """
    Retourne True si new_plane est parallèle à un plan accepté
    et possède moins de ratio_threshold * inliers de ce plan.
    """
    n_new = new_plane.normal
    for p in accepted_planes:
        if are_parallel(n_new, p.normal, normal_threshold):
            if new_plane.inlier_count < ratio_threshold * p.inlier_count:
                return True
    return False
```

Dans la boucle :
```python
if not is_redundant(candidate, accepted_planes):
    accepted_planes.append(candidate)
```

**Avantage** : évite de gaspiller une "slot" de plan sur une table.
**Limite** : si la table est trouvée *avant* le plafond (moins probable mais possible), elle serait gardée.

---

## Approche 3 — Filtrage par surface estimée (convex hull / bounding box)

**Idée** : un plan de mur ou sol couvre une grande surface ; une table, non.
Calculer l'étendue 2D des inliers projetés sur le plan et rejeter les plans dont la surface est trop petite.

```python
from scipy.spatial import ConvexHull

def estimate_plane_area(points, plane_normal):
    """Projette les points sur le plan et calcule l'aire du convex hull."""
    # Construire une base orthonormale dans le plan
    u = np.array([1, 0, 0]) if abs(plane_normal[0]) < 0.9 else np.array([0, 1, 0])
    v1 = np.cross(plane_normal, u); v1 /= np.linalg.norm(v1)
    v2 = np.cross(plane_normal, v1)
    coords_2d = np.column_stack([points @ v1, points @ v2])
    hull = ConvexHull(coords_2d)
    return hull.volume  # en 2D, volume = aire
```

Seuil typique : rejeter tout plan dont l'aire est inférieure à X % de la plus grande surface trouvée.

---

## Approche 4 — Hypothèse Manhattan World

**Idée** : dans une salle rectangulaire, il n'existe que 3 directions de normales orthogonales entre elles (X, Y, Z). Contraindre RANSAC à ne chercher que des plans dont la normale est proche de l'une de ces 3 directions.

**Pré-traitement** : estimer les 3 axes dominants via PCA ou clustering des normales de surface :

```python
from sklearn.cluster import KMeans

normals = pcd.estimate_normals(...)
kmeans = KMeans(n_clusters=3).fit(normals)
dominant_axes = kmeans.cluster_centers_  # 3 directions principales
```

Puis RANSAC ne valide un plan que si sa normale est à moins de `angle_thresh` degrés d'un axe dominant.

**Avantage** : élimine les plans obliques parasites en plus des doublons parallèles.
**Référence** : *"Manhattan-world stereo"* (Coughlan & Yuille, 1999) ; nombreuses adaptations pour la reconstruction 3D intérieure.

---

## Approche 5 — Regroupement et fusion de plans proches

**Idée** : deux plans quasi-parallèles avec une distance `|d1 - d2|` faible (ex. < 20 cm) sont probablement les deux faces d'un même objet mince (table, cloison).
Les fusionner en un seul plan ou conserver seulement le plus grand.

```python
def group_parallel_planes(planes, normal_thresh=0.05, dist_thresh=0.2):
    groups = []
    used = set()
    for i, p in enumerate(planes):
        if i in used:
            continue
        group = [p]
        for j, q in enumerate(planes[i+1:], i+1):
            if j not in used and are_parallel(p.normal, q.normal, normal_thresh):
                if abs(p.distance - q.distance) < dist_thresh:
                    group.append(q)
                    used.add(j)
        groups.append(group)
        used.add(i)
    # Garder le plan avec le plus d'inliers dans chaque groupe
    return [max(g, key=lambda x: x.inlier_count) for g in groups]
```

---

## Récapitulatif — Quelle approche implémenter en priorité ?

| Approche | Complexité | Efficacité | Recommandée |
|----------|-----------|------------|-------------|
| 1 — Filtrage post par normale | Faible | Bonne | ✅ Commencer par ici |
| 2 — Contrainte dans la boucle | Faible | Bonne | ✅ Complémentaire à 1 |
| 3 — Filtrage par surface | Moyenne | Très bonne | Après 1+2 |
| 4 — Manhattan World | Élevée | Excellente | Recherche avancée |
| 5 — Fusion de plans proches | Faible | Bonne | ✅ Facile à ajouter |

**Recommandation** : implémenter les approches 1 + 2 + 5 ensemble dans un post-processing
`filter_redundant_planes(planes, normal_threshold, ratio_threshold, dist_threshold)` appelé à la fin de `Ransac.py`.

---

## Références

- Fischler & Bolles (1981) — algorithme RANSAC original
- Monszpart et al. (2015) — *RAPter: Rebuilding Man-made Scenes with Regular Arrangements of Planes* — gestion des contraintes de parallélisme/perpendicularité
- Coughlan & Yuille (1999) — Manhattan World hypothesis
- Open3D `segment_plane()` doc — `ransac_n`, `num_iterations`, `distance_threshold`
