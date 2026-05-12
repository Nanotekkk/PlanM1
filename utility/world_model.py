"""Manhattan World et Atlanta World — estimation d'axes et recalage des plans.

Manhattan World (Coughlan & Yuille 1999) :
    Hypothèse que la scène est structurée selon 3 axes orthogonaux dominants
    (ex. sol/plafond + 2 paires de murs perpendiculaires). Les axes sont estimés
    par ACP sur les normales de surface et les plans détectés sont recalés sur
    l'axe le plus proche.

Atlanta World (Schindler et al. 2004) :
    Un axe vertical unique (gravité) + N directions horizontales arbitraires
    (murs non nécessairement orthogonaux). Adapté aux scènes en L ou polygonales.
"""

from __future__ import annotations

import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans

from utility.analysis_utils import point_to_plane_distance
from utility.visualizer import PlaneResult


# ---------------------------------------------------------------------------
# Estimation des normales de surface (sous-échantillonné pour la rapidité)
# ---------------------------------------------------------------------------

def _estimate_scene_normals(points: np.ndarray, max_points: int = 2000) -> np.ndarray:
    """Estime des normales de surface sur un sous-échantillon du nuage de points.

    Utilisé en interne pour l'estimation des axes dominants quand aucune
    normale pré-calculée n'est disponible.

    Args:
        points: Nuage de points, shape (N, 3).
        max_points: Nombre maximum de points à utiliser.

    Returns:
        Normales estimées, shape (M, 3).
    """
    if len(points) > max_points:
        rng = np.random.default_rng(42)
        idx = rng.choice(len(points), max_points, replace=False)
        pts = points[idx]
    else:
        pts = points

    diag = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0))) + 1e-12
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05 * diag, max_nn=30)
    )
    pcd.orient_normals_consistent_tangent_plane(15)
    return np.asarray(pcd.normals)


# ---------------------------------------------------------------------------
# Manhattan World
# ---------------------------------------------------------------------------

def estimate_manhattan_axes(normals: np.ndarray) -> np.ndarray:
    """Estime les 3 axes orthogonaux dominants par ACP sur les normales.

    Toutes les normales sont projetées dans l'hémisphère supérieur (z ≥ 0)
    avant le calcul de la covariance pour lever l'ambiguïté ±.

    Référence : Coughlan & Yuille, « Manhattan World: Compass Direction from
    a Single Image by Bayesian Inference », ICCV 1999.

    Args:
        normals: Normales de surface, shape (N, 3).

    Returns:
        axes : shape (3, 3), chaque ligne est un vecteur unité.
    """
    unit = normals / (np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12)
    unit = unit.copy()
    # Projeter dans l'hémisphère supérieur pour lever l'ambiguïté ±
    unit[unit[:, 2] < 0] *= -1
    cov = np.cov(unit.T)
    _, eigenvectors = np.linalg.eigh(cov)
    # eigh retourne les colonnes triées par valeur propre croissante → inverser
    axes = eigenvectors[:, ::-1].T  # (3, 3), chaque ligne = 1 axe
    # Assurer un repère direct (règle de la main droite)
    if np.dot(np.cross(axes[0], axes[1]), axes[2]) < 0:
        axes[2] = -axes[2]
    norms = np.linalg.norm(axes, axis=1, keepdims=True) + 1e-12
    return axes / norms


# ---------------------------------------------------------------------------
# Atlanta World
# ---------------------------------------------------------------------------

def estimate_atlanta_axes(
    normals: np.ndarray,
    n_horizontal: int = 4,
    vertical_tol: float = 0.3,
) -> np.ndarray:
    """Estime les axes sous le modèle Atlanta World.

    Identifie un axe vertical dominant (sol/plafond) et cluster les directions
    horizontales (murs) en n_horizontal groupes. Les murs n'ont pas à être
    orthogonaux entre eux.

    Référence : Schindler, Wang & Szeliski, « Atlanta World: An Urban Image
    Representation Incorporating Geometric and Semantic Context », CVPR 2004.

    Args:
        normals: Normales de surface, shape (N, 3).
        n_horizontal: Nombre de directions de murs horizontaux à détecter.
        vertical_tol: Une normale est considérée "verticale" (sol/plafond) si
                      |n · Z| > 1 − vertical_tol (valeur par défaut 0.3 ≈ ±72°
                      par rapport à l'horizontale).

    Returns:
        axes : shape (1 + n_horizontal, 3).
               Première ligne = axe vertical, suivantes = directions horizontales.
    """
    unit = normals / (np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12)
    unit = unit.copy()

    # ------------------------------------------------------------------
    # 1. Axe vertical dominant (direction sol/plafond)
    # ------------------------------------------------------------------
    z_align = np.abs(unit[:, 2])
    v_mask = z_align > (1.0 - vertical_tol)
    if np.sum(v_mask) >= 3:
        v_normals = unit[v_mask].copy()
        # Projeter dans l'hémisphère supérieur
        v_normals[v_normals[:, 2] < 0] *= -1
        cov = np.cov(v_normals.T)
        _, evecs = np.linalg.eigh(cov)
        vertical_axis = evecs[:, -1].copy()  # vecteur propre dominant
    else:
        # Fallback : axe Z mondial
        vertical_axis = np.array([0.0, 0.0, 1.0])
    if vertical_axis[2] < 0:
        vertical_axis = -vertical_axis
    vertical_axis /= np.linalg.norm(vertical_axis) + 1e-12

    # ------------------------------------------------------------------
    # 2. Directions de murs horizontaux (perpendiculaires à l'axe vertical)
    # ------------------------------------------------------------------
    h_align = np.abs(unit @ vertical_axis)
    h_mask = h_align < vertical_tol
    if np.sum(h_mask) < max(n_horizontal, 3):
        # Pas assez de normales horizontales → utiliser toutes les normales
        h_mask = np.ones(len(unit), dtype=bool)
    h_normals = unit[h_mask].copy()

    # Projeter dans le plan horizontal et normaliser
    proj = h_normals - (h_normals @ vertical_axis)[:, None] * vertical_axis
    proj_norms = np.linalg.norm(proj, axis=1, keepdims=True) + 1e-12
    proj = proj / proj_norms

    # Lever l'ambiguïté ± via un vecteur de référence perpendiculaire à l'axe vertical
    ref = np.array([1.0, 0.0, 0.0])
    if np.abs(np.dot(ref, vertical_axis)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    ref = ref - np.dot(ref, vertical_axis) * vertical_axis
    ref /= np.linalg.norm(ref) + 1e-12
    proj[proj @ ref < 0] *= -1

    # Clustering des directions horizontales
    n_clust = min(n_horizontal, len(proj))
    kmeans = KMeans(n_clusters=n_clust, n_init=10, random_state=42)
    kmeans.fit(proj)
    centers = kmeans.cluster_centers_
    centers /= np.linalg.norm(centers, axis=1, keepdims=True) + 1e-12

    return np.vstack([vertical_axis[None, :], centers])


# ---------------------------------------------------------------------------
# Dispatcher
# ---------------------------------------------------------------------------

def estimate_dominant_axes(
    points: np.ndarray,
    model: str,
    normals: np.ndarray | None = None,
    n_horizontal: int = 4,
) -> np.ndarray:
    """Estime les axes dominants selon le modèle de monde choisi.

    Args:
        points: Nuage de points, shape (N, 3).
        model: ``"manhattan"`` ou ``"atlanta"``.
        normals: Normales pré-calculées, shape (M, 3). Si ``None`` ou vide,
                 les normales sont estimées automatiquement sur un sous-échantillon.
        n_horizontal: Nombre de directions horizontales (Atlanta World uniquement).

    Returns:
        axes : shape (K, 3), chaque ligne est un vecteur unité.

    Raises:
        ValueError: Si ``model`` n'est ni ``"manhattan"`` ni ``"atlanta"``.
    """
    if normals is None or len(normals) == 0:
        normals = _estimate_scene_normals(points)
    if model == "manhattan":
        return estimate_manhattan_axes(normals)
    if model == "atlanta":
        return estimate_atlanta_axes(normals, n_horizontal=n_horizontal)
    raise ValueError(f"Modèle inconnu : {model!r}. Choisir 'manhattan' ou 'atlanta'.")


# ---------------------------------------------------------------------------
# Recalage des plans sur les axes dominants
# ---------------------------------------------------------------------------

def snap_planes_to_world(
    planes: list[PlaneResult],
    axes: np.ndarray,
    points: np.ndarray,
    angle_tol_deg: float = 15.0,
    distance_threshold: float = 0.05,
) -> list[PlaneResult]:
    """Recale les normales des plans détectés sur les axes dominants du modèle de monde.

    Pour chaque plan :
    - On cherche l'axe signé (±axe) le plus aligné avec la normale du plan.
    - Si l'angle entre la normale et cet axe est inférieur à ``angle_tol_deg``,
      la normale est remplacée par cet axe, ``d`` est recalculé depuis les
      inliers existants, et les inliers sont réévalués avec le plan recalé.
    - Sinon, le plan est écarté (considéré comme un artefact hors modèle).

    Args:
        planes: Liste de :class:`PlaneResult` détectés.
        axes: Axes dominants, shape (K, 3), chaque ligne est un vecteur unité.
        points: Nuage de points complet, shape (N, 3).
        angle_tol_deg: Angle maximum (degrés) pour recaler une normale sur un axe.
        distance_threshold: Seuil de distance (mètres) pour le recalcul des inliers.

    Returns:
        Liste filtrée de :class:`PlaneResult` avec normales recalées.
    """
    cos_tol = float(np.cos(np.deg2rad(angle_tol_deg)))
    norm_axes = axes / (np.linalg.norm(axes, axis=1, keepdims=True) + 1e-12)
    # Axes signés : chaque direction et son opposé pour couvrir les deux sens
    signed = np.vstack([norm_axes, -norm_axes])  # (2K, 3)

    result: list[PlaneResult] = []
    for plane in planes:
        n = np.array(plane.normal, dtype=float)
        n /= np.linalg.norm(n) + 1e-12

        dots = signed @ n
        best_idx = int(np.argmax(dots))
        if float(dots[best_idx]) < cos_tol:
            # Normale trop éloignée de tout axe dominant → plan ignoré
            continue

        snapped_n = signed[best_idx].copy()

        # Recalcul de d depuis le centroïde des inliers projeté sur la normale recalée
        inlier_pts = points[plane.inlier_indices]
        new_d = -float(np.mean(inlier_pts @ snapped_n))

        # Réévaluation des inliers avec le plan recalé
        plane_eq = np.append(snapped_n, new_d)
        dists = point_to_plane_distance(points, plane_eq)
        new_inliers = np.where(dists <= distance_threshold)[0]
        if len(new_inliers) < 3:
            # Seuil trop strict → conserver les inliers d'origine
            new_inliers = plane.inlier_indices

        result.append(
            PlaneResult(
                plane_id=plane.plane_id,
                normal=snapped_n,
                distance=new_d,
                inlier_indices=new_inliers,
                inlier_count=len(new_inliers),
                color=plane.color,
            )
        )

    return result
