"""Six-plane detection for a room using 100% libraries.

Uses Open3D for normal estimation and KMeans for clustering.
"""

from __future__ import annotations

import sys
import argparse
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans  # ← Library scikit-learn

from utility.analysis_utils import (
    color_name_from_rgb,
    evaluate_single_plane,
    export_rows_csv,
    point_to_plane_distance,
    sample_points,
)
from utility.io_utils import load_points_from_ply
from utility.visualizer import PlaneResult, visualize_point_cloud_with_planes


@dataclass
class PlaneModel:
    normal: np.ndarray
    distance: float
    inlier_indices: np.ndarray


def _estimate_normals(points: np.ndarray, radius: float, max_nn: int) -> np.ndarray:
    """Estime les normales de surface via Open3D."""
    # 🔷 APPEL LIBRARY OPEN3D: estimate_normals
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.estimate_normals(  # ← Estimation des normales
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius, max_nn=max_nn
        )
    )
    pcd.orient_normals_consistent_tangent_plane(30)
    return np.asarray(pcd.normals)


def _cluster_normals(normals: np.ndarray, n_clusters: int) -> np.ndarray:
    """Regroupe les normales unitaires en clusters via KMeans."""
    norms = np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12
    unit = normals / norms
    # 🔷 APPEL LIBRARY SCIKIT-LEARN: KMeans
    kmeans = KMeans(n_clusters=n_clusters, n_init=10, random_state=42)
    return kmeans.fit_predict(unit)  # ← Clustering des normales


def _fit_plane_open3d(points: np.ndarray, threshold: float = 0.05) -> np.ndarray:
    """Ajuste un plan aux points via Open3D.segment_plane."""
    # 🔷 APPEL LIBRARY OPEN3D: estimate_normals et segment_plane
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    plane_model, _ = pcd.segment_plane(  # ← RANSAC d'Open3D
        distance_threshold=threshold,
        ransac_n=3,
        num_iterations=1000,
    )
    
    return np.array(plane_model, dtype=float)


def _build_plane_models(
    reference_points: np.ndarray,
    sample_points: np.ndarray,
    sample_to_full_indices: np.ndarray,
    labels: np.ndarray,
    distance_threshold: float,
    n_planes: int,
    min_points: int,
) -> List[PlaneModel]:
    """Construit les modèles de plan pour les clusters de normales."""
    planes = []
    # Conserve les plus grands clusters pour garantir n_planes plans détectés
    unique, counts = np.unique(labels, return_counts=True)
    order = np.argsort(counts)[::-1]
    top_labels = unique[order][:n_planes]

    for label in top_labels:
        cluster_idx = np.where(labels == label)[0]
        cluster_points = sample_points[cluster_idx]

        if len(cluster_points) < 3:
            continue

        # Ajuste un plan au cluster via Open3D puis raffine avec seuil de distance
        try:
            plane = _fit_plane_open3d(cluster_points, threshold=0.1)
        except Exception:
            continue
            
        distances = point_to_plane_distance(reference_points, plane)
        inlier_mask = distances <= distance_threshold
        inlier_indices = np.where(inlier_mask)[0]

        # Fallback au cluster brut si le filtre de distance est trop strict
        if len(inlier_indices) < min_points:
            inlier_indices = sample_to_full_indices[cluster_idx]

        planes.append(
            PlaneModel(
                normal=plane[:3],
                distance=plane[3],
                inlier_indices=inlier_indices,
            )
        )

    return planes


def _load_points(ply_path: Optional[str]) -> tuple[np.ndarray, str]:
    """Charge les points depuis un fichier PLY via Open3D."""
    if not ply_path:
        ply_path = input("Nom du fichier .ply (dans ply_files): ").strip()
    if not ply_path:
        raise ValueError("Aucun fichier .ply fourni.")
    return load_points_from_ply(ply_path), ply_path


def run(
    ply_path: Optional[str] = None,
    n_planes: int = 6,
    distance_threshold: Optional[float] = None,
    visualize: bool = True,
    use_sampling: bool = False,
    sampling_method: str = "random",
    sample_target_points: int = 4000,
    sample_seed: int = 42,
    voxel_size: float = 0.05,
    metric: bool = False,
    export_csv: bool = False,
    csv_path: str = "room_planes_metrics.csv",
) -> None:
    """Point d'entrée principal pour la détection de plans de salle."""
    points, used_ply_path = _load_points(ply_path)

    sampled_points = points
    sampled_indices = np.arange(len(points), dtype=np.int64)
    sampling_info = None
    if use_sampling:
        sampled_points, sampled_indices, sampling_info = sample_points(
            points,
            method=sampling_method,
            target_points=sample_target_points,
            seed=sample_seed,
            voxel_size=voxel_size,
        )

    # Définit le seuil de distance par défaut basé sur la diagonale
    diag = np.linalg.norm(points.max(axis=0) - points.min(axis=0)) + 1e-12
    if distance_threshold is None:
        distance_threshold = 0.01 * diag

    min_points = max(100, len(sampled_points) // 200)
    # Methode unique: normals + KMeans.
    normals = _estimate_normals(sampled_points, radius=0.05 * diag, max_nn=50)
    labels = _cluster_normals(normals, n_clusters=n_planes)
    planes = _build_plane_models(
        reference_points=points,
        sample_points=sampled_points,
        sample_to_full_indices=sampled_indices,
        labels=labels,
        distance_threshold=distance_threshold,
        n_planes=n_planes,
        min_points=min_points,
    )

    print(f"PLY: {used_ply_path}")
    print("method: normals_kmeans")
    print(f"planes_detected: {len(planes)}")
    if sampling_info is not None and sampling_info.used_sampling:
        details = f"method={sampling_info.method}"
        if sampling_info.method == "random" and sampling_info.seed is not None:
            details += f", seed={sampling_info.seed}"
        if sampling_info.method == "voxel" and sampling_info.voxel_size is not None:
            details += f", voxel_size={sampling_info.voxel_size}"
        print(
            "Sampling: "
            f"{sampling_info.sampled_points}/{sampling_info.total_points} points "
            f"({details})"
        )
    else:
        print(f"Sampling: off ({len(points)} points used)")

    colors = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 1.0, 0.0),
        (1.0, 0.0, 1.0),
        (0.0, 1.0, 1.0),
    ]

    if metric or export_csv:
        rows = []
        for idx, plane in enumerate(planes):
            color = colors[idx % len(colors)]
            mask = np.zeros(len(points), dtype=bool)
            mask[plane.inlier_indices] = True
            metric_values = evaluate_single_plane(
                points,
                np.array([plane.normal[0], plane.normal[1], plane.normal[2], plane.distance], dtype=float),
                mask,
            )
            rows.append(
                {
                    "ply_file": used_ply_path,
                    "plane_id": idx,
                    "color_name": color_name_from_rgb(color),
                    "color_r": float(color[0]),
                    "color_g": float(color[1]),
                    "color_b": float(color[2]),
                    "normal_x": float(plane.normal[0]),
                    "normal_y": float(plane.normal[1]),
                    "normal_z": float(plane.normal[2]),
                    "distance": float(plane.distance),
                    "inlier_count": int(len(plane.inlier_indices)),
                    "inlier_ratio": metric_values["inlier_ratio"],
                    "rmse": metric_values["rmse"],
                    "score": metric_values["score"],
                }
            )
        if metric:
            print("Metrics:")
            for row in rows:
                print(f"Plane {row['plane_id']}:")
                print(
                    "normal: "
                    f"({row['normal_x']:.6f}, {row['normal_y']:.6f}, {row['normal_z']:.6f})"
                )
                print(f"distance: {row['distance']:.6f}")
                print(f"inlier_count: {row['inlier_count']}")
                print(f"inlier_ratio: {row['inlier_ratio']:.6f}")
                print(f"rmse: {row['rmse']:.6f}")
                print(f"score: {row['score']:.6f}")
        if export_csv:
            csv_output = export_rows_csv(rows, csv_path)
            print(f"csv: {csv_output}")

    # Visualise les plans via Open3D/custom visualizer
    if visualize and planes:
        plane_results = []
        for idx, plane in enumerate(planes):
            plane_results.append(
                PlaneResult(
                    plane_id=idx,
                    normal=plane.normal,
                    distance=plane.distance,
                    inlier_indices=plane.inlier_indices,
                    inlier_count=len(plane.inlier_indices),
                    color=colors[idx % len(colors)],
                )
            )
        visualize_point_cloud_with_planes(points, plane_results, "Room Plane Detection")

##parseur d'arguments pour la ligne de commande

def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detection de plans avec normales + KMeans")
    parser.add_argument("ply_path", nargs="?", default=None, help="Nom ou chemin du fichier .ply")
    parser.add_argument("--n-planes", type=int, default=6, help="Nombre de plans a detecter")
    parser.add_argument(
        "--distance-threshold",
        type=float,
        default=None,
        help="Seuil de distance point-plan (defaut: 1%% de la diagonale)",
    )
    parser.add_argument(
        "--no-visualize",
        action="store_true",
        help="Desactive la visualisation Open3D",
    )
    parser.add_argument("--metric", action="store_true", help="Affiche les metriques en console")
    parser.add_argument("--use-sampling", "--echant", dest="use_sampling", action="store_true", help="Active l'echantillonnage aleatoire")
    parser.add_argument(
        "--sampling-method",
        "--echant-method",
        dest="sampling_method",
        choices=["random", "voxel"],
        default="random",
        help="Methode d'echantillonnage a utiliser",
    )
    parser.add_argument("--sample-target-points", type=int, default=4000, help="Nombre de points echantillonnes")
    parser.add_argument("--sample-seed", type=int, default=42, help="Graine aleatoire pour l'echantillonnage")
    parser.add_argument("--voxel-size", type=float, default=0.05, help="Taille de voxel pour l'echantillonnage voxel")
    parser.add_argument("--export-csv", action="store_true", help="Exporte les metriques en CSV")
    parser.add_argument("--csv-path", type=str, default="room_planes_metrics.csv", help="Chemin du fichier CSV")
    return parser


if __name__ == "__main__":
    args = _build_arg_parser().parse_args(sys.argv[1:])
    run(
        ply_path=args.ply_path,
        n_planes=args.n_planes,
        distance_threshold=args.distance_threshold,
        visualize=not args.no_visualize,
        use_sampling=args.use_sampling,
        sampling_method=args.sampling_method,
        sample_target_points=args.sample_target_points,
        sample_seed=args.sample_seed,
        voxel_size=args.voxel_size,
        metric=args.metric,
        export_csv=args.export_csv,
        csv_path=args.csv_path,
    )
