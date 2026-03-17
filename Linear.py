"""Linear regression plane fitting using 100% libraries.

Uses scikit-learn LinearRegression and Open3D for plane fitting.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans
from sklearn.linear_model import LinearRegression  # ← Library scikit-learn

from utility.analysis_utils import (
    evaluate_single_plane,
    export_rows_csv,
    sample_points,
)
from utility.io_utils import load_points_from_ply
from utility.visualizer import PlaneResult, visualize_point_cloud_with_planes


@dataclass
class PlaneFitResult:
    normal: np.ndarray
    distance: float
    inlier_indices: np.ndarray


def _point_to_plane_distance(points: np.ndarray, plane: np.ndarray) -> np.ndarray:
    """Distance de chaque point au plan ax+by+cz+d=0."""
    a, b, c, d = plane
    numerator = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d)
    denominator = np.sqrt(a**2 + b**2 + c**2)
    return numerator / (denominator + 1e-12)


def _fit_plane_open3d(points: np.ndarray, threshold: float = 0.05) -> np.ndarray:
    """Ajuste un plan aux points via Open3D.segment_plane."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    plane_model, _ = pcd.segment_plane(
        distance_threshold=threshold,
        ransac_n=3,
        num_iterations=1000,
    )
    
    return np.array(plane_model, dtype=float)


def fit_plane_linear(
    points: np.ndarray,
    inlier_percentile: float = 15.0,
 ) -> PlaneFitResult:
    """Ajuste un plan via régression linéaire sklearn sur les résidus."""
    X = points[:, :2]
    y = points[:, 2]

    # 🔷 APPEL LIBRARY SCIKIT-LEARN: LinearRegression
    model = LinearRegression()
    model.fit(X, y)  # ← Fit du modèle de régression linéaire
    y_pred = model.predict(X)

    # Déterminer les inliers selon le percentile des résidus
    residuals = np.abs(y - y_pred)
    threshold = np.percentile(residuals, inlier_percentile)
    y_pred_full = model.predict(points[:, :2])
    residuals_full = np.abs(points[:, 2] - y_pred_full)
    inlier_mask = residuals_full <= threshold
    inlier_indices = np.where(inlier_mask)[0]

    # Convertir coefficients en équation de plan ax+by+cz+d=0
    a, b = model.coef_
    c = model.intercept_
    normal = np.array([-a, -b, 1.0], dtype=float)
    normal /= np.linalg.norm(normal) + 1e-12
    d = -c

    return PlaneFitResult(
        normal=normal,
        distance=d,
        inlier_indices=inlier_indices,
    )


def _load_points(ply_path: Optional[str]) -> tuple[np.ndarray, str]:
    """Charge les points depuis un fichier PLY via Open3D."""
    if not ply_path:
        ply_path = input("Nom du fichier .ply (dans ply_files): ").strip()
    if not ply_path:
        raise ValueError("Aucun fichier .ply fourni.")
    return load_points_from_ply(ply_path), ply_path


def _detect_planes_linear(
    points: np.ndarray,
    max_planes: int = 6,
    inlier_percentile: float = 15.0,
    min_points: int = 200,
) -> list[PlaneResult]:
    """Détecte plusieurs plans itérativement avec régression linéaire."""
    remaining_indices = np.arange(len(points))
    planes: list[PlaneResult] = []
    colors = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 1.0, 0.0),
        (1.0, 0.0, 1.0),
        (0.0, 1.0, 1.0),
        (1.0, 0.5, 0.0),
        (0.5, 0.0, 1.0),
    ]

    # Boucle principale de détection des plans
    while len(remaining_indices) >= min_points and len(planes) < max_planes:
        subset = points[remaining_indices]
        model = LinearRegression()
        model.fit(subset[:, :2], subset[:, 2])
        y_pred = model.predict(subset[:, :2])
        residuals = np.abs(subset[:, 2] - y_pred)
        threshold = np.percentile(residuals, inlier_percentile)
        inlier_mask = residuals <= threshold

        if np.sum(inlier_mask) < min_points:
            break

        # Récupère les indices des inliers
        inlier_indices = remaining_indices[inlier_mask]
        a, b = model.coef_
        c = model.intercept_
        normal = np.array([-a, -b, 1.0], dtype=float)
        normal /= np.linalg.norm(normal) + 1e-12
        d = -c

        planes.append(
            PlaneResult(
                plane_id=len(planes),
                normal=normal,
                distance=d,
                inlier_indices=inlier_indices,
                inlier_count=len(inlier_indices),
                color=colors[len(planes) % len(colors)],
            )
        )

        # Retire les inliers des points restants
        remaining_indices = remaining_indices[~inlier_mask]

    # Groupe les points restants avec KMeans et ajuste un plan par cluster via Open3D
    if len(planes) < max_planes and len(remaining_indices) >= 3:
        needed = max_planes - len(planes)
        remaining_points = points[remaining_indices]
        k = min(needed, len(remaining_points))
        if k >= 1:
            kmeans = KMeans(n_clusters=k, n_init=10, random_state=42)
            labels = kmeans.fit_predict(remaining_points)
            for label in range(k):
                cluster_idx = remaining_indices[labels == label]
                if len(cluster_idx) < 3:
                    continue
                
                # Ajuste un plan au cluster via Open3D
                cluster_points = points[cluster_idx]
                try:
                    plane_model = _fit_plane_open3d(cluster_points, threshold=0.1)
                    planes.append(
                        PlaneResult(
                            plane_id=len(planes),
                            normal=plane_model[:3],
                            distance=plane_model[3],
                            inlier_indices=cluster_idx,
                            inlier_count=len(cluster_idx),
                            color=colors[len(planes) % len(colors)],
                        )
                    )
                except Exception:
                    pass
                
                if len(planes) >= max_planes:
                    break

    return planes


def _visualize(points: np.ndarray, result: PlaneFitResult) -> None:
    """Visualise les plans détectés."""
    plane_results = _detect_planes_linear(points, max_planes=6)
    if not plane_results:
        plane_results = [
            PlaneResult(
                plane_id=0,
                normal=result.normal,
                distance=result.distance,
                inlier_indices=result.inlier_indices,
                inlier_count=len(result.inlier_indices),
                color=(0.0, 0.6, 1.0),
            )
        ]
    visualize_point_cloud_with_planes(points, plane_results, "Linear Plane Detection")


def run(
    ply_path: Optional[str] = None,
    visualize: bool = True,
    max_planes: int = 6,
    inlier_percentile: float = 15.0,
    min_points: int = 200,
    use_sampling: bool = False,
    sampling_method: str = "random",
    sample_target_points: int = 3000,
    sample_seed: int = 42,
    voxel_size: float = 0.05,
    metric: bool = False,
    export_csv: bool = False,
    csv_path: str = "linear_metrics.csv",
) -> None:
    """Point d'entrée principal pour l'ajustement de plan linéaire."""
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

    result = fit_plane_linear(sampled_points, inlier_percentile=inlier_percentile)
    if use_sampling:
        result = PlaneFitResult(
            normal=result.normal,
            distance=result.distance,
            inlier_indices=sampled_indices[result.inlier_indices],
        )

    print(f"PLY: {used_ply_path}")
    print("method: linear")
    print(f"planes_detected_target: {max_planes}")
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

    plane_results: list[PlaneResult] = []
    if metric or export_csv or visualize:
        plane_results = _detect_planes_linear(
            points,
            max_planes=max_planes,
            inlier_percentile=inlier_percentile,
            min_points=min_points,
        )
        if not plane_results:
            plane_results = [
                PlaneResult(
                    plane_id=0,
                    normal=result.normal,
                    distance=result.distance,
                    inlier_indices=result.inlier_indices,
                    inlier_count=len(result.inlier_indices),
                    color=(0.0, 0.6, 1.0),
                )
            ]
    if metric or export_csv:
        print(f"planes_detected: {len(plane_results)}")

    if metric or export_csv:
        rows = []
        for plane in plane_results:
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
                    "plane_id": int(plane.plane_id),
                    "normal_x": float(plane.normal[0]),
                    "normal_y": float(plane.normal[1]),
                    "normal_z": float(plane.normal[2]),
                    "distance": float(plane.distance),
                    "inlier_count": int(plane.inlier_count),
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

    if visualize:
        visualize_point_cloud_with_planes(points, plane_results, "Linear Plane Detection")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detection de plans par regression lineaire")
    parser.add_argument("ply_path", nargs="?", default=None, help="Nom ou chemin du fichier .ply")
    parser.add_argument("--max-planes", type=int, default=6, help="Nombre maximal de plans")
    parser.add_argument(
        "--inlier-percentile",
        type=float,
        default=15.0,
        help="Percentile de residus pour definir les inliers",
    )
    parser.add_argument("--min-points", type=int, default=200, help="Nombre minimal de points par plan")
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
    parser.add_argument("--sample-target-points", type=int, default=3000, help="Nombre de points echantillonnes")
    parser.add_argument("--sample-seed", type=int, default=42, help="Graine aleatoire pour l'echantillonnage")
    parser.add_argument("--voxel-size", type=float, default=0.05, help="Taille de voxel pour l'echantillonnage voxel")
    parser.add_argument("--export-csv", action="store_true", help="Exporte les metriques en CSV")
    parser.add_argument("--csv-path", type=str, default="linear_metrics.csv", help="Chemin du fichier CSV")
    return parser


if __name__ == "__main__":
    args = _build_arg_parser().parse_args(sys.argv[1:])
    run(
        ply_path=args.ply_path,
        visualize=not args.no_visualize,
        max_planes=args.max_planes,
        inlier_percentile=args.inlier_percentile,
        min_points=args.min_points,
        use_sampling=args.use_sampling,
        sampling_method=args.sampling_method,
        sample_target_points=args.sample_target_points,
        sample_seed=args.sample_seed,
        voxel_size=args.voxel_size,
        metric=args.metric,
        export_csv=args.export_csv,
        csv_path=args.csv_path,
    )
