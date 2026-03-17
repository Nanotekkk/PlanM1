from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans
from sklearn.linear_model import LinearRegression, RANSACRegressor  # ← Library scikit-learn

from utility.analysis_utils import (
    estimate_required_trials,
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
    """Calculate point-to-plane distances using plane equation ax+by+cz+d=0."""
    a, b, c, d = plane
    numerator = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d)
    denominator = np.sqrt(a**2 + b**2 + c**2)
    return numerator / (denominator + 1e-12)


def _fit_plane_open3d(points: np.ndarray, threshold: float = 0.05) -> tuple[np.ndarray, list]:
    """Fit a plane to points using Open3D segmentation."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=threshold,
        ransac_n=3,
        num_iterations=1000,
    )
    
    return np.array(plane_model, dtype=float), inliers


def fit_plane_ransac(
    points: np.ndarray,
    residual_threshold: Optional[float] = None,
    max_trials: int = 1000,
 ) -> PlaneFitResult:
    """Fit plane using scikit-learn RANSAC on (x, y, z) data."""
    if residual_threshold is None:
        diag = np.linalg.norm(points.max(axis=0) - points.min(axis=0)) + 1e-12
        residual_threshold = 0.01 * diag

    X = points[:, :2]
    y = points[:, 2]

    base = LinearRegression()
    # 🔷 APPEL LIBRARY SCIKIT-LEARN: RANSACRegressor
    model = RANSACRegressor(
        estimator=base,
        residual_threshold=residual_threshold,
        max_trials=max_trials,
        random_state=42,
    )
    model.fit(X, y)  # ← Fit du modèle RANSAC

    a, b = model.estimator_.coef_
    c = model.estimator_.intercept_
    normal = np.array([-a, -b, 1.0], dtype=float)
    normal /= np.linalg.norm(normal) + 1e-12
    d = -c

    plane = np.array([normal[0], normal[1], normal[2], d], dtype=float)
    distances_full = _point_to_plane_distance(points, plane)
    inlier_mask = distances_full <= residual_threshold
    inlier_indices = np.where(inlier_mask)[0]

    if not np.any(inlier_mask):
        closest_idx = int(np.argmin(distances_full))
        inlier_mask[closest_idx] = True
        inlier_indices = np.array([closest_idx], dtype=np.int64)

    return PlaneFitResult(
        normal=normal,
        distance=d,
        inlier_indices=inlier_indices,
    )


def _detect_planes_ransac(
    points: np.ndarray,
    max_planes: int = 6,
    min_points: int = 200,
    inlier_ratio_threshold: float = 0.05,
    residual_threshold: Optional[float] = None,
    max_trials: int = 1000,
) -> list[PlaneResult]:
    """Detect multiple planes using Open3D segment_plane (100% library-based)."""
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

    plane_id = 0
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    while len(remaining_indices) >= min_points and plane_id < max_planes:
        subset = np.asarray(pcd.points)
        if residual_threshold is None:
            diag = np.linalg.norm(subset.max(axis=0) - subset.min(axis=0)) + 1e-12
            residual_threshold = 0.01 * diag

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=residual_threshold,
            ransac_n=3,
            num_iterations=max_trials,
        )

        if len(inliers) < min_points:
            break

        inlier_ratio = len(inliers) / len(subset)
        if inlier_ratio < inlier_ratio_threshold:
            break

        inlier_indices = remaining_indices[np.array(inliers, dtype=int)]
        normal = np.array(plane_model[:3], dtype=float)
        d = float(plane_model[3])

        planes.append(
            PlaneResult(
                plane_id=plane_id,
                normal=normal,
                distance=d,
                inlier_indices=inlier_indices,
                inlier_count=len(inlier_indices),
                color=colors[plane_id % len(colors)],
            )
        )

        remaining_indices = np.delete(remaining_indices, inliers)
        pcd = pcd.select_by_index(inliers, invert=True)
        plane_id += 1

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
                
                cluster_points = points[cluster_idx]
                try:
                    plane_model, _ = _fit_plane_open3d(cluster_points, threshold=0.1)
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
                    # If Open3D fitting fails, skip this cluster
                    pass
                
                if len(planes) >= max_planes:
                    break

    return planes


def _load_points(ply_path: Optional[str]) -> tuple[np.ndarray, str]:
    """Load points from PLY file using Open3D."""
    if not ply_path:
        ply_path = input("Nom du fichier .ply (dans ply_files): ").strip()
    if not ply_path:
        raise ValueError("Aucun fichier .ply fourni.")
    return load_points_from_ply(ply_path), ply_path


def _visualize(points: np.ndarray, result: PlaneFitResult) -> None:
    """Visualize detected planes."""
    planes = _detect_planes_ransac(points, max_planes=6)
    if not planes:
        planes = [
            PlaneResult(
                plane_id=0,
                normal=result.normal,
                distance=result.distance,
                inlier_indices=result.inlier_indices,
                inlier_count=len(result.inlier_indices),
                color=(1.0, 0.0, 0.0),
            )
        ]
    visualize_point_cloud_with_planes(points, planes, "RANSAC Plane Detection")


def run(
    ply_path: Optional[str] = None,
    visualize: bool = True,
    max_planes: int = 6,
    min_points: int = 200,
    inlier_ratio_threshold: float = 0.05,
    residual_threshold: Optional[float] = None,
    max_trials: int = 1000,
    confidence: float = 0.99,
    use_sampling: bool = False,
    sampling_method: str = "random",
    sample_target_points: int = 3000,
    sample_seed: int = 42,
    voxel_size: float = 0.05,
    metric: bool = False,
    export_csv: bool = False,
    csv_path: str = "ransac_metrics.csv",
) -> None:
    """Main entry point for RANSAC plane fitting."""
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

    result = fit_plane_ransac(
        sampled_points,
        residual_threshold=residual_threshold,
        max_trials=max_trials,
    )
    if use_sampling:
        result = PlaneFitResult(
            normal=result.normal,
            distance=result.distance,
            inlier_indices=sampled_indices[result.inlier_indices],
        )

    print(f"PLY: {used_ply_path}")
    print("method: ransac")
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

    planes: list[PlaneResult] = []
    if metric or export_csv or visualize:
        planes = _detect_planes_ransac(
            points,
            max_planes=max_planes,
            min_points=min_points,
            inlier_ratio_threshold=inlier_ratio_threshold,
            residual_threshold=residual_threshold,
            max_trials=max_trials,
        )
        if not planes:
            planes = [
                PlaneResult(
                    plane_id=0,
                    normal=result.normal,
                    distance=result.distance,
                    inlier_indices=result.inlier_indices,
                    inlier_count=len(result.inlier_indices),
                    color=(1.0, 0.0, 0.0),
                )
            ]
    if metric or export_csv:
        print(f"planes_detected: {len(planes)}")

    if metric or export_csv:
        rows = []
        for plane in planes:
            mask = np.zeros(len(points), dtype=bool)
            mask[plane.inlier_indices] = True
            metric_values = evaluate_single_plane(
                points,
                np.array([plane.normal[0], plane.normal[1], plane.normal[2], plane.distance], dtype=float),
                mask,
            )
            estimated_trials = estimate_required_trials(
                inlier_ratio=metric_values["inlier_ratio"],
                confidence=confidence,
                min_samples=3,
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
                    "estimated_trials": estimated_trials,
                    "configured_max_trials": max_trials,
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
                print(f"estimated_trials: {row['estimated_trials']}")

    if export_csv:
        csv_output = export_rows_csv(rows, csv_path)
        print(f"csv: {csv_output}")

    if visualize:
        visualize_point_cloud_with_planes(points, planes, "RANSAC Plane Detection")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detection de plans avec RANSAC")
    parser.add_argument("ply_path", nargs="?", default=None, help="Nom ou chemin du fichier .ply")
    parser.add_argument("--max-planes", type=int, default=6, help="Nombre maximal de plans")
    parser.add_argument("--min-points", type=int, default=200, help="Nombre minimal de points par plan")
    parser.add_argument(
        "--inlier-ratio-threshold",
        type=float,
        default=0.05,
        help="Ratio minimal d'inliers pour valider un plan",
    )
    parser.add_argument(
        "--residual-threshold",
        type=float,
        default=None,
        help="Seuil de distance point-plan (defaut: 1%% de la diagonale)",
    )
    parser.add_argument("--max-trials", type=int, default=1000, help="Nombre d'iterations RANSAC")
    parser.add_argument("--confidence", type=float, default=0.99, help="Confiance pour l'estimation theorique des iterations")
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
    parser.add_argument("--csv-path", type=str, default="ransac_metrics.csv", help="Chemin du fichier CSV")
    return parser


if __name__ == "__main__":
    args = _build_arg_parser().parse_args(sys.argv[1:])
    run(
        ply_path=args.ply_path,
        visualize=not args.no_visualize,
        max_planes=args.max_planes,
        min_points=args.min_points,
        inlier_ratio_threshold=args.inlier_ratio_threshold,
        residual_threshold=args.residual_threshold,
        max_trials=args.max_trials,
        confidence=args.confidence,
        use_sampling=args.use_sampling,
        sampling_method=args.sampling_method,
        sample_target_points=args.sample_target_points,
        sample_seed=args.sample_seed,
        voxel_size=args.voxel_size,
        metric=args.metric,
        export_csv=args.export_csv,
        csv_path=args.csv_path,
    )
