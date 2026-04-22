"""Optional analysis helpers (sampling, metrics, CSV export)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


@dataclass
class SamplingInfo:
    used_sampling: bool
    total_points: int
    sampled_points: int
    method: str
    seed: int | None = None
    voxel_size: float | None = None


def random_sample_points(
    points: np.ndarray,
    target_points: int = 3000,
    seed: int = 42,
) -> tuple[np.ndarray, np.ndarray, SamplingInfo]:
    """Sample points randomly without replacement (reproducible)."""
    n_points = len(points)
    if n_points <= target_points:
        indices = np.arange(n_points, dtype=np.int64)
        return (
            points,
            indices,
            SamplingInfo(
                used_sampling=False,
                total_points=n_points,
                sampled_points=n_points,
                method="random",
                seed=seed,
            ),
        )

    rng = np.random.default_rng(seed)
    indices = np.sort(rng.choice(n_points, size=target_points, replace=False))
    return (
        points[indices],
        indices,
        SamplingInfo(
            used_sampling=True,
            total_points=n_points,
            sampled_points=len(indices),
            method="random",
            seed=seed,
        ),
    )


def voxel_sample_points(
    points: np.ndarray,
    voxel_size: float = 0.05,
) -> tuple[np.ndarray, np.ndarray, SamplingInfo]:
    """Downsample points by voxel grid using one representative original point per voxel."""
    if voxel_size <= 0:
        raise ValueError("voxel_size must be > 0")

    n_points = len(points)
    if n_points == 0:
        indices = np.array([], dtype=np.int64)
        return (
            points,
            indices,
            SamplingInfo(
                used_sampling=False,
                total_points=0,
                sampled_points=0,
                method="voxel",
                voxel_size=voxel_size,
            ),
        )

    mins = points.min(axis=0)
    voxel_keys = np.floor((points - mins) / voxel_size).astype(np.int64)
    _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
    indices = np.sort(unique_indices.astype(np.int64))

    return (
        points[indices],
        indices,
        SamplingInfo(
            used_sampling=len(indices) < n_points,
            total_points=n_points,
            sampled_points=len(indices),
            method="voxel",
            voxel_size=voxel_size,
        ),
    )


def sample_points(
    points: np.ndarray,
    method: str = "random",
    target_points: int = 3000,
    seed: int = 42,
    voxel_size: float = 0.05,
) -> tuple[np.ndarray, np.ndarray, SamplingInfo]:
    """Sample points with the requested method."""
    if method == "random":
        return random_sample_points(points, target_points=target_points, seed=seed)
    if method == "voxel":
        return voxel_sample_points(points, voxel_size=voxel_size)
    raise ValueError(f"Unknown sampling method: {method}")


def point_to_plane_distance(points: np.ndarray, plane: np.ndarray) -> np.ndarray:
    """Calculate point-to-plane distances using plane equation ax+by+cz+d=0."""
    a, b, c, d = plane
    numerator = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d)
    denominator = np.sqrt(a**2 + b**2 + c**2)
    return numerator / (denominator + 1e-12)


def evaluate_single_plane(
    points: np.ndarray,
    plane: np.ndarray,
    inlier_mask: np.ndarray,
) -> dict[str, float]:
    """Return inlier_ratio/rmse/score for one plane."""
    distances = point_to_plane_distance(points, plane)
    inlier_count = int(np.sum(inlier_mask))
    inlier_ratio = float(np.mean(inlier_mask))

    if inlier_count == 0:
        return {
            "inlier_ratio": 0.0,
            "rmse": float("inf"),
            "score": 0.0,
        }

    rmse = float(np.sqrt(np.mean(distances[inlier_mask] ** 2)))

    diag = np.linalg.norm(points.max(axis=0) - points.min(axis=0)) + 1e-12
    rmse_norm = min(rmse / diag, 1.0)
    score = 0.6 * inlier_ratio + 0.4 * (1.0 - rmse_norm) #inutile on peut supprimer car les méthodes sont différentes # a demander,car c'est arbitraire et pas forcément optimal ?

    return {
        "inlier_ratio": inlier_ratio,
        "rmse": rmse,
        "score": float(score),
    }


def estimate_required_trials(
    inlier_ratio: float,
    confidence: float = 0.99,
    min_samples: int = 3,
) -> int:
    """Theoretical RANSAC iteration estimate."""
    w = float(np.clip(inlier_ratio, 1e-6, 1.0 - 1e-12))
    p = float(np.clip(confidence, 1e-12, 1.0 - 1e-12))
    denom = math.log(1.0 - (w**min_samples))
    if abs(denom) < 1e-15:
        return 1
    n = math.log(1.0 - p) / denom
    return int(max(1, math.ceil(n)))


def color_name_from_rgb(color: tuple[float, float, float]) -> str:
    """Return a readable color name for known palette colors."""
    r, g, b = (float(color[0]), float(color[1]), float(color[2]))
    key = (round(r, 3), round(g, 3), round(b, 3))
    names = {
        (1.0, 0.0, 0.0): "rouge",
        (0.0, 1.0, 0.0): "vert",
        (0.0, 0.0, 1.0): "bleu",
        (1.0, 1.0, 0.0): "jaune",
        (1.0, 0.0, 1.0): "magenta",
        (0.0, 1.0, 1.0): "cyan",
        (1.0, 0.5, 0.0): "orange",
        (0.5, 0.0, 1.0): "violet",
    }
    if key in names:
        return names[key]
    return f"rgb({r:.3f},{g:.3f},{b:.3f})"


def export_rows_csv(rows: list[dict[str, object]], csv_path: str) -> Path:
    """Append rows to CSV while preserving existing data and schema changes."""
    out = Path(csv_path)
    new_df = pd.DataFrame(rows)
    if out.exists():
        existing_df = pd.read_csv(out)
        merged_df = pd.concat([existing_df, new_df], ignore_index=True, sort=False)
    else:
        merged_df = new_df
    merged_df = merged_df.drop(columns=["planarity"], errors="ignore")
    ordered_columns = [
        "ply_file",
        "plane_id",
        "color_name",
        "color_r",
        "color_g",
        "color_b",
        "normal_x",
        "normal_y",
        "normal_z",
        "distance",
        "inlier_count",
        "inlier_ratio",
        "rmse",
        "score",
        "estimated_trials",
        "configured_max_trials",
    ]
    remaining_columns = [col for col in merged_df.columns if col not in ordered_columns]
    merged_df = merged_df[[col for col in ordered_columns if col in merged_df.columns] + remaining_columns]
    merged_df.to_csv(out, index=False)
    return out.resolve()
