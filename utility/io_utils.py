"""I/O helpers using Open3D."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import open3d as o3d

PLY_DIR = Path(__file__).resolve().parents[1] / "ply_files"


def resolve_ply_path(ply_name: str) -> Path:
    path = Path(ply_name)
    if path.suffix.lower() != ".ply":
        path = path.with_suffix(".ply")
    if not path.is_absolute():
        path = PLY_DIR / path.name
    return path


def load_points_from_ply(ply_name: str) -> np.ndarray:
    path = resolve_ply_path(ply_name)
    if not path.exists():
        raise FileNotFoundError(f"Fichier .ply introuvable: {path}")
    pcd = o3d.io.read_point_cloud(str(path))
    if pcd.is_empty():
        raise ValueError("Le point cloud est vide.")
    return np.asarray(pcd.points)
