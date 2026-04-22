"""Open3D visualization helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import open3d as o3d


@dataclass
class PlaneResult:
    plane_id: int
    normal: np.ndarray
    distance: float
    inlier_indices: np.ndarray
    inlier_count: int
    color: Tuple[float, float, float]


def visualize_point_cloud_with_planes(
    points: np.ndarray,
    planes: List[PlaneResult],
    title: str,
    background_color: Tuple[float, float, float] = (1.0, 1.0, 1.0),
) -> None:
    vis = o3d.visualization.Visualizer()
    vis.create_window(title, width=1200, height=800)

    colors = np.ones((len(points), 3))
    colors[:] = (0.8, 0.8, 0.8)
    for plane in planes:
        colors[plane.inlier_indices] = plane.color

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    vis.add_geometry(pcd)

    opt = vis.get_render_option()
    opt.background_color = np.array(background_color)
    opt.point_size = 3.0

    vis.run()
    vis.destroy_window()


def visualize_planes_one_by_one(
    points: np.ndarray,
    planes: List[PlaneResult],
    background_color: Tuple[float, float, float] = (1.0, 1.0, 1.0),
) -> None:
    """Affiche chaque plan individuellement, un par un.
    Ferme la fenetre pour passer au plan suivant.
    """
    base_colors = np.full((len(points), 3), 0.8, dtype=np.float64)

    for plane in planes:
        a, b, c = plane.normal
        d = plane.distance
        title = (
            f"Plan {plane.plane_id}  |  "
            f"normale=({a:.3f}, {b:.3f}, {c:.3f})  d={d:.3f}  |  "
            f"inliers={plane.inlier_count}  "
            f"[{plane.plane_id + 1}/{len(planes)}]  —  fermer pour continuer"
        )

        colors = base_colors.copy()
        colors[plane.inlier_indices] = plane.color

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.copy())
        pcd.colors = o3d.utility.Vector3dVector(colors)

        o3d.visualization.draw_geometries(
            [pcd],
            window_name=title,
            width=1200,
            height=800,
        )
