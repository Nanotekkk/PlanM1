from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

import numpy as np

from utility.analysis_utils import evaluate_single_plane, export_rows_csv
from utility.io_utils import load_points_from_ply
from utility.visualizer import PlaneResult, visualize_point_cloud_with_planes, visualize_planes_one_by_one

ROOT_DIR = Path(__file__).resolve().parent
JSON_DIR = ROOT_DIR / "json"
PLANS_DIR = ROOT_DIR / "plans"


def _resolve_json_path(name: str) -> Path:
    p = Path(name)
    if p.exists():
        return p
    candidate = JSON_DIR / p.name
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"Fichier JSON introuvable: {name} (cherche aussi dans json/)")


def _resolve_planes_file_path(name: str) -> Path:
    p = Path(name)
    if p.exists():
        return p
    candidate = PLANS_DIR / p.name
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"Fichier de plans introuvable: {name} (cherche aussi dans plans/)")


def _parse_plane(plane_str: str) -> np.ndarray:
    parts = [p.strip() for p in plane_str.split(",")]
    if len(parts) != 4:
        raise ValueError(
            "Le plan doit etre au format a,b,c,d (ex: 1,0,5,2)."
        )
    plane = np.array([float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])], dtype=float)
    norm = float(np.linalg.norm(plane[:3]))
    if norm < 1e-12:
        raise ValueError("Le vecteur normal (a,b,c) ne peut pas etre nul.")
    return plane


def _smooth_value(value: float, epsilon: float) -> float:
    if abs(value) <= epsilon:
        return 0.0
    rounded = round(value)
    if abs(value - rounded) <= epsilon:
        return float(rounded)
    return float(value)


def _smooth_plane(plane: np.ndarray, epsilon: float) -> np.ndarray:
    smoothed = np.array([_smooth_value(float(v), epsilon) for v in plane], dtype=float)
    if np.linalg.norm(smoothed[:3]) < 1e-12:
        # Keep original normal if smoothing made it degenerate.
        smoothed[:3] = plane[:3]
    return smoothed


def _smooth_csv_normal_component(value: float, epsilon: float, unit_value: float, decimals: int) -> float:
    if abs(value) <= epsilon:
        return 0.0
    if abs(abs(value) - 1.0) <= epsilon:
        return float(np.sign(value) * unit_value)
    return float(round(value, decimals))


def _smooth_csv_distance(value: float, epsilon: float, decimals: int) -> float:
    if abs(value) <= epsilon:
        return 0.0
    return float(round(value, decimals))


def _load_points(ply_path: Optional[str]) -> tuple[np.ndarray, str]:
    if not ply_path:
        ply_path = input("Nom du fichier .ply (dans ply_files): ").strip()
    if not ply_path:
        raise ValueError("Aucun fichier .ply fourni.")
    return load_points_from_ply(ply_path), ply_path


def _find_density_peaks(
    counts: np.ndarray, edges: np.ndarray, min_ratio: float = 0.1, min_bins_apart: int = 10
) -> list[tuple[float, int]]:
    """Retourne les maxima locaux (position, count) au-dessus du seuil."""
    threshold = int(counts.max() * min_ratio)
    results: list[tuple[float, int]] = []
    last_peak_bin = -min_bins_apart - 1
    for i in range(1, len(counts) - 1):
        if counts[i] >= threshold and counts[i] >= counts[i - 1] and counts[i] > counts[i + 1]:
            if i - last_peak_bin >= min_bins_apart:
                pos = float((edges[i] + edges[i + 1]) / 2)
                results.append((pos, int(counts[i])))
                last_peak_bin = i
            elif counts[i] > results[-1][1]:
                pos = float((edges[i] + edges[i + 1]) / 2)
                results[-1] = (pos, int(counts[i]))
                last_peak_bin = i
    return sorted(results, key=lambda x: -x[1])


def _scan_normal_cmd(points: np.ndarray, normal_str: str, n_candidates: int = 6, bins: int = 300) -> None:
    """Projette le nuage sur la normale et affiche les valeurs d candidates."""
    parts = [p.strip() for p in normal_str.split(",")]
    if len(parts) != 3:
        raise ValueError("--scan-normal attend le format a,b,c (ex: 0.789,0.614,0)")
    a, b, c = float(parts[0]), float(parts[1]), float(parts[2])
    norm_len = float(np.sqrt(a * a + b * b + c * c))
    if norm_len < 1e-12:
        raise ValueError("Le vecteur normal (a,b,c) ne peut pas etre nul.")
    a /= norm_len
    b /= norm_len
    c /= norm_len

    projections = a * points[:, 0] + b * points[:, 1] + c * points[:, 2]
    counts, edges = np.histogram(projections, bins=bins)
    peaks = _find_density_peaks(counts, edges)

    print(f"\nScan normale ({a:.4f}, {b:.4f}, {c:.4f}) normalisee:")
    print(f"Plage de projections: [{projections.min():.4f}, {projections.max():.4f}]")
    print(f"Top {min(n_candidates, len(peaks))} candidats (de la plus dense a la moins dense):")
    for i, (pos, cnt) in enumerate(peaks[:n_candidates]):
        d_val = -pos
        print(f"  #{i + 1}  d = {d_val:+.6f}   (projection = {pos:+.4f},  points = {cnt})")
    if peaks:
        best_pos, _ = peaks[0]
        print(f"\nLigne recommandee: {a:.6f},{b:.6f},{c:.6f},{-best_pos:.6f}")


def _load_plane_strings_from_file(file_path: str) -> list[str]:
    lines = _resolve_planes_file_path(file_path).read_text(encoding="utf-8").splitlines()
    planes: list[str] = []
    for line in lines:
        # Strip inline comments and surrounding whitespace
        cleaned = line.split("#")[0].strip()
        if not cleaned:
            continue
        planes.append(cleaned)
    return planes


def _load_planes_from_json(json_path: str) -> list[str]:
    """Charge les plans depuis un fichier JSON.

    Format attendu::

        {"planes": [[a, b, c, d], ...]}

    Ou bien une liste directe::

        [[a, b, c, d], ...]
    """
    data = json.loads(_resolve_json_path(json_path).read_text(encoding="utf-8"))
    if isinstance(data, dict):
        raw_planes = data.get("planes", [])
    elif isinstance(data, list):
        raw_planes = data
    else:
        raise ValueError(f"Format JSON non reconnu dans {json_path}")

    planes: list[str] = []
    for entry in raw_planes:
        if isinstance(entry, (list, tuple)) and len(entry) == 4:
            planes.append(",".join(str(v) for v in entry))
        elif isinstance(entry, dict):
            a = entry.get("a", entry.get("normal_x", 0))
            b = entry.get("b", entry.get("normal_y", 0))
            c = entry.get("c", entry.get("normal_z", 0))
            d = entry.get("d", entry.get("distance", 0))
            planes.append(f"{a},{b},{c},{d}")
        else:
            raise ValueError(f"Entree de plan JSON invalide: {entry}")
    return planes


def _round_display(value: float, eps_zero: float = 0.005, eps_int: float = 0.005) -> float:
    """Arrondit une valeur pour l'affichage: snap vers 0 et entiers proches, sinon 2 decimales."""
    if abs(value) <= eps_zero:
        return 0.0
    nearest_int = round(value)
    if abs(value - nearest_int) <= eps_int:
        return float(nearest_int)
    return round(value, 2)


def run(
    ply_path: Optional[str],
    plane_strs: list[str],
    distance_threshold: Optional[float] = None,
    metric: bool = True,
    smooth_epsilon: float = 1e-4,
    export_csv: bool = False,
    csv_path: str = "csv/checkplan_metrics.csv",
    csv_smooth_epsilon: float = 1e-4,
    csv_unit_value: float = 0.9999,
    csv_decimals: int = 4,
    visualize: bool = False,
    visualize_one_by_one: bool = False,
) -> None:
    points, used_ply_path = _load_points(ply_path)

    diag = float(np.linalg.norm(points.max(axis=0) - points.min(axis=0)) + 1e-12)
    if distance_threshold is None:
        distance_threshold = 0.01 * diag

    print(f"PLY: {used_ply_path}")
    print("method: checkplan")
    print(f"planes_count: {len(plane_strs)}")
    print(f"distance_threshold: {distance_threshold:.6f}")

    palette = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 1.0, 0.0),
        (1.0, 0.0, 1.0),
        (0.0, 1.0, 1.0),
        (1.0, 0.5, 0.0),
        (0.5, 0.0, 1.0),
    ]

    rows: list[dict[str, float | int | str]] = []
    plane_results: list[PlaneResult] = []

    for idx, plane_str in enumerate(plane_strs):
        plane_raw = _parse_plane(plane_str)
        plane = _smooth_plane(plane_raw, smooth_epsilon)

        a, b, c, d = plane
        denom = float(np.sqrt(a * a + b * b + c * c) + 1e-12)
        distances = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d) / denom
        inlier_mask = distances <= distance_threshold
        inlier_indices = np.where(inlier_mask)[0]
        metric_values = evaluate_single_plane(points, plane, inlier_mask)

        ra = _round_display(float(a))
        rb = _round_display(float(b))
        rc = _round_display(float(c))
        rd = _round_display(float(d))
        print(f"\nPlan {idx}: ({ra}, {rb}, {rc}, {rd})")
        print(f"  inlier_count: {int(np.sum(inlier_mask))}")

        if metric:
            print(f"  inlier_ratio: {metric_values['inlier_ratio']:.4f}")
            print(f"  rmse:         {metric_values['rmse']:.4f}")

        plane_results.append(
            PlaneResult(
                plane_id=idx,
                normal=np.array([a, b, c], dtype=float),
                distance=float(d),
                inlier_indices=inlier_indices,
                inlier_count=len(inlier_indices),
                color=palette[idx % len(palette)],
            )
        )

        if export_csv:
            csv_nx = _smooth_csv_normal_component(float(a), csv_smooth_epsilon, csv_unit_value, csv_decimals)
            csv_ny = _smooth_csv_normal_component(float(b), csv_smooth_epsilon, csv_unit_value, csv_decimals)
            csv_nz = _smooth_csv_normal_component(float(c), csv_smooth_epsilon, csv_unit_value, csv_decimals)
            csv_d = _smooth_csv_distance(float(d), csv_smooth_epsilon, csv_decimals)
            rows.append(
                {
                    "ply_file": used_ply_path,
                    "plane_id": idx,
                    "normal_x": csv_nx,
                    "normal_y": csv_ny,
                    "normal_z": csv_nz,
                    "distance": csv_d,
                    "distance_threshold": float(distance_threshold),
                    "inlier_count": int(np.sum(inlier_mask)),
                    "inlier_ratio": float(metric_values["inlier_ratio"]),
                    "rmse": float(metric_values["rmse"]),
                }
            )

    if export_csv:
        out = export_rows_csv(rows, csv_path)
        print(f"\ncsv: {out}")

    if visualize_one_by_one:
        visualize_planes_one_by_one(points, plane_results)
    elif visualize:
        visualize_point_cloud_with_planes(points, plane_results, "Check Plans")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Selectionne et affiche un ou plusieurs plans a partir de coefficients a,b,c,d"
    )
    parser.add_argument("ply_path", nargs="?", default=None, help="Nom ou chemin du fichier .ply")
    parser.add_argument(
        "planes",
        nargs="*",
        help="Plans au format a,b,c,d ou chemin vers un fichier .json contenant les plans",
    )
    parser.add_argument(
        "--planes-file",
        type=str,
        default=None,
        help="Fichier texte contenant un plan par ligne (format: a,b,c,d)",
    )
    parser.add_argument(
        "--distance-threshold",
        type=float,
        default=None,
        help="Seuil de distance point-plan (defaut: 1%% de la diagonale)",
    )
    parser.add_argument(
        "--smooth-epsilon",
        type=float,
        default=1e-4,
        help="Lissage des coefficients: |valeur| <= epsilon devient 0 (defaut: 1e-4)",
    )
    parser.add_argument("--no-metric", action="store_true", help="Desactive l'affichage des metriques")
    parser.add_argument("--visualize", action="store_true", help="Affiche la visualisation 3D Open3D (desactivee par defaut)")
    parser.add_argument("--visualize-one-by-one", action="store_true", help="Affiche chaque plan un par un (fermer la fenetre pour passer au suivant)")
    parser.add_argument("--export-csv", action="store_true", help="Ajoute la ligne de metriques dans un CSV")
    parser.add_argument("--csv-path", type=str, default="csv/checkplan_metrics.csv", help="Chemin du fichier CSV")
    parser.add_argument(
        "--csv-smooth-epsilon",
        type=float,
        default=1e-4,
        help="Seuil de lissage CSV pour snap vers 0 et vers +/-1",
    )
    parser.add_argument(
        "--csv-unit-value",
        type=float,
        default=0.9999,
        help="Valeur cible pour les normales proches de +/-1 dans le CSV",
    )
    parser.add_argument(
        "--csv-decimals",
        type=int,
        default=4,
        help="Nombre de decimales pour les coefficients exportes en CSV",
    )
    parser.add_argument(
        "--scan-normal",
        type=str,
        default=None,
        metavar="a,b,c",
        help="Projette le nuage sur cette normale et affiche les valeurs d candidates (ex: 0.789,0.614,0)",
    )
    return parser


if __name__ == "__main__":
    args = _build_arg_parser().parse_args(sys.argv[1:])

    if args.scan_normal:
        pts, _ = _load_points(args.ply_path if not (args.ply_path or "").lower().endswith(".json") else None)
        _scan_normal_cmd(pts, args.scan_normal)
        sys.exit(0)

    all_planes: list[str] = []
    remaining_plane_args: list[str] = []
    ply_path_override = args.ply_path

    # If ply_path is actually a JSON file, treat it as a planes file
    if ply_path_override and ply_path_override.lower().endswith(".json"):
        print(f"Plans charges depuis: {ply_path_override}")
        all_planes.extend(_load_planes_from_json(ply_path_override))
        ply_path_override = None

    for arg in (args.planes or []):
        if arg.lower().endswith(".json"):
            print(f"Plans charges depuis: {arg}")
            all_planes.extend(_load_planes_from_json(arg))
        else:
            remaining_plane_args.append(arg)

    all_planes.extend(remaining_plane_args)

    if args.planes_file:
        all_planes.extend(_load_plane_strings_from_file(args.planes_file))

    if not all_planes:
        raise ValueError("Fournis au moins un plan via argument, fichier .json ou --planes-file")

    run(
        ply_path=ply_path_override,
        plane_strs=all_planes,
        distance_threshold=args.distance_threshold,
        metric=not args.no_metric,
        smooth_epsilon=args.smooth_epsilon,
        export_csv=args.export_csv,
        csv_path=args.csv_path,
        csv_smooth_epsilon=args.csv_smooth_epsilon,
        csv_unit_value=args.csv_unit_value,
        csv_decimals=args.csv_decimals,
        visualize=args.visualize,
        visualize_one_by_one=args.visualize_one_by_one,
    )