#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


def _parse_pcd_header(lines: List[str]) -> Dict[str, List[str]]:
    meta: Dict[str, List[str]] = {}
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if not parts:
            continue
        key = parts[0].lower()
        meta[key] = parts[1:]
    return meta


def _pcd_dtype(meta: Dict[str, List[str]]) -> np.dtype:
    fields = meta.get("fields", [])
    sizes = [int(x) for x in meta.get("size", [])]
    types = meta.get("type", [])
    counts = [int(x) for x in meta.get("count", [])] if "count" in meta else [1] * len(fields)

    if not fields or not sizes or not types:
        raise ValueError("PCD header missing FIELDS/SIZE/TYPE")
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD header length mismatch: FIELDS/SIZE/TYPE/COUNT")

    def base_dtype(t: str, s: int) -> np.dtype:
        t = t.upper()
        key = (t, int(s))
        mapping = {
            ("F", 4): np.dtype("<f4"),
            ("F", 8): np.dtype("<f8"),
            ("U", 1): np.dtype("<u1"),
            ("U", 2): np.dtype("<u2"),
            ("U", 4): np.dtype("<u4"),
            ("U", 8): np.dtype("<u8"),
            ("I", 1): np.dtype("<i1"),
            ("I", 2): np.dtype("<i2"),
            ("I", 4): np.dtype("<i4"),
            ("I", 8): np.dtype("<i8"),
        }
        if key not in mapping:
            raise ValueError(f"unsupported PCD field type/size: {key}")
        return mapping[key]

    descr = []
    for name, t, s, c in zip(fields, types, sizes, counts):
        dt = base_dtype(t, s)
        if c <= 1:
            descr.append((str(name), dt))
        else:
            descr.append((str(name), dt, (int(c),)))
    return np.dtype(descr)


def _read_pcd_points(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = path.read_bytes()
    # Header is ASCII lines terminated by "DATA <type>"
    header_lines: List[str] = []
    offset = 0
    while True:
        nl = data.find(b"\n", offset)
        if nl < 0:
            raise ValueError("invalid PCD: missing DATA line")
        line = data[offset : nl + 1].decode("utf-8", errors="replace").strip()
        header_lines.append(line)
        offset = nl + 1
        if line.upper().startswith("DATA"):
            break

    meta = _parse_pcd_header(header_lines)
    data_line = meta.get("data", [])
    if not data_line:
        raise ValueError("invalid PCD: missing DATA entry")
    data_type = data_line[0].lower()
    if data_type not in ("binary", "ascii"):
        raise ValueError(f"unsupported PCD DATA type: {data_type}")

    points = 0
    if "points" in meta and meta["points"]:
        points = int(meta["points"][0])
    elif "width" in meta and "height" in meta and meta["width"] and meta["height"]:
        points = int(meta["width"][0]) * int(meta["height"][0])
    if points <= 0:
        raise ValueError("invalid PCD: POINTS is zero/unknown")

    if data_type == "ascii":
        # Slow path: rely on numpy text parsing.
        text = data[offset:].decode("utf-8", errors="replace")
        arr = np.loadtxt(text.splitlines(), dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape((1, -1))
        # Assume x y z are first three columns in ASCII PCD.
        if arr.shape[1] < 3:
            raise ValueError("ascii PCD has fewer than 3 columns")
        return arr[:, 0], arr[:, 1], arr[:, 2]

    dtype = _pcd_dtype(meta)
    payload = data[offset:]
    arr = np.frombuffer(payload, dtype=dtype, count=points)

    # Field names can vary in case; match case-insensitively.
    name_map = {n.lower(): n for n in arr.dtype.names or ()}
    for key in ("x", "y", "z"):
        if key not in name_map:
            raise ValueError(f"binary PCD missing required field: {key}")

    x = arr[name_map["x"]].astype(np.float32, copy=False).reshape(-1)
    y = arr[name_map["y"]].astype(np.float32, copy=False).reshape(-1)
    z = arr[name_map["z"]].astype(np.float32, copy=False).reshape(-1)
    return x, y, z


def _estimate_ground_z(z: np.ndarray, bin_m: float) -> float:
    if z.size == 0:
        return 0.0
    zmin = float(np.quantile(z, 0.01))
    zmax = float(np.quantile(z, 0.99))
    if not math.isfinite(zmin) or not math.isfinite(zmax) or zmax <= zmin:
        return float(np.median(z))
    bin_m = max(1e-3, float(bin_m))
    bins = np.arange(zmin, zmax + bin_m, bin_m, dtype=np.float64)
    if bins.size < 2:
        return float(np.median(z))
    hist, edges = np.histogram(z, bins=bins)
    idx = int(np.argmax(hist))
    return float(0.5 * (edges[idx] + edges[idx + 1]))


def _compute_grid(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    *,
    resolution: float,
    bbox: Optional[Tuple[float, float, float, float]],
    margin: float,
    ground_z: Optional[float],
    ground_bin_m: float,
    ground_band_m: float,
    ground_min_hits: int,
    obstacle_min_h: float,
    obstacle_max_h: float,
    obstacle_min_hits: int,
    dilate_px: int,
    unknown_value: int,
    free_value: int,
    occupied_value: int,
) -> Tuple[np.ndarray, Dict[str, float]]:
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x = x[finite]
    y = y[finite]
    z = z[finite]
    if x.size == 0:
        raise ValueError("no finite points in input PCD")

    res = max(1e-3, float(resolution))
    margin = max(0.0, float(margin))

    gz = float(ground_z) if ground_z is not None else _estimate_ground_z(z, bin_m=ground_bin_m)
    ground_band_m = max(0.0, float(ground_band_m))
    obstacle_min_h = float(obstacle_min_h)
    obstacle_max_h = float(obstacle_max_h)
    if obstacle_max_h < obstacle_min_h:
        obstacle_min_h, obstacle_max_h = obstacle_max_h, obstacle_min_h

    ground_mask = np.abs(z - gz) <= ground_band_m
    obs_mask = (z >= gz + obstacle_min_h) & (z <= gz + obstacle_max_h)

    # bbox: (xmin, ymin, xmax, ymax)
    if bbox is None:
        # Prefer ground points to infer explored area; fall back to all points if needed.
        xx = x[ground_mask] if np.any(ground_mask) else x
        yy = y[ground_mask] if np.any(ground_mask) else y
        xmin = float(np.quantile(xx, 0.01)) - margin
        xmax = float(np.quantile(xx, 0.99)) + margin
        ymin = float(np.quantile(yy, 0.01)) - margin
        ymax = float(np.quantile(yy, 0.99)) + margin
    else:
        xmin, ymin, xmax, ymax = [float(v) for v in bbox]
        if xmax < xmin:
            xmin, xmax = xmax, xmin
        if ymax < ymin:
            ymin, ymax = ymax, ymin

    if not (math.isfinite(xmin) and math.isfinite(xmax) and math.isfinite(ymin) and math.isfinite(ymax)):
        raise ValueError("invalid bbox")

    width = int(math.ceil((xmax - xmin) / res)) + 1
    height = int(math.ceil((ymax - ymin) / res)) + 1
    if width <= 2 or height <= 2:
        raise ValueError(f"invalid map size: {width}x{height}")

    img = np.full((height, width), int(unknown_value), dtype=np.uint8)

    def to_rc(xx: np.ndarray, yy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        col = np.floor((xx - xmin) / res).astype(np.int64, copy=False)
        row_from_bottom = np.floor((yy - ymin) / res).astype(np.int64, copy=False)
        row = (height - 1) - row_from_bottom
        valid = (col >= 0) & (col < width) & (row >= 0) & (row < height)
        return row[valid], col[valid]

    # Mark free where the floor is observed.
    rows, cols = to_rc(x[ground_mask], y[ground_mask])
    if rows.size:
        flat = rows * width + cols
        counts = np.bincount(flat, minlength=width * height).reshape((height, width))
        img[counts >= int(max(1, ground_min_hits))] = int(free_value)

    # Mark occupied where obstacles are observed.
    rows, cols = to_rc(x[obs_mask], y[obs_mask])
    if rows.size:
        flat = rows * width + cols
        counts = np.bincount(flat, minlength=width * height).reshape((height, width))
        occ = counts >= int(max(1, obstacle_min_hits))
        if int(dilate_px) > 0:
            r = int(dilate_px)
            dil = occ.copy()
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx == 0 and dy == 0:
                        continue
                    src_y0 = max(0, -dy)
                    src_y1 = min(height, height - dy)
                    dst_y0 = max(0, dy)
                    dst_y1 = min(height, height + dy)
                    src_x0 = max(0, -dx)
                    src_x1 = min(width, width - dx)
                    dst_x0 = max(0, dx)
                    dst_x1 = min(width, width + dx)
                    if src_y0 >= src_y1 or src_x0 >= src_x1:
                        continue
                    dil[dst_y0:dst_y1, dst_x0:dst_x1] |= occ[src_y0:src_y1, src_x0:src_x1]
            occ = dil
        img[occ] = int(occupied_value)

    meta_out = {
        "resolution": res,
        "origin_x": xmin,
        "origin_y": ymin,
        "ground_z": gz,
        "width": float(width),
        "height": float(height),
    }
    return img, meta_out


def _write_pgm(path: Path, img: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    h, w = img.shape
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(img.tobytes(order="C"))


def _write_yaml(path: Path, image_name: str, resolution: float, origin_x: float, origin_y: float) -> None:
    content = "\n".join(
        [
            "# Auto-generated Nav2 map (OccupancyGrid) from a PCD point cloud",
            f"image: {image_name}",
            f"resolution: {resolution:.6f}",
            f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
            "",
        ]
    )
    path.write_text(content, encoding="utf-8")


def _expand_inputs(pcd_arg: str) -> List[Path]:
    # Allow passing a glob pattern (e.g., "PCD/scans_*.pcd") or a single file path.
    if any(ch in pcd_arg for ch in ["*", "?", "["]):
        return [Path(p) for p in sorted(glob.glob(pcd_arg))]
    return [Path(pcd_arg)]


def _merge_points(paths: List[Path]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    xs: List[np.ndarray] = []
    ys: List[np.ndarray] = []
    zs: List[np.ndarray] = []
    for p in paths:
        x, y, z = _read_pcd_points(p)
        xs.append(x)
        ys.append(y)
        zs.append(z)
    return np.concatenate(xs), np.concatenate(ys), np.concatenate(zs)


def _main() -> int:
    ap = argparse.ArgumentParser(description="Convert PCD (PointXYZ/PointXYZI) to a Nav2 map.yaml + map.pgm")
    ap.add_argument("--pcd", required=True, help="PCD file path or glob (e.g. 'src/point_lio/PCD/scans.pcd')")
    ap.add_argument("--out_dir", required=True, help="Output directory")
    ap.add_argument("--map_name", default="map", help="Basename for outputs (map_name.pgm/map_name.yaml)")
    ap.add_argument("--copy_pcd", action="store_true", help="Copy input PCD into out_dir as prior.pcd")
    ap.add_argument("--resolution", type=float, default=0.05, help="Map resolution (m/cell)")
    ap.add_argument("--margin", type=float, default=0.5, help="Extra margin added around auto bbox (meters)")
    ap.add_argument("--bbox", default="", help="Override bbox: xmin,ymin,xmax,ymax (meters)")
    ap.add_argument("--ground_z", type=float, default=float("nan"), help="Override ground Z (meters)")
    ap.add_argument("--ground_bin_m", type=float, default=0.05, help="Ground estimation histogram bin size (m)")
    ap.add_argument("--ground_band_m", type=float, default=0.12, help="Ground band half-width around ground_z (m)")
    ap.add_argument("--ground_min_hits", type=int, default=1, help="Min hits per cell to mark FREE")
    ap.add_argument("--obstacle_min_h", type=float, default=0.20, help="Obstacle min height above ground (m)")
    ap.add_argument("--obstacle_max_h", type=float, default=2.00, help="Obstacle max height above ground (m)")
    ap.add_argument("--obstacle_min_hits", type=int, default=3, help="Min hits per cell to mark OCCUPIED")
    ap.add_argument("--dilate_px", type=int, default=2, help="Dilate occupied cells (pixels)")
    args = ap.parse_args()

    in_paths = _expand_inputs(str(args.pcd))
    in_paths = [p for p in in_paths if p.exists()]
    if not in_paths:
        raise SystemExit(json.dumps({"ok": False, "reason": f"pcd not found: {args.pcd}"}))

    x, y, z = _merge_points(in_paths)

    bbox = None
    if str(args.bbox).strip():
        m = re.split(r"[,\s]+", str(args.bbox).strip())
        if len(m) != 4:
            raise SystemExit(json.dumps({"ok": False, "reason": "bbox must have 4 numbers: xmin,ymin,xmax,ymax"}))
        bbox = (float(m[0]), float(m[1]), float(m[2]), float(m[3]))

    ground_z = None if math.isnan(float(args.ground_z)) else float(args.ground_z)
    img, meta = _compute_grid(
        x,
        y,
        z,
        resolution=float(args.resolution),
        bbox=bbox,
        margin=float(args.margin),
        ground_z=ground_z,
        ground_bin_m=float(args.ground_bin_m),
        ground_band_m=float(args.ground_band_m),
        ground_min_hits=int(args.ground_min_hits),
        obstacle_min_h=float(args.obstacle_min_h),
        obstacle_max_h=float(args.obstacle_max_h),
        obstacle_min_hits=int(args.obstacle_min_hits),
        dilate_px=int(args.dilate_px),
        unknown_value=205,
        free_value=254,
        occupied_value=0,
    )

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = out_dir / f"{args.map_name}.pgm"
    yaml_path = out_dir / f"{args.map_name}.yaml"
    meta_path = out_dir / f"{args.map_name}.meta.json"

    _write_pgm(pgm_path, img)
    _write_yaml(
        yaml_path,
        image_name=pgm_path.name,
        resolution=float(meta["resolution"]),
        origin_x=float(meta["origin_x"]),
        origin_y=float(meta["origin_y"]),
    )
    meta_path.write_text(json.dumps({"ok": True, "inputs": [str(p) for p in in_paths], **meta}, indent=2), encoding="utf-8")

    prior_pcd_path = ""
    if args.copy_pcd:
        dst = out_dir / "prior.pcd"
        # If multiple inputs were used, keep the first path as the copied prior.
        shutil.copy2(in_paths[0], dst)
        prior_pcd_path = str(dst)

    print(
        json.dumps(
            {
                "ok": True,
                "map_yaml": str(yaml_path),
                "map_pgm": str(pgm_path),
                "meta_json": str(meta_path),
                "prior_pcd": prior_pcd_path,
                "origin": [float(meta["origin_x"]), float(meta["origin_y"]), 0.0],
                "resolution": float(meta["resolution"]),
                "size": [int(img.shape[1]), int(img.shape[0])],
                "ground_z": float(meta["ground_z"]),
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
