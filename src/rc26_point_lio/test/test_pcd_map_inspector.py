#!/usr/bin/env python3

import struct
import sys
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import pcd_map_inspector as inspector  # noqa: E402


def write_png_stub(path: Path, width: int, height: int) -> None:
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + struct.pack(">I", 13)
        + b"IHDR"
        + struct.pack(">II", width, height)
        + b"\x08\x00\x00\x00\x00"
        + b"\x00\x00\x00\x00"
    )


def lzf_literal_encode(payload: bytes) -> bytes:
    encoded = bytearray()
    for start in range(0, len(payload), 32):
        chunk = payload[start:start + 32]
        encoded.append(len(chunk) - 1)
        encoded.extend(chunk)
    return bytes(encoded)


def aos_to_pcl_compressed_field_planes(aos_payload: bytes, field_sizes: list[int], points: int) -> bytes:
    point_step = sum(field_sizes)
    field_planes = bytearray(point_step * points)
    plane_offsets = []
    offset = 0
    for field_size in field_sizes:
        plane_offsets.append(offset)
        offset += field_size * points
    for point_index in range(points):
        point_base = point_index * point_step
        field_offset = 0
        for field_index, field_size in enumerate(field_sizes):
            source_start = point_base + field_offset
            source_end = source_start + field_size
            target_start = plane_offsets[field_index] + point_index * field_size
            field_planes[target_start:target_start + field_size] = aos_payload[source_start:source_end]
            field_offset += field_size
    return bytes(field_planes)


def test_ascii_pcd_bounds_and_z_filter(tmp_path: Path) -> None:
    pcd = tmp_path / "scan_ascii.pcd"
    pcd.write_text(
        "\n".join(
            [
                "# .PCD v0.7",
                "FIELDS x y z intensity",
                "SIZE 4 4 4 4",
                "TYPE F F F F",
                "COUNT 1 1 1 1",
                "WIDTH 4",
                "HEIGHT 1",
                "POINTS 4",
                "DATA ascii",
                "-1.0 2.0 0.0 10",
                "3.0 -4.0 1.0 11",
                "5.0 6.0 3.0 12",
                "nan 0.0 0.0 13",
            ]
        )
        + "\n",
        encoding="ascii",
    )

    result = inspector.analyze_pcd(pcd, z_min=0.5, z_max=2.0)

    assert result.total_records == 4
    assert result.valid_xyz_points == 3
    assert result.selected_points == 1
    assert result.skipped_nonfinite == 1
    assert result.skipped_z_filter == 2
    bounds = result.bounds.to_dict()
    assert bounds["x_min"] == pytest.approx(3.0)
    assert bounds["x_max"] == pytest.approx(3.0)
    assert bounds["y_min"] == pytest.approx(-4.0)
    assert bounds["y_max"] == pytest.approx(-4.0)


def test_binary_pcd_bounds(tmp_path: Path) -> None:
    pcd = tmp_path / "scan_binary.pcd"
    header = "\n".join(
        [
            "# .PCD v0.7",
            "FIELDS x y z intensity",
            "SIZE 4 4 4 4",
            "TYPE F F F F",
            "COUNT 1 1 1 1",
            "WIDTH 2",
            "HEIGHT 1",
            "POINTS 2",
            "DATA binary",
        ]
    ).encode("ascii") + b"\n"
    payload = struct.pack("<ffffffff", -2.0, 1.0, 0.5, 10.0, 4.0, -3.0, 1.5, 11.0)
    pcd.write_bytes(header + payload)

    result = inspector.analyze_pcd(pcd)

    assert result.total_records == 2
    assert result.selected_points == 2
    bounds = result.bounds.to_dict()
    assert bounds["x_min"] == pytest.approx(-2.0)
    assert bounds["x_max"] == pytest.approx(4.0)
    assert bounds["y_min"] == pytest.approx(-3.0)
    assert bounds["y_max"] == pytest.approx(1.0)
    assert bounds["z_min"] == pytest.approx(0.5)
    assert bounds["z_max"] == pytest.approx(1.5)


def test_binary_compressed_pcd_bounds(tmp_path: Path) -> None:
    pcd = tmp_path / "scan_binary_compressed.pcd"
    points = 2
    point_step = 16
    aos_payload = struct.pack("<ffffffff", -2.0, 1.0, 0.5, 10.0, 4.0, -3.0, 1.5, 11.0)
    field_planes_payload = aos_to_pcl_compressed_field_planes(aos_payload, field_sizes=[4, 4, 4, 4], points=points)
    compressed = lzf_literal_encode(field_planes_payload)
    header = "\n".join(
        [
            "# .PCD v0.7",
            "FIELDS x y z intensity",
            "SIZE 4 4 4 4",
            "TYPE F F F F",
            "COUNT 1 1 1 1",
            "WIDTH 2",
            "HEIGHT 1",
            "POINTS 2",
            "DATA binary_compressed",
        ]
    ).encode("ascii") + b"\n"
    pcd.write_bytes(header + struct.pack("<II", len(compressed), len(field_planes_payload)) + compressed)

    result = inspector.analyze_pcd(pcd)

    assert result.total_records == 2
    assert result.selected_points == 2
    bounds = result.bounds.to_dict()
    assert bounds["x_min"] == pytest.approx(-2.0)
    assert bounds["x_max"] == pytest.approx(4.0)
    assert bounds["y_min"] == pytest.approx(-3.0)
    assert bounds["y_max"] == pytest.approx(1.0)
    assert bounds["z_min"] == pytest.approx(0.5)
    assert bounds["z_max"] == pytest.approx(1.5)


def test_map_yaml_coverage_passes(tmp_path: Path) -> None:
    pcd = tmp_path / "scan_ascii.pcd"
    pcd.write_text(
        "\n".join(
            [
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 2",
                "HEIGHT 1",
                "POINTS 2",
                "DATA ascii",
                "-1.0 -2.0 0.0",
                "3.0 4.0 0.0",
            ]
        )
        + "\n",
        encoding="ascii",
    )
    image = tmp_path / "map.png"
    write_png_stub(image, width=10, height=14)
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text(
        "\n".join(
            [
                "image: map.png",
                "resolution: 0.5",
                "origin: [-1.0, -2.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    result = inspector.analyze_pcd(pcd)
    map_info = inspector.load_nav2_map_info(map_yaml)
    coverage = inspector.check_map_coverage(result.bounds, map_info, tolerance_m=1e-6)

    assert map_info.image_width_px == 10
    assert map_info.image_height_px == 14
    assert coverage.inside is True
    assert coverage.margins["left_x_min"] == pytest.approx(0.0)
    assert coverage.margins["bottom_y_min"] == pytest.approx(0.0)
    assert coverage.margins["right_x_max"] == pytest.approx(1.0)
    assert coverage.margins["top_y_max"] == pytest.approx(1.0)


def test_missing_xy_fields_fails(tmp_path: Path) -> None:
    pcd = tmp_path / "bad.pcd"
    pcd.write_text(
        "\n".join(
            [
                "FIELDS z intensity",
                "SIZE 4 4",
                "TYPE F F",
                "COUNT 1 1",
                "WIDTH 1",
                "HEIGHT 1",
                "POINTS 1",
                "DATA ascii",
                "0.0 1.0",
            ]
        )
        + "\n",
        encoding="ascii",
    )

    with pytest.raises(inspector.InspectorError, match="缺少 x 字段"):
        inspector.analyze_pcd(pcd)
