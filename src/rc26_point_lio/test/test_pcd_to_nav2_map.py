#!/usr/bin/env python3

import struct
import sys
import zlib
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import pcd_map_inspector as inspector  # noqa: E402
import pcd_to_nav2_map as converter  # noqa: E402


def write_ascii_pcd(path: Path, points: list[tuple[float, float, float]]) -> None:
    lines = [
        "# .PCD v0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]
    lines.extend(f"{x} {y} {z}" for x, y, z in points)
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def read_pgm_pixels(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    index = 0

    def next_token() -> bytes:
        nonlocal index
        while index < len(data):
            if data[index:index + 1].isspace():
                index += 1
                continue
            if data[index:index + 1] == b"#":
                while index < len(data) and data[index:index + 1] not in (b"\n", b"\r"):
                    index += 1
                continue
            break
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        return data[start:index]

    magic = next_token()
    width = int(next_token())
    height = int(next_token())
    max_value = int(next_token())
    while index < len(data) and data[index:index + 1].isspace():
        index += 1

    assert magic == b"P5"
    assert max_value == 255
    pixels = data[index:]
    assert len(pixels) == width * height
    return width, height, pixels


def read_png_pixels(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    index = 8
    width = 0
    height = 0
    idat_payload = bytearray()

    while index < len(data):
        length = struct.unpack(">I", data[index:index + 4])[0]
        chunk_type = data[index + 4:index + 8]
        payload = data[index + 8:index + 8 + length]
        index += 12 + length
        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, compression, filter_method, interlace = struct.unpack(
                ">IIBBBBB", payload
            )
            assert bit_depth == 8
            assert color_type == 0
            assert compression == 0
            assert filter_method == 0
            assert interlace == 0
        elif chunk_type == b"IDAT":
            idat_payload.extend(payload)
        elif chunk_type == b"IEND":
            break

    raw = zlib.decompress(bytes(idat_payload))
    pixels = bytearray()
    row_step = width + 1
    assert len(raw) == height * row_step
    for row in range(height):
        row_start = row * row_step
        assert raw[row_start] == 0
        pixels.extend(raw[row_start + 1:row_start + 1 + width])
    return width, height, bytes(pixels)


def test_conversion_filters_floor_ceiling_and_flips_y_axis(tmp_path: Path) -> None:
    pcd = tmp_path / "scan.pcd"
    write_ascii_pcd(
        pcd,
        [
            (0.1, 0.1, 0.50),
            (0.2, 0.2, 0.60),
            (0.3, 0.3, 0.70),
            (1.2, 1.2, 0.80),
            (1.3, 1.3, 0.90),
            (1.4, 1.4, 1.00),
            (1.2, 0.1, 0.00),
            (0.1, 1.2, 2.50),
        ],
    )

    result = converter.convert_pcd_to_nav2_map(
        pcd,
        tmp_path / "map",
        resolution=1.0,
        z_min=0.05,
        z_max=2.0,
        min_points_per_cell=3,
    )

    assert result.analysis.selected_points == 6
    assert result.analysis.skipped_z_filter == 2
    assert result.occupied_cells == 2

    assert result.image_format == "png"
    assert result.image_path.name == "map.png"
    width, height, pixels = read_png_pixels(result.image_path)
    assert (width, height) == (2, 2)
    assert pixels == bytes(
        [
            converter.FREE_PIXEL,
            converter.OCCUPIED_PIXEL,
            converter.OCCUPIED_PIXEL,
            converter.FREE_PIXEL,
        ]
    )


def test_min_points_per_cell_is_configurable(tmp_path: Path) -> None:
    pcd = tmp_path / "scan.pcd"
    write_ascii_pcd(
        pcd,
        [
            (0.0, 0.0, 0.50),
            (0.1, 0.1, 0.60),
        ],
    )

    occupied = converter.convert_pcd_to_nav2_map(
        pcd,
        tmp_path / "occupied",
        resolution=1.0,
        z_min=0.05,
        z_max=2.0,
        min_points_per_cell=2,
    )
    free = converter.convert_pcd_to_nav2_map(
        pcd,
        tmp_path / "free",
        resolution=1.0,
        z_min=0.05,
        z_max=2.0,
        min_points_per_cell=3,
    )

    assert occupied.occupied_cells == 1
    assert free.occupied_cells == 0


def test_yaml_fields_and_overwrite_guard(tmp_path: Path) -> None:
    pcd = tmp_path / "scan.pcd"
    write_ascii_pcd(
        pcd,
        [
            (0.1, 0.1, 0.50),
            (0.2, 0.2, 0.60),
            (0.3, 0.3, 0.70),
        ],
    )
    prefix = tmp_path / "map"

    result = converter.convert_pcd_to_nav2_map(
        pcd,
        prefix,
        resolution=0.5,
        z_min=0.05,
        z_max=2.0,
        min_points_per_cell=1,
    )

    yaml_data = inspector.parse_nav2_map_yaml(result.yaml_path)
    assert yaml_data["image"] == "map.png"
    assert yaml_data["resolution"] == pytest.approx(0.5)
    assert yaml_data["origin"] == pytest.approx([0.1, 0.1, 0.0])
    assert yaml_data["negate"] == 0
    assert yaml_data["occupied_thresh"] == pytest.approx(0.65)
    assert yaml_data["free_thresh"] == pytest.approx(0.196)

    with pytest.raises(converter.MapConversionError, match="--overwrite"):
        converter.convert_pcd_to_nav2_map(
            pcd,
            prefix,
            resolution=0.5,
            z_min=0.05,
            z_max=2.0,
            min_points_per_cell=1,
        )


def test_explicit_pgm_output_is_still_supported(tmp_path: Path) -> None:
    pcd = tmp_path / "scan.pcd"
    write_ascii_pcd(
        pcd,
        [
            (0.1, 0.1, 0.50),
            (0.2, 0.2, 0.60),
            (0.3, 0.3, 0.70),
        ],
    )

    result = converter.convert_pcd_to_nav2_map(
        pcd,
        tmp_path / "map",
        resolution=1.0,
        z_min=0.05,
        z_max=2.0,
        min_points_per_cell=3,
        image_format="pgm",
    )

    assert result.image_format == "pgm"
    assert result.image_path.name == "map.pgm"
    width, height, pixels = read_pgm_pixels(result.image_path)
    assert (width, height) == (1, 1)
    assert pixels == bytes([converter.OCCUPIED_PIXEL])
    yaml_data = inspector.parse_nav2_map_yaml(result.yaml_path)
    assert yaml_data["image"] == "map.pgm"
