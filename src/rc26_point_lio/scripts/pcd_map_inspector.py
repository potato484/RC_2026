#!/usr/bin/env python3
"""
Inspect Point-LIO PCD bounds and optionally verify coverage against a Nav2 map YAML.

The tool is intentionally read-only: it reports bounds, suggested map metadata and
coverage margins, but never edits map files.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


class InspectorError(RuntimeError):
    """User-facing inspection failure."""


@dataclass(frozen=True)
class PcdHeader:
    fields: list[str]
    sizes: list[int]
    types: list[str]
    counts: list[int]
    width: int
    height: int
    points: int
    data: str
    data_offset: int
    point_step: int
    expanded_fields: list[str]
    ascii_indices: dict[str, int]
    binary_offsets: dict[str, int]


@dataclass
class BoundsAccumulator:
    x_min: float = math.inf
    x_max: float = -math.inf
    y_min: float = math.inf
    y_max: float = -math.inf
    z_min: float = math.inf
    z_max: float = -math.inf

    def update(self, x: float, y: float, z: float) -> None:
        self.x_min = min(self.x_min, x)
        self.x_max = max(self.x_max, x)
        self.y_min = min(self.y_min, y)
        self.y_max = max(self.y_max, y)
        self.z_min = min(self.z_min, z)
        self.z_max = max(self.z_max, z)

    def to_dict(self) -> dict[str, float]:
        return {
            "x_min": self.x_min,
            "x_max": self.x_max,
            "x_span": self.x_max - self.x_min,
            "y_min": self.y_min,
            "y_max": self.y_max,
            "y_span": self.y_max - self.y_min,
            "z_min": self.z_min,
            "z_max": self.z_max,
            "z_span": self.z_max - self.z_min,
        }


@dataclass(frozen=True)
class PcdAnalysis:
    path: Path
    header: PcdHeader
    total_records: int
    valid_xyz_points: int
    selected_points: int
    skipped_parse: int
    skipped_nonfinite: int
    skipped_z_filter: int
    bounds: BoundsAccumulator
    z_filter: tuple[float | None, float | None]

    def to_dict(self) -> dict[str, Any]:
        return {
            "path": str(self.path),
            "data": self.header.data,
            "fields": self.header.fields,
            "width": self.header.width,
            "height": self.header.height,
            "header_points": self.header.points,
            "point_step": self.header.point_step,
            "total_records": self.total_records,
            "valid_xyz_points": self.valid_xyz_points,
            "selected_points": self.selected_points,
            "skipped_parse": self.skipped_parse,
            "skipped_nonfinite": self.skipped_nonfinite,
            "skipped_z_filter": self.skipped_z_filter,
            "z_filter": {"min": self.z_filter[0], "max": self.z_filter[1]},
            "bounds": self.bounds.to_dict(),
        }


@dataclass(frozen=True)
class MapSuggestion:
    resolution: float
    padding_m: float
    origin: list[float]
    width_px: int
    height_px: int
    width_m: float
    height_m: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "resolution": self.resolution,
            "padding_m": self.padding_m,
            "origin": self.origin,
            "width_px": self.width_px,
            "height_px": self.height_px,
            "width_m": self.width_m,
            "height_m": self.height_m,
        }


@dataclass(frozen=True)
class Nav2MapInfo:
    yaml_path: Path
    image_path: Path
    resolution: float
    origin: list[float]
    image_width_px: int
    image_height_px: int
    image_format: str

    @property
    def yaw(self) -> float:
        return self.origin[2] if len(self.origin) >= 3 else 0.0

    @property
    def x_min(self) -> float:
        return self.origin[0]

    @property
    def y_min(self) -> float:
        return self.origin[1]

    @property
    def x_max(self) -> float:
        return self.x_min + self.image_width_px * self.resolution

    @property
    def y_max(self) -> float:
        return self.y_min + self.image_height_px * self.resolution

    def to_dict(self) -> dict[str, Any]:
        return {
            "yaml_path": str(self.yaml_path),
            "image_path": str(self.image_path),
            "resolution": self.resolution,
            "origin": self.origin,
            "image_width_px": self.image_width_px,
            "image_height_px": self.image_height_px,
            "image_format": self.image_format,
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
            "yaw": self.yaw,
        }


@dataclass(frozen=True)
class CoverageCheck:
    supported: bool
    inside: bool
    tolerance_m: float
    margins: dict[str, float]
    warnings: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "supported": self.supported,
            "inside": self.inside,
            "tolerance_m": self.tolerance_m,
            "margins": self.margins,
            "warnings": self.warnings,
        }


def _parse_required_int(values: dict[str, str], key: str) -> int:
    if key not in values:
        raise InspectorError(f"PCD header 缺少 {key}")
    try:
        return int(values[key].split()[0])
    except (ValueError, IndexError) as exc:
        raise InspectorError(f"PCD header {key} 不是合法整数: {values[key]!r}") from exc


def _parse_int_list(values: dict[str, str], key: str) -> list[int]:
    if key not in values:
        raise InspectorError(f"PCD header 缺少 {key}")
    try:
        parsed = [int(value) for value in values[key].split()]
    except ValueError as exc:
        raise InspectorError(f"PCD header {key} 包含非法整数: {values[key]!r}") from exc
    if not parsed:
        raise InspectorError(f"PCD header {key} 不能为空")
    return parsed


def _parse_str_list(values: dict[str, str], key: str) -> list[str]:
    if key not in values:
        raise InspectorError(f"PCD header 缺少 {key}")
    parsed = values[key].split()
    if not parsed:
        raise InspectorError(f"PCD header {key} 不能为空")
    return parsed


def parse_pcd_header(path: Path) -> PcdHeader:
    values: dict[str, str] = {}
    data_offset: int | None = None

    with path.open("rb") as stream:
        while True:
            raw = stream.readline()
            if not raw:
                break
            try:
                line = raw.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise InspectorError("PCD header 不是 ASCII 文本，无法解析") from exc
            if not line or line.startswith("#"):
                continue
            parts = line.split(None, 1)
            key = parts[0].upper()
            value = parts[1].strip() if len(parts) > 1 else ""
            values[key] = value
            if key == "DATA":
                data_offset = stream.tell()
                break

    if data_offset is None:
        raise InspectorError("PCD header 缺少 DATA 行")

    fields = _parse_str_list(values, "FIELDS")
    sizes = _parse_int_list(values, "SIZE")
    types = [value.upper() for value in _parse_str_list(values, "TYPE")]
    counts = [1] * len(fields)
    if "COUNT" in values:
        counts = _parse_int_list(values, "COUNT")

    if len(sizes) != len(fields) or len(types) != len(fields) or len(counts) != len(fields):
        raise InspectorError("PCD header 中 FIELDS/SIZE/TYPE/COUNT 长度不一致")
    if any(size <= 0 for size in sizes):
        raise InspectorError("PCD header SIZE 必须全部为正数")
    if any(count <= 0 for count in counts):
        raise InspectorError("PCD header COUNT 必须全部为正数")

    width = _parse_required_int(values, "WIDTH")
    height = _parse_required_int(values, "HEIGHT")
    points = int(values["POINTS"].split()[0]) if "POINTS" in values else width * height
    if width < 0 or height < 0 or points < 0:
        raise InspectorError("PCD header WIDTH/HEIGHT/POINTS 不能为负数")

    data = values["DATA"].lower()
    if data not in ("ascii", "binary", "binary_compressed"):
        raise InspectorError(f"不支持的 PCD DATA 类型: {data}")

    expanded_fields: list[str] = []
    ascii_indices: dict[str, int] = {}
    binary_offsets: dict[str, int] = {}
    point_step = 0
    ascii_index = 0
    for field, size, count in zip(fields, sizes, counts):
        if field not in ascii_indices:
            ascii_indices[field] = ascii_index
            binary_offsets[field] = point_step
        for item_index in range(count):
            expanded_fields.append(field if count == 1 else f"{field}_{item_index}")
        ascii_index += count
        point_step += size * count

    for axis in ("x", "y", "z"):
        if axis not in ascii_indices:
            raise InspectorError(f"PCD 缺少 {axis} 字段")

    return PcdHeader(
        fields=fields,
        sizes=sizes,
        types=types,
        counts=counts,
        width=width,
        height=height,
        points=points,
        data=data,
        data_offset=data_offset,
        point_step=point_step,
        expanded_fields=expanded_fields,
        ascii_indices=ascii_indices,
        binary_offsets=binary_offsets,
    )


def _axis_binary_reader(header: PcdHeader, axis: str) -> tuple[int, str]:
    field_index = header.fields.index(axis)
    type_code = header.types[field_index]
    size = header.sizes[field_index]
    offset = header.binary_offsets[axis]

    if type_code == "F" and size == 4:
        return offset, "<f"
    if type_code == "F" and size == 8:
        return offset, "<d"
    if type_code == "I" and size == 1:
        return offset, "<b"
    if type_code == "I" and size == 2:
        return offset, "<h"
    if type_code == "I" and size == 4:
        return offset, "<i"
    if type_code == "I" and size == 8:
        return offset, "<q"
    if type_code == "U" and size == 1:
        return offset, "<B"
    if type_code == "U" and size == 2:
        return offset, "<H"
    if type_code == "U" and size == 4:
        return offset, "<I"
    if type_code == "U" and size == 8:
        return offset, "<Q"
    raise InspectorError(f"字段 {axis} 使用了暂不支持的二进制类型: TYPE={type_code}, SIZE={size}")


def _lzf_decompress(payload: bytes, expected_size: int) -> bytes:
    """Decompress the LZF payload used by PCL PCD binary_compressed files."""
    out = bytearray()
    index = 0
    while index < len(payload):
        control = payload[index]
        index += 1
        if control < 32:
            length = control + 1
            if index + length > len(payload):
                raise InspectorError("PCD binary_compressed LZF literal 越界")
            out.extend(payload[index:index + length])
            index += length
            continue

        length = control >> 5
        ref_offset = (control & 0x1F) << 8
        if length == 7:
            if index >= len(payload):
                raise InspectorError("PCD binary_compressed LZF length 越界")
            length += payload[index]
            index += 1
        if index >= len(payload):
            raise InspectorError("PCD binary_compressed LZF offset 越界")
        ref_offset += payload[index]
        index += 1

        ref_index = len(out) - ref_offset - 1
        if ref_index < 0:
            raise InspectorError("PCD binary_compressed LZF 引用非法")
        for _ in range(length + 2):
            out.append(out[ref_index])
            ref_index += 1

    if len(out) != expected_size:
        raise InspectorError(f"PCD binary_compressed 解压长度异常: 期望 {expected_size} bytes，实际 {len(out)} bytes")
    return bytes(out)


def _read_binary_compressed_field_planes(path: Path, header: PcdHeader) -> bytes:
    expected_size = header.points * header.point_step
    with path.open("rb") as stream:
        stream.seek(header.data_offset)
        size_header = stream.read(8)
        if len(size_header) != 8:
            raise InspectorError("PCD binary_compressed 缺少压缩尺寸头")
        compressed_size, uncompressed_size = struct.unpack("<II", size_header)
        if uncompressed_size != expected_size:
            raise InspectorError(
                f"PCD binary_compressed 解压尺寸与 header 不匹配: header={expected_size}, data={uncompressed_size}"
            )
        compressed_payload = stream.read(compressed_size)
    if len(compressed_payload) != compressed_size:
        raise InspectorError(
            f"PCD binary_compressed 数据长度不足: 需要 {compressed_size} bytes，实际 {len(compressed_payload)} bytes"
        )
    return _lzf_decompress(compressed_payload, uncompressed_size)


def _extract_axis_from_compressed_field_planes(payload: bytes, header: PcdHeader, axis: str) -> tuple[bytearray, str]:
    field_index = header.fields.index(axis)
    size = header.sizes[field_index]
    count = header.counts[field_index]
    _, fmt = _axis_binary_reader(header, axis)
    field_sizes = [field_size * field_count for field_size, field_count in zip(header.sizes, header.counts)]
    field_plane_start = sum(field_sizes[:field_index]) * header.points
    field_size = field_sizes[field_index]

    if count == 1:
        axis_payload = bytearray(payload[field_plane_start:field_plane_start + header.points * size])
    else:
        axis_payload = bytearray(header.points * size)
        for point_index in range(header.points):
            source_start = field_plane_start + point_index * field_size
            source_end = source_start + size
            target_start = point_index * size
            axis_payload[target_start:target_start + size] = payload[source_start:source_end]
    return axis_payload, fmt


def _passes_filter(x: float, y: float, z: float, z_min: float | None, z_max: float | None) -> bool:
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return False
    if z_min is not None and z < z_min:
        return False
    if z_max is not None and z > z_max:
        return False
    return True


def analyze_pcd(path: Path, *, z_min: float | None = None, z_max: float | None = None) -> PcdAnalysis:
    if not path.exists():
        raise InspectorError(f"PCD 文件不存在: {path}")
    if z_min is not None and z_max is not None and z_min > z_max:
        raise InspectorError("--z-min 不能大于 --z-max")

    header = parse_pcd_header(path)
    bounds = BoundsAccumulator()
    total_records = 0
    valid_xyz_points = 0
    selected_points = 0
    skipped_parse = 0
    skipped_nonfinite = 0
    skipped_z_filter = 0

    if header.data == "ascii":
        x_index = header.ascii_indices["x"]
        y_index = header.ascii_indices["y"]
        z_index = header.ascii_indices["z"]
        min_values = max(x_index, y_index, z_index) + 1
        with path.open("rb") as stream:
            stream.seek(header.data_offset)
            for raw in stream:
                stripped = raw.strip()
                if not stripped or stripped.startswith(b"#"):
                    continue
                total_records += 1
                parts = stripped.split()
                if len(parts) < min_values:
                    skipped_parse += 1
                    continue
                try:
                    x = float(parts[x_index])
                    y = float(parts[y_index])
                    z = float(parts[z_index])
                except ValueError:
                    skipped_parse += 1
                    continue

                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    skipped_nonfinite += 1
                    continue
                valid_xyz_points += 1
                if not _passes_filter(x, y, z, z_min, z_max):
                    skipped_z_filter += 1
                    continue
                bounds.update(x, y, z)
                selected_points += 1
    elif header.data == "binary":
        readers = {
            axis: _axis_binary_reader(header, axis)
            for axis in ("x", "y", "z")
        }
        expected_bytes = header.points * header.point_step
        with path.open("rb") as stream:
            stream.seek(header.data_offset)
            payload = stream.read(expected_bytes)
        if len(payload) < expected_bytes:
            raise InspectorError(
                f"PCD binary 数据长度不足: 需要 {expected_bytes} bytes，实际 {len(payload)} bytes"
            )

        for point_index in range(header.points):
            total_records += 1
            base = point_index * header.point_step
            try:
                values = {
                    axis: float(struct.unpack_from(fmt, payload, base + offset)[0])
                    for axis, (offset, fmt) in readers.items()
                }
            except struct.error:
                skipped_parse += 1
                continue
            x = values["x"]
            y = values["y"]
            z = values["z"]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                skipped_nonfinite += 1
                continue
            valid_xyz_points += 1
            if not _passes_filter(x, y, z, z_min, z_max):
                skipped_z_filter += 1
                continue
            bounds.update(x, y, z)
            selected_points += 1
    else:
        payload = _read_binary_compressed_field_planes(path, header)
        x_payload, x_fmt = _extract_axis_from_compressed_field_planes(payload, header, "x")
        y_payload, y_fmt = _extract_axis_from_compressed_field_planes(payload, header, "y")
        z_payload, z_fmt = _extract_axis_from_compressed_field_planes(payload, header, "z")
        x_iter = struct.iter_unpack(x_fmt, x_payload)
        y_iter = struct.iter_unpack(y_fmt, y_payload)
        z_iter = struct.iter_unpack(z_fmt, z_payload)

        for (x_raw,), (y_raw,), (z_raw,) in zip(x_iter, y_iter, z_iter):
            total_records += 1
            x = float(x_raw)
            y = float(y_raw)
            z = float(z_raw)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                skipped_nonfinite += 1
                continue
            valid_xyz_points += 1
            if not _passes_filter(x, y, z, z_min, z_max):
                skipped_z_filter += 1
                continue
            bounds.update(x, y, z)
            selected_points += 1

    if selected_points == 0:
        raise InspectorError("PCD 没有可用于统计的有效点；请检查字段、数据和 z 过滤条件")

    return PcdAnalysis(
        path=path,
        header=header,
        total_records=total_records,
        valid_xyz_points=valid_xyz_points,
        selected_points=selected_points,
        skipped_parse=skipped_parse,
        skipped_nonfinite=skipped_nonfinite,
        skipped_z_filter=skipped_z_filter,
        bounds=bounds,
        z_filter=(z_min, z_max),
    )


def make_map_suggestion(bounds: BoundsAccumulator, *, resolution: float, padding_m: float) -> MapSuggestion:
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise InspectorError("--resolution 必须是正有限数")
    if padding_m < 0.0 or not math.isfinite(padding_m):
        raise InspectorError("--padding-m 必须是非负有限数")

    x_min = bounds.x_min - padding_m
    y_min = bounds.y_min - padding_m
    x_span = (bounds.x_max - bounds.x_min) + padding_m * 2.0
    y_span = (bounds.y_max - bounds.y_min) + padding_m * 2.0
    width_px = max(1, math.ceil(x_span / resolution))
    height_px = max(1, math.ceil(y_span / resolution))
    return MapSuggestion(
        resolution=resolution,
        padding_m=padding_m,
        origin=[x_min, y_min, 0.0],
        width_px=width_px,
        height_px=height_px,
        width_m=width_px * resolution,
        height_m=height_px * resolution,
    )


def _strip_yaml_comment(line: str) -> str:
    in_single = False
    in_double = False
    escaped = False
    for index, char in enumerate(line):
        if escaped:
            escaped = False
            continue
        if char == "\\" and in_double:
            escaped = True
            continue
        if char == "'" and not in_double:
            in_single = not in_single
            continue
        if char == '"' and not in_single:
            in_double = not in_double
            continue
        if char == "#" and not in_single and not in_double:
            return line[:index]
    return line


def _parse_scalar(value: str) -> Any:
    value = value.strip()
    if not value:
        return ""
    if value[0] in ("'", '"'):
        try:
            return ast.literal_eval(value)
        except (ValueError, SyntaxError):
            return value.strip("'\"")
    if value.startswith("[") or value.startswith("("):
        try:
            return ast.literal_eval(value)
        except (ValueError, SyntaxError) as exc:
            raise InspectorError(f"YAML 列表字段无法解析: {value}") from exc
    try:
        if any(ch in value.lower() for ch in (".", "e")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def parse_nav2_map_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise InspectorError(f"map YAML 不存在: {path}")
    parsed: dict[str, Any] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = _strip_yaml_comment(raw_line).strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        parsed[key.strip()] = _parse_scalar(value)
    return parsed


def _read_png_size(path: Path) -> tuple[int, int]:
    data = path.read_bytes()[:32]
    if len(data) < 24 or data[:8] != b"\x89PNG\r\n\x1a\n" or data[12:16] != b"IHDR":
        raise InspectorError(f"PNG 文件头非法: {path}")
    width, height = struct.unpack(">II", data[16:24])
    return int(width), int(height)


def _pnm_tokens(data: bytes) -> Iterable[bytes]:
    index = 0
    length = len(data)
    while index < length:
        while index < length and data[index:index + 1].isspace():
            index += 1
        if index >= length:
            return
        if data[index:index + 1] == b"#":
            while index < length and data[index:index + 1] not in (b"\n", b"\r"):
                index += 1
            continue
        start = index
        while index < length and not data[index:index + 1].isspace():
            index += 1
        yield data[start:index]


def _read_pnm_size(path: Path) -> tuple[int, int]:
    tokens = list(_pnm_tokens(path.read_bytes()[:4096]))
    if len(tokens) < 3 or tokens[0] not in (b"P2", b"P3", b"P5", b"P6"):
        raise InspectorError(f"PGM/PPM 文件头非法: {path}")
    try:
        return int(tokens[1]), int(tokens[2])
    except ValueError as exc:
        raise InspectorError(f"PGM/PPM 图片尺寸非法: {path}") from exc


def read_image_size(path: Path) -> tuple[int, int, str]:
    if not path.exists():
        raise InspectorError(f"map image 不存在: {path}")
    suffix = path.suffix.lower()
    if suffix == ".png":
        width, height = _read_png_size(path)
        return width, height, "png"
    if suffix in (".pgm", ".ppm"):
        width, height = _read_pnm_size(path)
        return width, height, suffix[1:]
    raise InspectorError(f"暂不支持的 map image 格式: {path.suffix or '<none>'}")


def load_nav2_map_info(path: Path) -> Nav2MapInfo:
    data = parse_nav2_map_yaml(path)
    for key in ("image", "resolution", "origin"):
        if key not in data:
            raise InspectorError(f"map YAML 缺少字段: {key}")

    image_value = str(data["image"])
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = path.parent / image_path

    try:
        resolution = float(data["resolution"])
    except (TypeError, ValueError) as exc:
        raise InspectorError("map YAML resolution 不是合法数字") from exc
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise InspectorError("map YAML resolution 必须是正有限数")

    origin_value = data["origin"]
    if not isinstance(origin_value, (list, tuple)) or len(origin_value) < 2:
        raise InspectorError("map YAML origin 必须是至少包含 x/y 的数组")
    try:
        origin = [float(origin_value[0]), float(origin_value[1]), float(origin_value[2] if len(origin_value) >= 3 else 0.0)]
    except (TypeError, ValueError) as exc:
        raise InspectorError("map YAML origin 包含非法数字") from exc
    if not all(math.isfinite(value) for value in origin):
        raise InspectorError("map YAML origin 必须全部是有限数")

    image_width_px, image_height_px, image_format = read_image_size(image_path)
    return Nav2MapInfo(
        yaml_path=path,
        image_path=image_path,
        resolution=resolution,
        origin=origin,
        image_width_px=image_width_px,
        image_height_px=image_height_px,
        image_format=image_format,
    )


def check_map_coverage(bounds: BoundsAccumulator, map_info: Nav2MapInfo, *, tolerance_m: float) -> CoverageCheck:
    if tolerance_m < 0.0 or not math.isfinite(tolerance_m):
        raise InspectorError("--tolerance-m 必须是非负有限数")
    warnings: list[str] = []
    supported = True
    if abs(map_info.yaw) > 1e-9:
        supported = False
        warnings.append("map YAML origin yaw 非 0；当前只对轴对齐 Nav2 map 做覆盖校验")

    margins = {
        "left_x_min": bounds.x_min - map_info.x_min,
        "right_x_max": map_info.x_max - bounds.x_max,
        "bottom_y_min": bounds.y_min - map_info.y_min,
        "top_y_max": map_info.y_max - bounds.y_max,
    }
    inside = supported and all(value >= -tolerance_m for value in margins.values())
    return CoverageCheck(
        supported=supported,
        inside=inside,
        tolerance_m=tolerance_m,
        margins=margins,
        warnings=warnings,
    )


def _fmt(value: float) -> str:
    return f"{value:.6f}"


def _format_filter(z_filter: tuple[float | None, float | None]) -> str:
    z_min, z_max = z_filter
    if z_min is None and z_max is None:
        return "未启用"
    return f"[{z_min if z_min is not None else '-inf'}, {z_max if z_max is not None else '+inf'}]"


def print_report(
    analysis: PcdAnalysis,
    suggestion: MapSuggestion,
    map_info: Nav2MapInfo | None,
    coverage: CoverageCheck | None,
) -> None:
    bounds = analysis.bounds.to_dict()
    print("PCD 地图检查报告")
    print("=" * 72)
    print(f"PCD 文件: {analysis.path}")
    print(f"DATA 类型: {analysis.header.data}")
    print(f"header 点数: {analysis.header.points}")
    print(f"数据记录: {analysis.total_records}")
    print(f"有效 XYZ 点: {analysis.valid_xyz_points}")
    print(f"参与统计点: {analysis.selected_points}")
    print(f"跳过: parse={analysis.skipped_parse}, nonfinite={analysis.skipped_nonfinite}, z_filter={analysis.skipped_z_filter}")
    print(f"Z 过滤: {_format_filter(analysis.z_filter)}")
    print()
    print("PCD 边界")
    print(f"  x: [{_fmt(bounds['x_min'])}, {_fmt(bounds['x_max'])}], span={_fmt(bounds['x_span'])} m")
    print(f"  y: [{_fmt(bounds['y_min'])}, {_fmt(bounds['y_max'])}], span={_fmt(bounds['y_span'])} m")
    print(f"  z: [{_fmt(bounds['z_min'])}, {_fmt(bounds['z_max'])}], span={_fmt(bounds['z_span'])} m")
    print()
    print("推荐 Nav2 map 参数")
    print(f"  resolution: {_fmt(suggestion.resolution)} m/px")
    print(f"  padding: {_fmt(suggestion.padding_m)} m")
    print(f"  origin: [{_fmt(suggestion.origin[0])}, {_fmt(suggestion.origin[1])}, {_fmt(suggestion.origin[2])}]")
    print(f"  image size: {suggestion.width_px} x {suggestion.height_px} px")
    print(f"  physical size: {_fmt(suggestion.width_m)} x {_fmt(suggestion.height_m)} m")

    if map_info is None or coverage is None:
        return

    print()
    print("Nav2 map YAML 校验")
    print(f"  YAML: {map_info.yaml_path}")
    print(f"  image: {map_info.image_path} ({map_info.image_width_px} x {map_info.image_height_px}, {map_info.image_format})")
    print(f"  resolution: {_fmt(map_info.resolution)} m/px")
    print(f"  origin: [{_fmt(map_info.origin[0])}, {_fmt(map_info.origin[1])}, {_fmt(map_info.origin[2])}]")
    print(f"  coverage x: [{_fmt(map_info.x_min)}, {_fmt(map_info.x_max)}]")
    print(f"  coverage y: [{_fmt(map_info.y_min)}, {_fmt(map_info.y_max)}]")
    print("  margins:")
    print(f"    left_x_min:  {_fmt(coverage.margins['left_x_min'])} m")
    print(f"    right_x_max: {_fmt(coverage.margins['right_x_max'])} m")
    print(f"    bottom_y_min:{_fmt(coverage.margins['bottom_y_min'])} m")
    print(f"    top_y_max:   {_fmt(coverage.margins['top_y_max'])} m")
    for warning in coverage.warnings:
        print(f"  warning: {warning}")
    print(f"  result: {'PASS' if coverage.inside else 'FAIL'}")


def build_json_report(
    analysis: PcdAnalysis,
    suggestion: MapSuggestion,
    map_info: Nav2MapInfo | None,
    coverage: CoverageCheck | None,
) -> dict[str, Any]:
    report: dict[str, Any] = {
        "pcd": analysis.to_dict(),
        "suggested_map": suggestion.to_dict(),
    }
    if map_info is not None:
        report["map_yaml"] = map_info.to_dict()
    if coverage is not None:
        report["coverage"] = coverage.to_dict()
    return report


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="解析 Point-LIO PCD 边界，并可校验 Nav2 map YAML 是否覆盖点云范围。"
    )
    parser.add_argument("pcd_file", type=Path, help="待检查的 PCD 文件路径")
    parser.add_argument("--map-yaml", type=Path, help="可选 Nav2 map YAML，用于校验 image/resolution/origin 覆盖范围")
    parser.add_argument("--resolution", type=float, default=0.05, help="推荐地图尺寸使用的分辨率，默认 0.05 m/px")
    parser.add_argument("--padding-m", type=float, default=0.0, help="推荐 origin/尺寸时加入的边界留白，默认 0")
    parser.add_argument("--z-min", type=float, help="只统计 z >= 该值的点")
    parser.add_argument("--z-max", type=float, help="只统计 z <= 该值的点")
    parser.add_argument("--tolerance-m", type=float, default=1e-6, help="map 覆盖校验允许的边界误差，默认 1e-6 m")
    parser.add_argument("--json", action="store_true", help="输出 JSON 报告")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        analysis = analyze_pcd(args.pcd_file, z_min=args.z_min, z_max=args.z_max)
        suggestion = make_map_suggestion(analysis.bounds, resolution=args.resolution, padding_m=args.padding_m)
        map_info = load_nav2_map_info(args.map_yaml) if args.map_yaml else None
        coverage = check_map_coverage(analysis.bounds, map_info, tolerance_m=args.tolerance_m) if map_info else None

        if args.json:
            print(json.dumps(build_json_report(analysis, suggestion, map_info, coverage), ensure_ascii=False, indent=2))
        else:
            print_report(analysis, suggestion, map_info, coverage)
        return 0 if coverage is None or coverage.inside else 2
    except InspectorError as exc:
        if args.json:
            print(json.dumps({"error": str(exc)}, ensure_ascii=False, indent=2), file=sys.stderr)
        else:
            print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
