#!/usr/bin/env python3
"""
================================================================================
                    数据集 RGB 均值/标准差/方差 计算工具
================================================================================

功能说明:
    计算图片数据集的 RGB 通道均值 (Mean)、标准差 (Std) 与方差 (Var)，用于:
    - ONNX 模型平台量化的预处理参数配置
    - 深度学习模型训练时的数据归一化
    - 图像预处理流水线参数设定

算法特点:
    - 统计量按 (sum, sum_sq) 聚合，支持多进程并行
    - 多进程并行加速，充分利用多核 CPU
    - 支持递归遍历子目录

支持格式:
    .jpg, .jpeg, .png, .bmp, .webp, .tiff, .tif

================================================================================
                                使用方法
================================================================================

1. 基本用法 (使用默认数据集路径):
   python3 calc_dataset_stats.py

2. 指定数据集路径:
   python3 calc_dataset_stats.py --data_dir /path/to/images

3. 指定并行进程数 (默认自动检测):
   python3 calc_dataset_stats.py --workers 8

4. 指定输出文件路径 (默认: 脚本目录/dataset_stats.md):
   python3 calc_dataset_stats.py --output /path/to/output.md

5. 组合使用:
   python3 calc_dataset_stats.py --data_dir /path/to/images --workers 8 --output result.md

================================================================================
                                输出说明
================================================================================

输出文件包含两种格式的统计结果:

1. 归一化格式 [0, 1]:
    - 适用于: PyTorch, TensorFlow 等框架的标准归一化
    - 使用方式: (pixel / 255.0 - mean) / std

2. 原始像素格式 [0, 255]:
    - 适用于: 某些量化平台直接使用的参数格式
    - 使用方式: (pixel - mean) / std

补充说明:
    - 不同平台 UI 中“方差”的含义可能不同:
      - 有些 UI 把除数(Std/Scale)标成“方差”
      - 也有 UI 真的要求 Var (= Std^2)
    - 本脚本会在报告中同时给出 Std 和 Var，并给出 RGB/BGR 两种顺序，方便直接粘贴。

================================================================================
"""

import os
import sys
import argparse
from pathlib import Path
from datetime import datetime
from multiprocessing import Pool, cpu_count
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
from tqdm import tqdm

IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tiff', '.tif'}


def get_image_paths(root_dir: str) -> list:
    """递归获取目录下所有图片路径"""
    paths = []
    for root, _, files in os.walk(root_dir):
        for f in files:
            if Path(f).suffix.lower() in IMAGE_EXTENSIONS:
                paths.append(os.path.join(root, f))
    return paths


def compute_image_stats(img_path: str) -> tuple:
    """计算单张图片的像素和与像素平方和 (用于并行)"""
    img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    if img is None:
        return None
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float64) / 255.0
    pixel_count = img.shape[0] * img.shape[1]
    channel_sum = img.sum(axis=(0, 1))
    channel_sq_sum = (img ** 2).sum(axis=(0, 1))
    return pixel_count, channel_sum, channel_sq_sum


def _as_list(x: np.ndarray, fmt: str) -> str:
    return "[" + ", ".join(format(float(v), fmt) for v in x.tolist()) + "]"


def generate_markdown_report(data_dir, total_images, total_pixels, failed,
                              mean_rgb, std_rgb, var_rgb, mean_255, std_255, var_255):
    """生成 Markdown 格式的统计报告"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    mean_bgr = mean_rgb[::-1]
    std_bgr = std_rgb[::-1]
    var_bgr = var_rgb[::-1]
    mean_255_bgr = mean_255[::-1]
    std_255_bgr = std_255[::-1]
    var_255_bgr = var_255[::-1]

    mean_rgb_s = _as_list(mean_rgb, ".6f")
    std_rgb_s = _as_list(std_rgb, ".6f")
    var_rgb_s = _as_list(var_rgb, ".6f")
    mean_bgr_s = _as_list(mean_bgr, ".6f")
    std_bgr_s = _as_list(std_bgr, ".6f")
    var_bgr_s = _as_list(var_bgr, ".6f")

    mean_255_s = _as_list(mean_255, ".4f")
    std_255_s = _as_list(std_255, ".4f")
    var_255_s = _as_list(var_255, ".4f")
    mean_255_bgr_s = _as_list(mean_255_bgr, ".4f")
    std_255_bgr_s = _as_list(std_255_bgr, ".4f")
    var_255_bgr_s = _as_list(var_255_bgr, ".4f")

    report = f"""# 数据集 RGB 统计报告

> 生成时间: {timestamp}

---

## 1. 数据集信息

| 项目 | 值 |
|------|-----|
| 数据集路径 | `{data_dir}` |
| 图片总数 | {total_images:,} 张 |
| 像素总数 | {total_pixels:,} 个 |
| 读取失败 | {failed} 张 |

---

## 2. 统计结果

### 2.1 归一化格式 (像素值范围 [0, 1])

> **适用场景**: PyTorch / TensorFlow 标准归一化流程
>
> **使用方式**: `normalized = (pixel / 255.0 - mean) / std`

| 通道 | 均值 (Mean) | 标准差 (Std) | 方差 (Var) |
|------|-------------|--------------|------------|
| R (红) | {mean_rgb[0]:.6f} | {std_rgb[0]:.6f} | {var_rgb[0]:.6f} |
| G (绿) | {mean_rgb[1]:.6f} | {std_rgb[1]:.6f} | {var_rgb[1]:.6f} |
| B (蓝) | {mean_rgb[2]:.6f} | {std_rgb[2]:.6f} | {var_rgb[2]:.6f} |

**数组格式 (方便复制)**:
```python
mean_rgb = {mean_rgb_s}
std_rgb  = {std_rgb_s}
var_rgb  = {var_rgb_s}

# 如果平台选择 BGR 顺序
mean_bgr = {mean_bgr_s}
std_bgr  = {std_bgr_s}
var_bgr  = {var_bgr_s}
```

---

### 2.2 原始像素格式 (像素值范围 [0, 255])

> **适用场景**: 部分量化平台直接使用的参数格式
>
> **使用方式**: `normalized = (pixel - mean) / std`

| 通道 | 均值 (Mean) | 标准差 (Std) | 方差 (Var) |
|------|-------------|--------------|------------|
| R (红) | {mean_255[0]:.4f} | {std_255[0]:.4f} | {var_255[0]:.4f} |
| G (绿) | {mean_255[1]:.4f} | {std_255[1]:.4f} | {var_255[1]:.4f} |
| B (蓝) | {mean_255[2]:.4f} | {std_255[2]:.4f} | {var_255[2]:.4f} |

**数组格式 (方便复制)**:
```python
mean_255 = {mean_255_s}
std_255  = {std_255_s}
var_255  = {var_255_s}

# 如果平台选择 BGR 顺序
mean_255_bgr = {mean_255_bgr_s}
std_255_bgr  = {std_255_bgr_s}
var_255_bgr  = {var_255_bgr_s}
```

---

## 3. 平台 UI 直接填写参考

许多量化平台的预处理逻辑是类似下面的形式 (每通道):

```
out = (in - mean) / variance
```

其中 UI 的“方差(variance)”经常实际上是 **Std/Scale(除数)**，不一定是真正的 Var。

### 3.1 如果你的模型仅做 `/255` 归一化 (常见于 YOLO)

> **对应**: `out = in / 255`

- 色彩模式: **RGB**
- 均值(mean): `[0, 0, 0]`
- 方差/Scale(除数): `[255, 255, 255]`

### 3.2 如果你的模型使用 `mean/std` 归一化

- 若平台输入 `in` 是 `uint8` `[0,255]`:
  - 均值(mean) 填 `mean_255`
  - 方差(Std/Scale) 填 `std_255`
  - 若平台真的要求 Var，填 `var_255`
- 若平台输入 `in` 已经是 `[0,1]`:
  - 均值(mean) 填 `mean_rgb`
  - 方差(Std/Scale) 填 `std_rgb`
  - 若平台真的要求 Var，填 `var_rgb`

---

## 4. ONNX 量化配置参考

以下为常见量化平台的配置格式:

### 4.1 通用配置
```yaml
preprocess:
  mean: [{mean_rgb[0]:.6f}, {mean_rgb[1]:.6f}, {mean_rgb[2]:.6f}]
  std: [{std_rgb[0]:.6f}, {std_rgb[1]:.6f}, {std_rgb[2]:.6f}]
  input_range: [0, 1]
```

### 4.2 OpenVINO 配置
```
mean_values: [{mean_255[0]:.2f}, {mean_255[1]:.2f}, {mean_255[2]:.2f}]
scale_values: [{std_255[0]:.2f}, {std_255[1]:.2f}, {std_255[2]:.2f}]
```

### 4.3 TensorRT 配置
```python
config.set_flag(trt.BuilderFlag.FP16)
# 预处理: (input - mean) / std
mean = np.array([{mean_rgb[0]:.6f}, {mean_rgb[1]:.6f}, {mean_rgb[2]:.6f}])
std = np.array([{std_rgb[0]:.6f}, {std_rgb[1]:.6f}, {std_rgb[2]:.6f}])
```

---

*本报告由 calc_dataset_stats.py 自动生成*
"""
    return report


def main():
    parser = argparse.ArgumentParser(description='计算数据集 RGB 均值与方差')
    parser.add_argument('--data_dir', type=str,
                        default='/home/potato/RC_2026/train',
                        help='图片数据集路径')
    parser.add_argument('--workers', type=int, default=0,
                        help='并行进程数 (0=自动)')
    parser.add_argument('--output', type=str, default='',
                        help='输出文件路径 (默认: 脚本目录/dataset_stats.md)')
    args = parser.parse_args()

    data_dir = args.data_dir
    workers = args.workers if args.workers > 0 else max(1, cpu_count() - 2)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = args.output if args.output else os.path.join(script_dir, 'dataset_stats.md')

    print(f"[INFO] 数据集路径: {data_dir}")
    print(f"[INFO] 并行进程数: {workers}")

    # 收集图片路径
    print("[INFO] 扫描图片文件...")
    image_paths = get_image_paths(data_dir)
    total_images = len(image_paths)

    if total_images == 0:
        print("[ERROR] 未找到任何图片文件")
        sys.exit(1)

    print(f"[INFO] 共找到 {total_images} 张图片")

    # 计算统计量
    total_pixels = 0
    sum_rgb = np.zeros(3, dtype=np.float64)
    sum_sq_rgb = np.zeros(3, dtype=np.float64)
    failed = 0

    pbar_kwargs = dict(
        total=total_images,
        desc="计算统计量",
        unit="img",
        ncols=80,
    )

    def consume_results(iterator):
        nonlocal total_pixels, sum_rgb, sum_sq_rgb, failed
        for res in tqdm(iterator, **pbar_kwargs):
            if res is None:
                failed += 1
                continue
            px, s, sq = res
            total_pixels += px
            sum_rgb += s
            sum_sq_rgb += sq

    if workers <= 1:
        consume_results(map(compute_image_stats, image_paths))
    else:
        try:
            with Pool(workers) as pool:
                consume_results(pool.imap(compute_image_stats, image_paths, chunksize=64))
        except PermissionError as e:
            # Some sandbox/container environments restrict multiprocessing semaphores.
            print(f"[WARN] multiprocessing 不可用 ({e}); 回退到多线程模式")
            with ThreadPoolExecutor(max_workers=workers) as executor:
                consume_results(executor.map(compute_image_stats, image_paths))

    if failed > 0:
        print(f"[WARN] {failed} 张图片读取失败")

    # 计算均值与标准差
    mean_rgb = sum_rgb / total_pixels
    var_rgb = (sum_sq_rgb / total_pixels) - (mean_rgb ** 2)
    std_rgb = np.sqrt(var_rgb)
    mean_255 = mean_rgb * 255
    std_255 = std_rgb * 255
    var_255 = var_rgb * (255 ** 2)

    # 输出结果到终端
    print("\n" + "=" * 50)
    print("数据集统计结果 (像素值归一化到 [0, 1])")
    print("=" * 50)
    print(f"总像素数: {total_pixels:,}")
    print(f"\nMean (R, G, B): [{mean_rgb[0]:.6f}, {mean_rgb[1]:.6f}, {mean_rgb[2]:.6f}]")
    print(f"Std  (R, G, B): [{std_rgb[0]:.6f}, {std_rgb[1]:.6f}, {std_rgb[2]:.6f}]")

    print("\n" + "-" * 50)
    print("数据集统计结果 (像素值范围 [0, 255])")
    print("-" * 50)
    print(f"Mean (R, G, B): [{mean_255[0]:.4f}, {mean_255[1]:.4f}, {mean_255[2]:.4f}]")
    print(f"Std  (R, G, B): [{std_255[0]:.4f}, {std_255[1]:.4f}, {std_255[2]:.4f}]")

    # 生成并保存 Markdown 报告
    report = generate_markdown_report(
        data_dir, total_images, total_pixels, failed,
        mean_rgb, std_rgb, var_rgb, mean_255, std_255, var_255
    )
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(report)
    print(f"\n[INFO] 结果已保存到: {output_path}")


if __name__ == '__main__':
    main()
