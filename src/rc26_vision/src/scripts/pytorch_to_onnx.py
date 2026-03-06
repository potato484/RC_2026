#!/usr/bin/env python3
# RC26_WS=${RC26_WS:-$HOME/RC_2026}
# python3 ${RC26_WS}/src/rc26_vision/src/scripts/pytorch_to_onnx.py --checkpoint ${RC26_WS}/src/rc26_vision/models/best.pt --ultralytics --onnx ${RC26_WS}/src/rc26_vision/models/best.onnx --imgsz 640 --opset 17 --simplify
import argparse
import importlib
import importlib.util
import inspect
import json
import os
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple


def _parse_shape(text: str) -> Tuple[int, ...]:
    parts = [p.strip() for p in text.replace("x", ",").split(",") if p.strip()]
    shape: list[int] = []
    for p in parts:
        try:
            v = int(p)
        except ValueError as e:
            raise ValueError(f"Invalid shape component: {p!r}") from e
        if v <= 0:
            raise ValueError(f"Shape components must be > 0, got: {v}")
        shape.append(v)
    if not shape:
        raise ValueError("Empty shape")
    return tuple(shape)


def _load_object(spec: str) -> Any:
    if ":" not in spec:
        raise ValueError("Model spec must be in the form 'module_or_file.py:attr'")

    mod_part, attr = spec.split(":", 1)
    mod_part = mod_part.strip()
    attr = attr.strip()
    if not mod_part or not attr:
        raise ValueError("Model spec must be in the form 'module_or_file.py:attr'")

    if mod_part.endswith(".py") and os.path.exists(mod_part):
        path = Path(mod_part).resolve()
        module_name = f"_pytorch_to_onnx_{path.stem}"
        spec_obj = importlib.util.spec_from_file_location(module_name, path)
        if spec_obj is None or spec_obj.loader is None:
            raise ImportError(f"Failed to import module from file: {path}")
        module = importlib.util.module_from_spec(spec_obj)
        spec_obj.loader.exec_module(module)
    else:
        module = importlib.import_module(mod_part)

    if not hasattr(module, attr):
        raise AttributeError(f"{module.__name__} has no attribute {attr!r}")
    return getattr(module, attr)


def _extract_state_dict(obj: Any, checkpoint_key: Optional[str]) -> Optional[Dict[str, Any]]:
    if not isinstance(obj, dict):
        return None

    if checkpoint_key:
        v = obj.get(checkpoint_key)
        if isinstance(v, dict):
            return v
        return None

    for k in ("state_dict", "model_state_dict", "model"):
        v = obj.get(k)
        if isinstance(v, dict):
            return v

    # Best-effort: treat dict[str, Tensor] as a state_dict-like object.
    if all(isinstance(k, str) for k in obj.keys()):
        return obj  # type: ignore[return-value]

    return None


def _maybe_strip_module_prefix(state_dict: Dict[str, Any]) -> Dict[str, Any]:
    if not state_dict:
        return state_dict
    if not any(k.startswith("module.") for k in state_dict.keys()):
        return state_dict
    return {k[len("module.") :] if k.startswith("module.") else k: v for k, v in state_dict.items()}


def _filter_kwargs(fn: Any, kwargs: Dict[str, Any]) -> Dict[str, Any]:
    try:
        sig = inspect.signature(fn)
    except (TypeError, ValueError):
        return kwargs

    if any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values()):
        return kwargs

    allowed = set(sig.parameters.keys())
    return {k: v for k, v in kwargs.items() if k in allowed}


def _export_with_ultralytics(
    checkpoint_path: Path,
    onnx_path: Path,
    imgsz: int,
    opset: int,
    dynamic: bool,
    device: str,
    nms: bool,
    simplify: bool,
) -> None:
    try:
        from ultralytics import YOLO  # type: ignore
    except Exception as e:
        raise RuntimeError(
            "ultralytics is not available. Install it (e.g. `pip install ultralytics`) "
            "or export via torch.onnx with --model/--torchscript."
        ) from e

    model = YOLO(str(checkpoint_path))
    export_kwargs: Dict[str, Any] = {
        "format": "onnx",
        "opset": opset,
        "imgsz": imgsz,
        "dynamic": dynamic,
        "device": device,
    }
    if nms:
        export_kwargs["nms"] = True
    if simplify:
        export_kwargs["simplify"] = True
    export_kwargs = _filter_kwargs(model.export, export_kwargs)
    exported = model.export(**export_kwargs)

    # ultralytics may return a path-like object; also usually writes next to the .pt
    candidate_paths = []
    if exported:
        candidate_paths.append(Path(str(exported)))
    candidate_paths.append(checkpoint_path.with_suffix(".onnx"))

    src = next((p for p in candidate_paths if p.exists()), None)
    if src is None:
        raise RuntimeError(
            "ultralytics export completed but the ONNX file was not found. "
            f"Tried: {[str(p) for p in candidate_paths]}"
        )

    onnx_path.parent.mkdir(parents=True, exist_ok=True)
    if src.resolve() != onnx_path.resolve():
        onnx_path.write_bytes(src.read_bytes())


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert a PyTorch model to ONNX.")
    parser.add_argument("--checkpoint", required=True, help="Path to a PyTorch checkpoint (.pt/.pth).")
    parser.add_argument("--onnx", required=True, help="Output ONNX file path.")

    parser.add_argument(
        "--input-shape",
        default="1,3,640,640",
        help="Dummy input tensor shape (default: 1,3,640,640). Also accepts '1x3x640x640'.",
    )
    parser.add_argument("--opset", type=int, default=12, help="ONNX opset version (default: 12).")
    parser.add_argument("--device", default="cpu", help="Device for exporting: cpu/cuda[:index] (default: cpu).")
    parser.add_argument("--dynamic-batch", action="store_true", help="Export with dynamic batch axis.")
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="Export with dynamic batch/height/width axes (may not be supported by all models).",
    )
    parser.add_argument("--input-name", default="images", help="ONNX input name (default: images).")
    parser.add_argument("--output-names", default="", help="Comma-separated ONNX output names (optional).")
    parser.add_argument("--simplify", action="store_true", help="Run onnxsim simplify (if installed).")
    parser.add_argument(
        "--fp16",
        action="store_true",
        help="Convert exported ONNX to FP16 (if possible). Keeps model IO as FP32 by default for compatibility.",
    )
    parser.add_argument(
        "--fp16-io",
        action="store_true",
        help="Also convert ONNX model input/output types to FP16 (may break FP32 inference code). Requires --fp16.",
    )
    parser.add_argument("--no-verify", action="store_true", help="Skip ONNX verification with onnx.checker.")

    parser.add_argument(
        "--torchscript",
        action="store_true",
        help="Treat checkpoint as a TorchScript file (torch.jit.load).",
    )
    parser.add_argument(
        "--model",
        default="",
        help="Model factory in the form 'module_or_file.py:callable' (for state_dict checkpoints).",
    )
    parser.add_argument(
        "--model-kwargs",
        default="{}",
        help="JSON dict passed to the model factory (default: {}).",
    )
    parser.add_argument(
        "--checkpoint-key",
        default="",
        help="When checkpoint is a dict, use this key as the state_dict (optional).",
    )
    parser.add_argument("--strict", action="store_true", help="Strict state_dict loading (default).")
    parser.add_argument(
        "--no-strict",
        dest="strict",
        action="store_false",
        help="Non-strict state_dict loading.",
    )
    parser.set_defaults(strict=True)

    parser.add_argument(
        "--ultralytics",
        action="store_true",
        help="Export via ultralytics.YOLO (useful for YOLOv8/v5 .pt).",
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Ultralytics export image size (default: 640).")
    parser.add_argument(
        "--ultralytics-nms",
        action="store_true",
        help="Ultralytics export with NMS in graph (default off; rc26_vision does NMS in C++).",
    )

    args = parser.parse_args()

    checkpoint_path = Path(args.checkpoint)
    onnx_path = Path(args.onnx)

    if args.fp16_io and not args.fp16:
        raise SystemExit("--fp16-io requires --fp16.")
    if not checkpoint_path.exists():
        raise SystemExit(f"Checkpoint not found: {checkpoint_path}")

    if args.ultralytics:
        _export_with_ultralytics(
            checkpoint_path=checkpoint_path,
            onnx_path=onnx_path,
            imgsz=args.imgsz,
            opset=args.opset,
            dynamic=args.dynamic or args.dynamic_batch,
            device=args.device,
            nms=args.ultralytics_nms,
            simplify=args.simplify,
        )
    else:
        try:
            model_kwargs = json.loads(args.model_kwargs)
        except json.JSONDecodeError as e:
            raise SystemExit(f"Invalid --model-kwargs JSON: {e}") from e
        if not isinstance(model_kwargs, dict):
            raise SystemExit("--model-kwargs must be a JSON object (dict).")

        try:
            shape = _parse_shape(args.input_shape)
        except ValueError as e:
            raise SystemExit(f"Invalid --input-shape: {e}") from e

        try:
            import torch
        except Exception as e:
            raise SystemExit("PyTorch is required. Install it (e.g. `pip install torch`).") from e

        device = torch.device(args.device)

        checkpoint_key = args.checkpoint_key.strip() or None

        if args.torchscript:
            model = torch.jit.load(str(checkpoint_path), map_location=device)
        else:
            ckpt = torch.load(str(checkpoint_path), map_location="cpu")

            if isinstance(ckpt, torch.nn.Module):
                model = ckpt
            elif isinstance(ckpt, dict) and isinstance(ckpt.get("model"), torch.nn.Module):
                model = ckpt["model"]
            else:
                state_dict = _extract_state_dict(ckpt, checkpoint_key)
                if state_dict is None:
                    raise SystemExit(
                        "Checkpoint does not look like a state_dict. "
                        "Use --torchscript, or provide --ultralytics for YOLO .pt, or save a state_dict."
                    )

                if not args.model:
                    raise SystemExit(
                        "State-dict checkpoint requires --model (module_or_file.py:callable) to build the nn.Module."
                    )

                factory = _load_object(args.model)
                if callable(factory):
                    model = factory(**model_kwargs)
                else:
                    model = factory

                if not isinstance(model, torch.nn.Module):
                    raise SystemExit("--model must resolve to a torch.nn.Module or a callable returning one.")

                state_dict = _maybe_strip_module_prefix(state_dict)
                model.load_state_dict(state_dict, strict=args.strict)

            model = model.to(device)

        model.eval()

        dummy = torch.zeros(*shape, device=device, dtype=torch.float32)

        output_names: Optional[Sequence[str]] = None
        if args.output_names.strip():
            output_names = [n.strip() for n in args.output_names.split(",") if n.strip()]

        dynamic_axes: Optional[Dict[str, Dict[int, str]]] = None
        if args.dynamic:
            dynamic_axes = {args.input_name: {0: "batch", 2: "height", 3: "width"}}
            if output_names:
                for name in output_names:
                    dynamic_axes[name] = {0: "batch"}
        elif args.dynamic_batch:
            dynamic_axes = {args.input_name: {0: "batch"}}
            if output_names:
                for name in output_names:
                    dynamic_axes[name] = {0: "batch"}

        onnx_path.parent.mkdir(parents=True, exist_ok=True)

        export_kwargs: Dict[str, Any] = {
            "opset_version": args.opset,
            "do_constant_folding": True,
            "input_names": [args.input_name],
            "output_names": list(output_names) if output_names else None,
            "dynamic_axes": dynamic_axes,
        }
        export_kwargs = _filter_kwargs(torch.onnx.export, export_kwargs)

        with torch.no_grad():
            torch.onnx.export(
                model,
                dummy,
                str(onnx_path),
                **export_kwargs,
            )

    # Ultralytics export already handles its own slimming/simplify step; only run onnxsim here
    # for non-ultralytics exports.
    if args.simplify and (not args.ultralytics):
        try:
            import onnx  # type: ignore
            from onnxsim import simplify  # type: ignore
        except Exception:
            print("[WARN] --simplify requested but 'onnx'/'onnxsim' not available; skipping.")
        else:
            m = onnx.load(str(onnx_path))
            ms, ok = simplify(m)
            if not ok:
                raise SystemExit("onnxsim simplify failed.")
            tmp = onnx_path.with_suffix(onnx_path.suffix + ".tmp")
            onnx.save(ms, str(tmp))
            tmp.replace(onnx_path)

    if args.fp16:
        try:
            import onnx  # type: ignore
            from onnxconverter_common import float16  # type: ignore
        except Exception:
            print("[WARN] --fp16 requested but 'onnx'/'onnxconverter_common' not available; skipping.")
        else:
            m = onnx.load(str(onnx_path))
            m16 = float16.convert_float_to_float16(m, keep_io_types=not args.fp16_io)
            tmp = onnx_path.with_suffix(onnx_path.suffix + ".tmp")
            onnx.save(m16, str(tmp))
            tmp.replace(onnx_path)

    if not args.no_verify:
        try:
            import onnx  # type: ignore
        except Exception:
            print("[WARN] Verification enabled but 'onnx' not available; skipping.")
        else:
            m = onnx.load(str(onnx_path))
            try:
                onnx.checker.check_model(m)
            except Exception as e:
                raise SystemExit(f"ONNX verification failed: {e}") from e

    print(f"[OK] Exported ONNX: {onnx_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
