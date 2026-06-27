#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from typing import Any


def normalize_namespace(namespace: str) -> str:
    return "/".join(part for part in namespace.strip().split("/") if part)


def apply_namespace(topic: str, namespace: str) -> str:
    if not topic.startswith("/"):
        return topic

    normalized_namespace = normalize_namespace(namespace)
    if not normalized_namespace:
        return topic

    prefix = f"/{normalized_namespace}"
    if topic == prefix or topic.startswith(prefix + "/"):
        return topic
    return prefix + topic


def rewrite_layout_value(value: Any, namespace: str) -> Any:
    if isinstance(value, dict):
        rewritten: dict[str, Any] = {}
        for key, item in value.items():
            if key == "topics" and isinstance(item, dict):
                rewritten_topics = {
                    apply_namespace(topic_name, namespace): rewrite_layout_value(topic_config, namespace)
                    for topic_name, topic_config in item.items()
                }
                rewritten[key] = rewritten_topics
            elif key == "topicPath" and isinstance(item, str):
                rewritten[key] = apply_namespace(item, namespace)
            else:
                rewritten[key] = rewrite_layout_value(item, namespace)
        return rewritten
    if isinstance(value, list):
        return [rewrite_layout_value(item, namespace) for item in value]
    return value


def render_layout_file(source_path: Path, output_path: Path, namespace: str) -> None:
    with source_path.open("r", encoding="utf-8") as handle:
        layout = json.load(handle)

    rendered = rewrite_layout_value(layout, namespace)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(rendered, handle, ensure_ascii=False, indent=2)
        handle.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Render Foxglove layouts for the active ROS namespace.")
    parser.add_argument("--source-dir", required=True, help="Directory containing source Foxglove JSON layouts")
    parser.add_argument("--output-dir", required=True, help="Directory to write rendered layouts")
    parser.add_argument("--namespace", default="", help="Active ROS namespace, e.g. r2 or team/r2")
    args = parser.parse_args()

    source_dir = Path(args.source_dir)
    output_dir = Path(args.output_dir)
    namespace = normalize_namespace(args.namespace)

    for layout_name in ("operator.json", "engineering.json", "diagnostic.json"):
        render_layout_file(source_dir / layout_name, output_dir / layout_name, namespace)

    manifest = {
        "namespace": namespace,
        "source_dir": str(source_dir),
        "output_dir": str(output_dir),
        "layouts": ["operator.json", "engineering.json", "diagnostic.json"],
    }
    with (output_dir / "manifest.json").open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)
        handle.write("\n")

    print(f"Rendered Foxglove layouts to {output_dir} (namespace='{namespace}')")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
