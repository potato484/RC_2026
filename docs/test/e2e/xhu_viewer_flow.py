#!/usr/bin/env python3
from __future__ import annotations

import os
import re
from pathlib import Path

from playwright.sync_api import TimeoutError as PlaywrightTimeoutError
from playwright.sync_api import sync_playwright


FRONTEND_URL = os.environ.get("E2E_FRONTEND_URL", "http://127.0.0.1:4173")
ARTIFACT_DIR = Path(os.environ.get("E2E_ARTIFACT_DIR", "artifacts/e2e"))
ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)


def save_failure_artifacts(page) -> None:
    page.screenshot(path=str(ARTIFACT_DIR / "xhu-viewer-e2e-failure.png"), full_page=True)
    (ARTIFACT_DIR / "xhu-viewer-e2e-failure.html").write_text(page.content(), encoding="utf-8")


def click_canvas_fraction(page, x_ratio: float, y_ratio: float) -> None:
    canvas = page.locator("canvas")
    canvas.wait_for(state="visible", timeout=10000)
    box = canvas.bounding_box()
    if box is None:
        raise AssertionError("未能获取 3D 画布尺寸，无法执行选点。")

    canvas.click(
        position={
            "x": box["width"] * x_ratio,
            "y": box["height"] * y_ratio,
        }
    )


def click_canvas_until_status(page, positions: list[tuple[float, float]], expected_text: str) -> None:
    last_error: Exception | None = None
    for x_ratio, y_ratio in positions:
        click_canvas_fraction(page, x_ratio, y_ratio)
        try:
            page.get_by_text(expected_text).wait_for(timeout=2500)
            return
        except PlaywrightTimeoutError as error:
            last_error = error

    raise AssertionError(f"多次点击画布后仍未看到预期状态: {expected_text}") from last_error


def set_trace_index(page, index: int) -> None:
    slider = page.locator("#trace-index")
    slider.wait_for(state="visible", timeout=5000)
    current = int(slider.input_value() or "0")
    if current == index:
        return

    slider.focus()
    key = "ArrowLeft" if index < current else "ArrowRight"
    for _ in range(abs(index - current)):
        slider.press(key)


def main() -> None:
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(base_url=FRONTEND_URL, viewport={"width": 1440, "height": 960})

        try:
            page.goto("/", wait_until="networkidle")
            page.get_by_role("heading", name="RC26 全局比赛场地闭环可视化平台").wait_for()
            page.get_by_text("场景已就绪，可切换布局并在场景里生成路线").wait_for(timeout=10000)
            page.get_by_text("当前布局: 操作员").wait_for(timeout=10000)
            page.get_by_text("梅林区").first.wait_for(timeout=10000)

            page.get_by_role("button", name="顶部正交").click()

            page.get_by_role("button", name="设起点").click()
            click_canvas_until_status(
                page,
                positions=[(0.47, 0.53), (0.46, 0.55), (0.48, 0.51), (0.45, 0.57)],
                expected_text="起点已记录，继续在场景中设置终点",
            )

            page.get_by_role("button", name="设终点").click()
            click_canvas_until_status(
                page,
                positions=[(0.53, 0.47), (0.54, 0.45), (0.52, 0.49), (0.55, 0.43)],
                expected_text="终点已记录，可以生成三维路线",
            )

            page.get_by_role("button", name="生成三维路线").click()
            page.get_by_text("三维路线与搜索回放已就绪", exact=False).wait_for(timeout=20000)
            page.get_by_text("表面起点采样点", exact=True).wait_for(timeout=10000)
            page.get_by_text("表面终点采样点", exact=True).wait_for(timeout=10000)
            page.get_by_text("回放帧").wait_for(timeout=10000)

            expanded_toggle = page.get_by_role("button", name="已探查")
            expanded_toggle.click()
            page.wait_for_function(
                """() => {
                    const button = Array.from(document.querySelectorAll("button"))
                      .find((element) => element.getAttribute("aria-label") === "已探查");
                    return button?.getAttribute("aria-pressed") === "true";
                }""",
                timeout=5000,
            )

            set_trace_index(page, 0)
            page.wait_for_function(
                """() => Array.from(document.querySelectorAll(".panel-section .trace-title"))
                    .some((element) => element.textContent?.trim() === "初始化前沿")""",
                timeout=5000,
            )
            page.locator(".range-readout").filter(has_text=re.compile(r"^1 / ")).wait_for(timeout=5000)

            page.get_by_role("button", name="加载局部规划案例").click()
            page.get_by_text("已载入局部规划案例 pass_straight，最终状态 ok").wait_for(timeout=10000)
        except PlaywrightTimeoutError:
            save_failure_artifacts(page)
            browser.close()
            raise
        except Exception:
            save_failure_artifacts(page)
            browser.close()
            raise

        page.screenshot(path=str(ARTIFACT_DIR / "xhu-viewer-e2e-success.png"), full_page=True)
        browser.close()


if __name__ == "__main__":
    main()
