#!/usr/bin/env python3
from __future__ import annotations

import os
from pathlib import Path

from playwright.sync_api import TimeoutError as PlaywrightTimeoutError
from playwright.sync_api import sync_playwright


FRONTEND_URL = os.environ.get("E2E_FRONTEND_URL", "http://127.0.0.1:4173")
ARTIFACT_DIR = Path(os.environ.get("E2E_ARTIFACT_DIR", "artifacts/e2e"))
ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)


def save_failure_artifacts(page) -> None:
    page.screenshot(path=str(ARTIFACT_DIR / "sim-viewer-e2e-failure.png"), full_page=True)
    (ARTIFACT_DIR / "sim-viewer-e2e-failure.html").write_text(page.content(), encoding="utf-8")


def main() -> None:
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(base_url=FRONTEND_URL, viewport={"width": 1440, "height": 960})

        try:
            page.goto("/", wait_until="networkidle")
            page.get_by_role("heading", name="3D 战术观测沙盘").wait_for()
            page.get_by_text("场景已就绪，可在画面上设起点或终点").wait_for(timeout=10000)

            graph_toggle = page.get_by_role("button", name="拓扑")
            if graph_toggle.get_attribute("aria-pressed") != "false":
                raise AssertionError("拓扑图层默认应为关闭。")

            page.get_by_role("button", name="高级 / 调试").click()
            page.get_by_label("起点节点").wait_for(timeout=5000)

            page.get_by_role("button", name="生成手动离线运行").click()
            page.wait_for_function(
                """() => {
                    const buttons = Array.from(document.querySelectorAll("button"));
                    const stepButton = buttons.find((element) => element.textContent?.includes("单步"));
                    return Boolean(stepButton && !stepButton.disabled);
                }""",
                timeout=20000,
            )
            page.wait_for_function(
                """() => {
                    return Array.from(document.querySelectorAll(".stat-value"))
                      .some((element) => element.textContent?.trim() === "0 / 2");
                }""",
                timeout=20000,
            )

            page.get_by_role("button", name="单步").click()
            page.wait_for_function(
                """() => {
                    return Array.from(document.querySelectorAll(".stat-value"))
                      .some((element) => element.textContent?.trim() === "1 / 2");
                }""",
                timeout=20000,
            )

            graph_toggle.click()
            page.wait_for_function(
                """() => {
                    const button = Array.from(document.querySelectorAll("button"))
                      .find((element) => element.getAttribute("aria-label") === "拓扑");
                    return button?.getAttribute("aria-pressed") === "true";
                }""",
                timeout=5000,
            )

            page.get_by_role("button", name="实时只读").click()
            page.get_by_role("button", name="阻塞区").wait_for(timeout=5000)
            live_button = page.get_by_role("button", name="启动实时桥接")
            live_button.wait_for(timeout=5000)
            live_button.click()

            page.wait_for_function(
                """() => {
                    return Array.from(document.querySelectorAll(".metric-row strong"))
                      .some((element) => element.textContent?.trim() == "stub_edge_live");
                }""",
                timeout=10000,
            )
            page.wait_for_function(
                """() => {
                    return Array.from(document.querySelectorAll(".metric-row strong"))
                      .some((element) => element.textContent?.trim() == "stub_corridor");
                }""",
                timeout=10000,
            )
            page.wait_for_function(
                """() => {
                    return Array.from(document.querySelectorAll(".metric-row strong"))
                      .some((element) => element.textContent?.trim() == "0.480");
                }""",
                timeout=10000,
            )
        except PlaywrightTimeoutError:
            save_failure_artifacts(page)
            browser.close()
            raise
        except Exception:
            save_failure_artifacts(page)
            browser.close()
            raise

        page.screenshot(path=str(ARTIFACT_DIR / "sim-viewer-e2e-success.png"), full_page=True)
        browser.close()


if __name__ == "__main__":
    main()
