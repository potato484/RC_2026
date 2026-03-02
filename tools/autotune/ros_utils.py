from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


def run_command(
    cmd: List[str],
    timeout_sec: Optional[float] = None,
    cwd: Optional[Path] = None,
    env: Optional[Dict[str, str]] = None,
) -> Tuple[int, str, str]:
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    try:
        proc = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            cwd=str(cwd) if cwd else None,
            env=merged_env,
        )
        return proc.returncode, proc.stdout, proc.stderr
    except subprocess.TimeoutExpired as ex:
        stdout = ex.stdout if isinstance(ex.stdout, str) else ""
        stderr = ex.stderr if isinstance(ex.stderr, str) else ""
        return 124, stdout, stderr + "\nTIMEOUT"
    except Exception as ex:
        return 1, "", str(ex)


@dataclass
class ManagedProcess:
    cmd: List[str]
    name: str
    cwd: Optional[Path] = None
    env: Optional[Dict[str, str]] = None
    popen: Optional[subprocess.Popen] = field(default=None, init=False)

    def start(self) -> None:
        if self.popen is not None:
            return
        merged_env = os.environ.copy()
        if self.env:
            merged_env.update(self.env)
        self.popen = subprocess.Popen(
            self.cmd,
            cwd=str(self.cwd) if self.cwd else None,
            env=merged_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=os.setsid,
        )

    def is_running(self) -> bool:
        return self.popen is not None and self.popen.poll() is None

    def wait(self, timeout_sec: Optional[float] = None) -> Optional[int]:
        if self.popen is None:
            return None
        try:
            return self.popen.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            return None

    def terminate(self, grace_sec: float = 3.0) -> None:
        if self.popen is None:
            return
        if self.popen.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.popen.pid), signal.SIGTERM)
        except Exception:
            pass
        deadline = time.time() + max(0.1, grace_sec)
        while time.time() < deadline:
            if self.popen.poll() is not None:
                return
            time.sleep(0.1)
        try:
            os.killpg(os.getpgid(self.popen.pid), signal.SIGKILL)
        except Exception:
            pass


def wait_for_topic(topic: str, timeout_sec: float = 15.0) -> bool:
    deadline = time.time() + max(0.5, timeout_sec)
    while time.time() < deadline:
        code, out, _ = run_command(["ros2", "topic", "list"], timeout_sec=2.0)
        if code == 0 and topic in out.splitlines():
            return True
        time.sleep(0.5)
    return False


def _parse_ros2_param_get(text: str) -> Tuple[bool, Any]:
    m = re.search(r"value is:\s*(.+)$", text, flags=re.IGNORECASE | re.MULTILINE)
    if not m:
        return False, None
    raw = m.group(1).strip()
    low = raw.lower()
    if low in {"true", "false"}:
        return True, low == "true"
    try:
        if "." in raw or "e" in low:
            return True, float(raw)
        return True, int(raw)
    except ValueError:
        return True, raw.strip("'\"")


def ros2_param_get(node: str, param: str, timeout_sec: float = 3.0) -> Tuple[bool, Any, str]:
    code, out, err = run_command(["ros2", "param", "get", node, param], timeout_sec=timeout_sec)
    if code != 0:
        return False, None, (out + "\n" + err).strip()
    ok, value = _parse_ros2_param_get(out)
    if not ok:
        return False, None, out.strip()
    return True, value, ""


def ros2_param_set(node: str, param: str, value: Any, timeout_sec: float = 3.0) -> Tuple[bool, str]:
    text = str(value).lower() if isinstance(value, bool) else str(value)
    code, out, err = run_command(["ros2", "param", "set", node, param, text], timeout_sec=timeout_sec)
    merged = (out + "\n" + err).strip()
    if code != 0:
        return False, merged
    success = "successful" in merged.lower() or "set parameter successful" in merged.lower()
    return success, merged


def call_set_nav_mode(
    profile: str = "safe",
    timeout: float = 0.0,
    reason: str = "autotune_trial",
    service: str = "/set_nav_mode",
    timeout_sec: float = 5.0,
) -> Tuple[bool, str]:
    safe_reason = reason.replace("'", "")
    payload = "{profile: '" + profile + "', timeout: " + str(float(timeout)) + ", reason: '" + safe_reason + "'}"
    code, out, err = run_command(
        ["ros2", "service", "call", service, "rc26_interfaces/srv/SetNavMode", payload],
        timeout_sec=timeout_sec,
    )
    merged = (out + "\n" + err).strip()
    if code != 0:
        return False, merged
    if re.search(r"success:\s*true", merged, flags=re.IGNORECASE):
        return True, merged
    return False, merged


def read_bool_param(nodes: Iterable[str], param: str) -> Tuple[bool, Optional[bool]]:
    for node in nodes:
        ok, value, _ = ros2_param_get(node, param, timeout_sec=2.0)
        if ok and isinstance(value, bool):
            return True, value
    return False, None


@dataclass
class ParamSnapshot:
    values: Dict[Tuple[str, str], Any] = field(default_factory=dict)

    def capture(self, pairs: Iterable[Tuple[str, str]]) -> None:
        for node, param in pairs:
            ok, value, _ = ros2_param_get(node, param, timeout_sec=3.0)
            if ok:
                self.values[(node, param)] = value

    def restore(self) -> Dict[str, Any]:
        failures = []
        for (node, param), value in list(self.values.items())[::-1]:
            ok, msg = ros2_param_set(node, param, value, timeout_sec=3.0)
            if not ok:
                failures.append({"node": node, "param": param, "reason": msg})
        return {"ok": len(failures) == 0, "failures": failures}


def find_latest_bundle(evidence_dir: Path) -> Optional[Path]:
    if not evidence_dir.exists():
        return None
    bundles = [p for p in evidence_dir.iterdir() if p.is_dir() and p.name.startswith("bundle_")]
    if not bundles:
        return None
    bundles.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return bundles[0]
