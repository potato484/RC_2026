#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import textwrap


SDK_REPO = "https://github.com/Livox-SDK/Livox-SDK2.git"
HELPER_SOURCE = textwrap.dedent(
    r'''
    #include "livox_lidar_api.h"
    #include "livox_lidar_def.h"

    #include <arpa/inet.h>

    #include <atomic>
    #include <chrono>
    #include <cstdio>
    #include <cstdlib>
    #include <thread>

    static std::atomic<bool> point_seen(false);
    static std::atomic<bool> imu_seen(false);
    static std::atomic<bool> reboot_sent(false);
    static std::atomic<uint32_t> active_handle(0);

    void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
      (void)dev_type;
      (void)client_data;
      if (data == nullptr) {
        return;
      }
      point_seen.store(true);
      std::printf("POINT handle=%u dots=%u len=%u frame=%u\n", handle, data->dot_num, data->length, data->frame_cnt);
      std::fflush(stdout);
    }

    void ImuDataCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
      (void)dev_type;
      (void)client_data;
      if (data == nullptr) {
        return;
      }
      imu_seen.store(true);
      std::printf("IMU handle=%u dots=%u len=%u frame=%u\n", handle, data->dot_num, data->length, data->frame_cnt);
      std::fflush(stdout);
    }

    void WorkModeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data) {
      (void)client_data;
      std::printf("WORKMODE status=%d handle=%u ret=%u err=%u\n",
                  status,
                  handle,
                  response ? response->ret_code : 255,
                  response ? response->error_key : 65535);
      std::fflush(stdout);
    }

    void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
      (void)client_data;
      std::printf("REBOOT status=%d handle=%u ret=%u\n",
                  status,
                  handle,
                  response ? response->ret_code : 255);
      std::fflush(stdout);
    }

    void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
      (void)client_data;
      active_handle.store(handle);
      struct in_addr addr;
      addr.s_addr = handle;
      std::printf("LIDAR handle=%u ip=%s sn=%s\n", handle, inet_ntoa(addr), info ? info->sn : "?");
      std::fflush(stdout);
    }

    int main(int argc, char** argv) {
      if (argc != 4) {
        std::fprintf(stderr, "usage: %s <config_json> <warmup_sec> <timeout_sec>\n", argv[0]);
        return 2;
      }

      const char* config_path = argv[1];
      const double warmup_sec = std::atof(argv[2]);
      const double timeout_sec = std::atof(argv[3]);

      if (!LivoxLidarSdkInit(config_path)) {
        std::fprintf(stderr, "LivoxLidarSdkInit failed\n");
        LivoxLidarSdkUninit();
        return 3;
      }

      SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
      SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
      SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

      const auto begin = std::chrono::steady_clock::now();
      while (true) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - begin).count();

        if (point_seen.load() && imu_seen.load()) {
          std::printf("RECOVERED pointcloud_and_imu_ready\n");
          std::fflush(stdout);
          LivoxLidarSdkUninit();
          return 0;
        }

        const uint32_t handle = active_handle.load();
        if (handle != 0 && !reboot_sent.load() && elapsed >= warmup_sec) {
          bool expected = false;
          if (reboot_sent.compare_exchange_strong(expected, true)) {
            const auto work_mode_rc = SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
            const auto reboot_rc = LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
            std::printf("RECOVER action=set_normal rc=%d reboot rc=%d handle=%u\n", work_mode_rc, reboot_rc, handle);
            std::fflush(stdout);
          }
        }

        if (elapsed >= timeout_sec) {
          std::fprintf(stderr, "Timed out waiting for point cloud / IMU data\n");
          LivoxLidarSdkUninit();
          return 4;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    '''
)


def run(cmd: list[str], cwd: Path | None = None) -> None:
    print("+", " ".join(shlex.quote(part) for part in cmd), flush=True)
    env = os.environ.copy()
    env.setdefault("GIT_TERMINAL_PROMPT", "0")
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True, env=env)


def ensure_sdk_checkout(sdk_dir: Path, refresh_sdk: bool) -> bool:
    if sdk_dir.exists():
      if refresh_sdk:
        run(["git", "pull", "--ff-only"], cwd=sdk_dir)
        return True
      return False
    sdk_dir.parent.mkdir(parents=True, exist_ok=True)
    run(["git", "clone", "--depth", "1", SDK_REPO, str(sdk_dir)])
    return True


def build_helper(sdk_dir: Path, force_sdk_rebuild: bool = False) -> Path:
    build_dir = sdk_dir / "build"
    build_dir.mkdir(parents=True, exist_ok=True)

    cache_file = build_dir / "CMakeCache.txt"
    static_lib = build_dir / "sdk_core" / "liblivox_lidar_sdk_static.a"
    helper_src = build_dir / "rc26_mid360_recover.cpp"
    helper_bin = build_dir / "rc26_mid360_recover"

    if force_sdk_rebuild or not cache_file.exists():
        run(["cmake", "-B", str(build_dir), "-S", str(sdk_dir)])

    if force_sdk_rebuild or not static_lib.exists():
        run(["cmake", "--build", str(build_dir), "-j3", "--target", "livox_lidar_sdk_static"])

    helper_src_needs_update = True
    if helper_src.exists():
        helper_src_needs_update = helper_src.read_text(encoding="utf-8") != HELPER_SOURCE
    if helper_src_needs_update:
        helper_src.write_text(HELPER_SOURCE, encoding="utf-8")

    helper_needs_build = (
        not helper_bin.exists()
        or helper_src_needs_update
        or static_lib.stat().st_mtime > helper_bin.stat().st_mtime
    )
    if helper_needs_build:
        run(
            [
                "g++",
                "-std=c++11",
                "-I",
                str(sdk_dir / "include"),
                "-I",
                str(sdk_dir / "3rdparty"),
                "-I",
                str(sdk_dir / "sdk_core"),
                "-o",
                str(helper_bin),
                str(helper_src),
                str(static_lib),
                "-lpthread",
            ]
        )
    else:
        print(f"Reusing cached Livox helper: {helper_bin}", flush=True)

    return helper_bin


def write_config(config_path: Path, lidar_ip: str, host_ip: str, cmd_port: int, push_port: int, point_port: int, imu_port: int, log_port: int) -> None:
    config = {
        "MID360": {
            "lidar_net_info": {
                "cmd_data_port": 56100,
                "push_msg_port": 56200,
                "point_data_port": 56300,
                "imu_data_port": 56400,
                "log_data_port": 56500,
            },
            "host_net_info": [
                {
                    "lidar_ip": [lidar_ip],
                    "host_ip": host_ip,
                    "cmd_data_port": cmd_port,
                    "push_msg_port": push_port,
                    "point_data_port": point_port,
                    "imu_data_port": imu_port,
                    "log_data_port": log_port,
                }
            ],
        }
    }
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Recover Livox Mid-360 UDP streaming with official SDK2.")
    parser.add_argument("--lidar-ip", default="192.168.1.140")
    parser.add_argument("--host-ip", default="192.168.1.50")
    parser.add_argument("--cmd-port", type=int, default=56101)
    parser.add_argument("--push-port", type=int, default=56201)
    parser.add_argument("--point-port", type=int, default=56301)
    parser.add_argument("--imu-port", type=int, default=56401)
    parser.add_argument("--log-port", type=int, default=56501)
    parser.add_argument("--warmup-before-reboot", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument(
        "--sdk-dir",
        default=str(Path.home() / ".cache" / "rc26_mid360_driver" / "Livox-SDK2"),
        help="Local Livox-SDK2 checkout. Reused if already present.",
    )
    parser.add_argument("--refresh-sdk", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sdk_dir = Path(args.sdk_dir).expanduser().resolve()

    try:
        sdk_changed = ensure_sdk_checkout(sdk_dir, args.refresh_sdk)
        helper_bin = build_helper(sdk_dir, force_sdk_rebuild=sdk_changed)
        with tempfile.TemporaryDirectory(prefix="mid360_recover_") as tmp_dir:
            config_path = Path(tmp_dir) / "mid360_config.json"
            write_config(
                config_path=config_path,
                lidar_ip=args.lidar_ip,
                host_ip=args.host_ip,
                cmd_port=args.cmd_port,
                push_port=args.push_port,
                point_port=args.point_port,
                imu_port=args.imu_port,
                log_port=args.log_port,
            )
            run([
                str(helper_bin),
                str(config_path),
                str(args.warmup_before_reboot),
                str(args.timeout),
            ])
    except subprocess.CalledProcessError as exc:
        print(f"recover_mid360_stream failed with exit code {exc.returncode}", file=sys.stderr)
        return exc.returncode

    print("Mid-360 recovery completed. You can now verify `/livox/lidar` and `/livox/imu`.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
