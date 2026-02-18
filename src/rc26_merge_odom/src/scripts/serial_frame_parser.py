#!/usr/bin/env python3
import argparse
import re
import struct
import sys
from typing import List, Tuple

FRAME_HEAD = bytes([0xAA, 0x55])
FRAME_TAIL = bytes([0x55, 0xAA])
ALT_HEAD = bytes([0x55, 0xAA])
ALT_TAIL = bytes([0xAA, 0x55])
MAX_PAYLOAD_SIZE = 32
ODOM_CMD = 0x20
ODOM_PAYLOAD_LEN = 16


def crc32_mpeg2(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc


def parse_hex_string(text: str) -> bytes:
    tokens = re.findall(r"[0-9A-Fa-f]{2}", text)
    if not tokens:
        return b""
    return bytes(int(t, 16) for t in tokens)


def find_next_head_any(data: bytes, start: int) -> Tuple[int, bytes, bytes]:
    idx_a = data.find(FRAME_HEAD, start)
    idx_b = data.find(ALT_HEAD, start)

    if idx_a < 0 and idx_b < 0:
        return -1, b"", b""
    if idx_a < 0:
        return idx_b, ALT_HEAD, ALT_TAIL
    if idx_b < 0:
        return idx_a, FRAME_HEAD, FRAME_TAIL
    if idx_a <= idx_b:
        return idx_a, FRAME_HEAD, FRAME_TAIL
    return idx_b, ALT_HEAD, ALT_TAIL


def decode_frame(frame: bytes, head: bytes, tail: bytes, force_payload_len: int = None) -> Tuple[bool, str, dict]:
    if len(frame) < 12:
        return False, "frame_too_short", {}

    if frame[0:2] != head:
        return False, "bad_head", {}

    seq = frame[2]
    length = frame[3]
    retry = frame[4]
    cmd = frame[5]

    if force_payload_len is not None:
        payload_len = force_payload_len
        expected_size = 11 + 1 + payload_len
    else:
        if length < 1:
            return False, "bad_len_zero", {"seq": seq, "len": length}

        payload_len = length - 1
        if payload_len > MAX_PAYLOAD_SIZE:
            return False, "bad_len_oversize", {"seq": seq, "len": length, "payload_len": payload_len}

        expected_size = 11 + length

    if len(frame) != expected_size:
        return False, "bad_len_mismatch", {"seq": seq, "len": length, "frame_size": len(frame)}

    if frame[-2:] != tail:
        return False, "bad_tail", {"seq": seq, "len": length, "tail": frame[-2:]}

    crc_wire = frame[-6] | (frame[-5] << 8) | (frame[-4] << 16) | (frame[-3] << 24)
    if force_payload_len is not None:
        synthetic_len = 1 + payload_len
        crc_buf = bytearray(frame[2 : 2 + (1 + 1 + 1 + synthetic_len)])
        crc_buf[1] = synthetic_len  # replace LEN for CRC check
        crc_calc = crc32_mpeg2(bytes(crc_buf))
    else:
        crc_calc = crc32_mpeg2(frame[2 : 2 + (1 + 1 + 1 + length)])
    if crc_wire != crc_calc:
        return False, "bad_crc", {"seq": seq, "len": length, "crc_wire": crc_wire, "crc_calc": crc_calc}

    payload = frame[6 : 6 + payload_len]
    info = {
        "seq": seq,
        "len": length,
        "retry": retry,
        "cmd": cmd,
        "payload": payload,
    }
    return True, "ok", info


def parse_stream(data: bytes) -> None:
    idx = 0
    total = 0
    ok_count = 0
    errors = {}
    len_counts = {}
    cmd_counts = {}
    print(f"Input bytes: {len(data)}")

    while idx + 1 < len(data):
        head_pos, head_bytes, tail_bytes = find_next_head_any(data, idx)
        if head_pos < 0:
            break
        if head_pos + 4 > len(data):
            break

        length = data[head_pos + 3]
        cmd = data[head_pos + 5]
        len_counts[length] = len_counts.get(length, 0) + 1
        cmd_counts[cmd] = cmd_counts.get(cmd, 0) + 1
        if cmd == ODOM_CMD:
            frame_size = 11 + 1 + ODOM_PAYLOAD_LEN
        else:
            frame_size = 11 + length
        if head_pos + frame_size > len(data):
            break

        frame = data[head_pos : head_pos + frame_size]
        total += 1
        if cmd == ODOM_CMD:
            ok, reason, info = decode_frame(frame, head_bytes, tail_bytes, force_payload_len=ODOM_PAYLOAD_LEN)
        else:
            ok, reason, info = decode_frame(frame, head_bytes, tail_bytes)
        if ok:
            ok_count += 1
            cmd = info["cmd"]
            payload = info["payload"]
            print(f"[OK] frame#{ok_count} seq={info['seq']} retry={info['retry']} cmd=0x{cmd:02X} payload_len={len(payload)}")
            if cmd == ODOM_CMD:
                if len(payload) != 16:
                    print("  ODOM_DATA payload length mismatch (expected 16)")
                else:
                    v_fl, v_rl, v_rr, v_fr = struct.unpack("<ffff", payload)
                    print(f"  ODOM_DATA v_fl={v_fl:.4f} v_rl={v_rl:.4f} v_rr={v_rr:.4f} v_fr={v_fr:.4f}")
        else:
            errors[reason] = errors.get(reason, 0) + 1
            if reason == "bad_crc":
                print(
                    f"[BAD] crc seq={info.get('seq')} len={info.get('len')} "
                    f"wire=0x{info.get('crc_wire', 0):08X} calc=0x{info.get('crc_calc', 0):08X}"
                )
            elif reason == "bad_len_oversize":
                seq = info.get("seq")
                length = info.get("len")
                cmd = info.get("cmd")
                payload_start = head_pos + 6
                payload_preview = data[payload_start : payload_start + 16]
                preview_hex = payload_preview.hex().upper()
                print(
                    f"[BAD] bad_len_oversize seq={seq} len={length} cmd=0x{cmd:02X} "
                    f"next16={preview_hex}"
                )
            else:
                print(f"[BAD] {reason}")

        if ok:
            idx = head_pos + frame_size
        else:
            idx = head_pos + 2

    print("\nSummary:")
    print(f"  total_frames_parsed: {total}")
    print(f"  ok_frames: {ok_count}")
    if errors:
        for k, v in sorted(errors.items()):
            print(f"  {k}: {v}")
    else:
        print("  errors: none")
    if len_counts:
        top_lens = sorted(len_counts.items(), key=lambda x: (-x[1], x[0]))[:10]
        print("  len_distribution_top10:", ", ".join(f"{k}: {v}" for k, v in top_lens))
    if cmd_counts:
        top_cmds = sorted(cmd_counts.items(), key=lambda x: (-x[1], x[0]))[:10]
        print("  cmd_distribution_top10:", ", ".join(f"0x{k:02X}: {v}" for k, v in top_cmds))

    if ok_count == 0:
        print("\nLikely causes:")
        if errors.get("bad_crc"):
            print("  - CRC32 mismatch: MCU CRC algorithm or byte order differs from MPEG-2 CRC32.")
        if errors.get("bad_tail"):
            print("  - Frame tail mismatch: MCU tail bytes differ from 0x55 0xAA.")
        if errors.get("bad_len_mismatch") or errors.get("bad_len_oversize") or errors.get("bad_len_zero"):
            print("  - LEN field mismatch: MCU length definition differs (LEN should be CMD+PAYLOAD).")
        if not errors:
            print("  - No full frame detected: baudrate mismatch, noisy line, or data stream not following expected head 0xAA 0x55.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Parse RC26 serial frames (HEAD AA55, TAIL 55AA, CRC32 MPEG-2)."
    )
    parser.add_argument("--hex", help="Hex string of bytes (e.g. 'AA 55 01 11 ...')")
    parser.add_argument("--file", help="Binary file containing raw serial bytes")
    args = parser.parse_args()

    if args.hex:
        data = parse_hex_string(args.hex)
    elif args.file:
        with open(args.file, "rb") as f:
            data = f.read()
    else:
        data = parse_hex_string(sys.stdin.read())

    if not data:
        print("No input bytes. Provide --hex, --file, or pipe hex text into stdin.")
        return 1

    parse_stream(data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
