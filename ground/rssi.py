#!/usr/bin/env python3
"""Mac 端 WiFi 信号监视 —— 户外测履带车链路上限时用。

读的是 Mac 当前连着的那个 AP 的 Signal / Noise。
测车时必须连着车自己开的热点（SSID `r3car`），不是家里的网——连家里的网时这个工具没用。

用法：
  python3 -u ground/rssi.py                 # 每采一次打印一行
  python3 -u ground/rssi.py --csv rssi.csv  # 同时写 CSV，事后画曲线

判读（2.4G、板载天线，实测经验值）：
  -50 dBm 以内   很近，链路随便跑
  -70 左右       正常
  -80 左右       开始丢包
  -85 以下       基本断了

注意：system_profiler 一次要跑 3–8 秒，所以采样间隔由它决定，别指望它做高频记录。
"""
import argparse
import csv
import re
import subprocess
import sys
import time

CMD = ["system_profiler", "SPAirPortDataType"]
RE_SIG = re.compile(r"(-?\d+) dBm\s*/\s*(-?\d+) dBm")
RE_CH = re.compile(r"Channel:\s*(\d+)\s*\((\S+?)(?:,|\))")


def sample():
    """返回 dict(ssid, rssi, noise, snr, channel, band)；未连接时 ssid=None。"""
    try:
        out = subprocess.run(CMD, capture_output=True, text=True, timeout=90).stdout
    except subprocess.TimeoutExpired:
        return None
    lines = out.splitlines()
    for i, line in enumerate(lines):
        if "Current Network Information" not in line:
            continue
        block = lines[i + 1:i + 16]
        # 只取当前网络这一段，别把后面的 "Other Local Wi-Fi Networks" 混进来
        for j, b in enumerate(block):
            if "Other Local Wi-Fi Networks" in b:
                block = block[:j]
                break
        if not block:
            return None
        ssid = block[0].strip().rstrip(":")
        info = {"ssid": ssid, "rssi": None, "noise": None, "snr": None,
                "channel": None, "band": None}
        for b in block[1:]:
            m = RE_SIG.search(b)
            if m:
                info["rssi"] = int(m.group(1))
                info["noise"] = int(m.group(2))
                info["snr"] = info["rssi"] - info["noise"]
            m = RE_CH.search(b)
            if m:
                info["channel"] = int(m.group(1))
                info["band"] = m.group(2)
        return info
    return None


def bar(rssi):
    """-90..-30 映射成 12 格。"""
    if rssi is None:
        return ""
    n = max(0, min(12, round((rssi + 90) / 5)))
    return "█" * n + "·" * (12 - n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", metavar="PATH", help="把每次采样追加写进 CSV")
    ap.add_argument("--quiet", action="store_true", help="只在信号变化超过 3 dB 时打印")
    args = ap.parse_args()

    fh = None
    wr = None
    if args.csv:
        fh = open(args.csv, "a", newline="")
        wr = csv.writer(fh)
        if fh.tell() == 0:
            wr.writerow(["time", "ssid", "rssi_dbm", "noise_dbm", "snr_db", "channel", "band"])

    last_rssi = None
    print("开始采样（Ctrl-C 退出）。Mac 要连着 r3car 的热点才有意义。\n")
    try:
        while True:
            info = sample()
            now = time.strftime("%H:%M:%S")
            if info is None or info.get("ssid") is None:
                print(f"{now}  (读不到当前网络)")
            else:
                r = info["rssi"]
                show = (not args.quiet) or last_rssi is None or abs(r - last_rssi) >= 3
                if show:
                    print(f"{now}  {info['ssid']:<14s} RSSI {r:4d} dBm  "
                          f"SNR {info['snr']:3d} dB  ch{str(info['channel']):>3s} {info['band'] or '':6s} "
                          f"{bar(r)}")
                last_rssi = r
                if wr:
                    wr.writerow([now, info["ssid"], r, info["noise"], info["snr"],
                                 info["channel"], info["band"]])
                    fh.flush()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n退出。")
    finally:
        if fh:
            fh.close()


if __name__ == "__main__":
    sys.exit(main())
