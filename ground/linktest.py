#!/usr/bin/env python3
"""链路测距：一边发零油门保活包，一边统计收包率、车端丢包、Mac 侧 RSSI。

专为户外拉距离用：油门固定 0（车不会动），你只需带着车和电脑走，每到一个距离停 20 秒。
所有数据同时打印和写 CSV，回来直接画曲线。

  连车的热点时：
    python3 -u ground/linktest.py --host 192.168.4.1 --csv link.csv
  有线（只验工具本身）：
    python3 -u ground/linktest.py --serial /dev/cu.usbmodem2101 --seconds 5

信号强度来自车端（上行包里的 RSSI），所以**不需要 macOS 扫描**——扫描会打断射频，
测出来的丢包其实是自己在打自己。想看 Mac 侧的数再加 --mac-rssi，但那一项的代价就是丢包。

屏幕每一行：时间 | 收包 Hz | 车丢增量 | Mac 侧 RSSI
收包 Hz 是最近 2 秒的统计。标称 10 Hz，**有线实测 9.3 Hz 是正常上限**（固件 100 ms 定时的抖动），
所以判读看的是相对下降：9 Hz 左右正常，掉到 5 以下开始丢，0 就是断。
"""
import argparse
import csv
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import drive  # noqa: E402

try:
    import rssi as rssi_mod
except Exception:
    rssi_mod = None


class RssiThread(threading.Thread):
    def __init__(self, period=3.0):
        super().__init__(daemon=True)
        self.period = period
        self.value = None          # (ssid, rssi, snr, ch)
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            if rssi_mod is None:
                return
            info = rssi_mod.sample()
            if info and info.get("rssi") is not None:
                self.value = (info["ssid"], info["rssi"], info["snr"], info["channel"])
            self._stop.wait(self.period)

    def stop(self):
        self._stop.set()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=None, help="车 IP，如 192.168.4.1")
    ap.add_argument("--serial", metavar="PORT", help="有线串口")
    ap.add_argument("--csv", metavar="PATH", help="记录到 CSV")
    ap.add_argument("--mac-rssi", action="store_true",
                    help="额外读 Mac 侧 RSSI。注意：它会触发 macOS 扫描，扫描期间必然丢包，默认关")
    ap.add_argument("--seconds", type=float, default=0, help="跑多少秒后退出，0=一直跑")
    args = ap.parse_args()

    link = drive.SerialLink(args.serial) if args.serial else drive.UdpLink(args.host)
    print(f"传输：{link.name}")

    rt = None
    if args.mac_rssi and rssi_mod is not None:
        rt = RssiThread()
        rt.start()

    fh = wr = None
    if args.csv:
        fh = open(args.csv, "a", newline="")
        wr = csv.writer(fh)
        if fh.tell() == 0:
            wr.writerow(["time", "距离", "收包hz", "车丢增量", "车侧rssi", "mac_rssi", "snr", "ch"])

    seq = 0
    t0 = time.time()
    stamps = []            # 最近 2 秒收到的遥测时刻
    last_loss = None
    loss_delta = 0
    car_rssi = 0
    last_print = 0.0
    print("发零油门保活包中（车不会动）。Ctrl-C 退出。\n")

    try:
        while True:
            now = time.time()
            if args.seconds and now - t0 > args.seconds:
                break
            link.send(drive.build_cmd(seq, 0, 0, True))
            seq += 1
            for t in link.poll():
                stamps.append(now)
                if last_loss is not None:
                    loss_delta = (t["loss"] - last_loss) & 0xFFFF
                last_loss = t["loss"]
                car_rssi = t.get("rssi", 0)

            while stamps and now - stamps[0] > 3.0:
                stamps.pop(0)

            if now - last_print > 1.0 and now - t0 > 2.5:
                last_print = now
                win = min(3.0, max(0.5, now - t0))
                hz = len(stamps) / win
                mac = ""
                mr = ""
                ssid = snr = ch = ""
                if rt and rt.value:
                    ssid, mr, snr, ch = rt.value
                    mac = f"  mac {mr} dBm"
                tag = "断链" if hz < 0.5 else ("丢包" if hz < 5.0 else "")
                print(f"{time.strftime('%H:%M:%S')}  收包 {hz:4.1f} Hz  车丢 +{loss_delta:2d}  "
                      f"车侧 {car_rssi:4d} dBm{mac}  {tag}")
                if wr:
                    wr.writerow([time.strftime("%H:%M:%S"), "", f"{hz:.1f}", loss_delta,
                                 car_rssi, mr if rt and rt.value else "", snr, ch])
                    fh.flush()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        link.send(drive.build_cmd(seq, 0, 0, False))
        link.close()
        if rt:
            rt.stop()
        if fh:
            fh.close()
        print("\n已停。")


if __name__ == "__main__":
    main()
