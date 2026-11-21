#!/usr/bin/env python3
"""抓车的串口日志，自动落到 r3car/logs/ 里，文件名带时间戳。

为什么不用 idf.py monitor：那个会拉 DTR/RTS，偶尔把板子拉进下载模式，
而且日志不落盘。这个脚本开串口后立刻把 dtr/rts 拉低，只读不碰。

用法：
    python3 ~/r3car/logs/capture.py                    # 自动找 usbmodem，存到 logs/
    python3 ~/r3car/logs/capture.py -p /dev/cu.usbmodem1101
    python3 ~/r3car/logs/capture.py -o 跑车.log         # 指定文件名
    Ctrl-C 结束，会打印存到哪了。
"""
import argparse
import datetime as dt
import glob
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial：python3 -m pip install pyserial")

HERE = os.path.dirname(os.path.abspath(__file__))


def pick_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        ports = sorted(glob.glob("/dev/cu.wchusbserial*"))
    if not ports:
        sys.exit("没找到串口。车板插原生 USB 口（usbmodem）或 UART 口（wchusbserial）都行。")
    if len(ports) > 1:
        print("发现多个串口，默认用第一个：", ", ".join(ports), file=sys.stderr)
    return ports[0]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-o", "--out")
    a = ap.parse_args()

    port = a.port or pick_port()
    name = a.out or dt.datetime.now().strftime("%Y-%m-%d-%H%M.log")
    path = name if os.path.isabs(name) else os.path.join(HERE, name)

    ser = serial.Serial(port, a.baud, timeout=0.2)
    try:
        ser.dtr = False
        ser.rts = False
        time.sleep(0.1)
        ser.reset_input_buffer()
    except Exception:
        pass

    started = dt.datetime.now()
    print(f"# 监听 {port} -> {path}（Ctrl-C 结束）", file=sys.stderr)

    lines = 0
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(f"# {port}  {started:%Y-%m-%d %H:%M:%S}\n")
        fh.flush()
        t0 = time.time()
        try:
            while True:
                data = ser.read(4096)
                if not data:
                    continue
                text = data.decode("utf-8", "replace")
                stamp = f"[{time.time()-t0:9.3f}] "
                for line in text.splitlines():
                    fh.write(stamp + line + "\n")
                    lines += 1
                fh.flush()
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
            fh.write(f"# 结束 {dt.datetime.now():%Y-%m-%d %H:%M:%S}，共 {lines} 行\n")

    print(f"\n存好了：{path}（{lines} 行）", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
