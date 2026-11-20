#!/usr/bin/env python3
"""
r3car 地面站：大疆 RC2 -> (USB 串口 或 WiFi) -> 履带车

先有线，再无线。两边报文格式完全一样，只是传输不同。

用法：
  python3 drive.py --serial /dev/cu.usbmodem2101     # 有线（推荐先跑这个）
  python3 drive.py                                    # WiFi（自动广播发现车）
  python3 drive.py --serial ... --test                # 不接遥控器，自动斜坡验链路
  python3 drive.py --buttons                          # 只打印按键位，用来认开关
  python3 drive.py --serial ... --max 300             # 限速 30%

摇杆映射（和 Mode 2 FPV 一致）：
  左摇杆 左右 Rz -> 转向（右为正）
  右摇杆 上下 Y  -> 油门（自回中，松手即停）
  左摇杆 上下 Z  -> 暂不使用
"""
import argparse
import socket
import struct
import sys
import time

VID, PID = 0x2CA3, 0x1020
REPORT_LEN = 13
FULL = 660.0
DEADBAND = 40

MAGIC_CMD = (0xA5, 0x5A)
MAGIC_TEL = (0xA5, 0x5B)
UDP_PORT = 3333
SEND_HZ = 50

# RC2 三档开关（实测）：正常位 = bit6 置位，另一侧 = bit7 置位，中间两位都不置。
# 拨到 bit7 那一侧视为"上锁"，禁止输出。
LOCK_BIT = 7
LOCK_WHEN_SET = True


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def build_cmd(seq: int, throttle: int, steer: int, armed: bool) -> bytes:
    body = struct.pack("<BBBBHhh", MAGIC_CMD[0], MAGIC_CMD[1], 1,
                       1 if armed else 0, seq & 0xFFFF, int(throttle), int(steer))
    return body + struct.pack("<H", crc16_ccitt(body))


def parse_telemetry(pkt: bytes):
    if len(pkt) != 16 or pkt[0] != MAGIC_TEL[0] or pkt[1] != MAGIC_TEL[1] or pkt[2] != 1:
        return None
    if (pkt[14] | (pkt[15] << 8)) != crc16_ccitt(pkt[:14]):
        return None
    return dict(state=pkt[3],
                seq=pkt[4] | (pkt[5] << 8),
                vbat=pkt[6] | (pkt[7] << 8),
                rpm_l=struct.unpack_from("<h", pkt, 8)[0],
                rpm_r=struct.unpack_from("<h", pkt, 10)[0],
                loss=pkt[12] | (pkt[13] << 8))


def scan_frames(buf: bytearray):
    """从字节流里挑出遥测帧"""
    out = []
    while True:
        i = buf.find(bytes(MAGIC_TEL))
        if i < 0:
            if len(buf) > 64:
                del buf[:-32]
            return out, buf
        if len(buf) - i < 16:
            del buf[:i]
            return out, buf
        tel = parse_telemetry(bytes(buf[i:i + 16]))
        if tel:
            out.append(tel)
            del buf[:i + 16]
        else:
            del buf[:i + 2]


class UdpLink:
    def __init__(self, host=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setblocking(False)
        self.sock.bind(("", 0))
        self.dest = host or self._broadcast()
        self.addr = (self.dest, UDP_PORT)
        self.name = f"UDP -> {self.dest}:{UDP_PORT}"

    @staticmethod
    def _broadcast():
        """
        找本机真实网卡地址。
        不能直接用 connect(8.8.8.8) —— 机器上有 Clash/Surge 的 TUN 网卡时，
        会拿到 198.18.x.x 这种虚拟网段，广播就发错地方了。
        """
        import subprocess, re
        try:
            out = subprocess.check_output(["ifconfig"], text=True, errors="ignore")
        except Exception:
            out = ""
        # 按接口切块，跳过虚拟接口
        blocks = re.split(r"^(\w+):", out, flags=re.M)
        candidates = []
        for i in range(1, len(blocks) - 1, 2):
            name, body = blocks[i], blocks[i + 1]
            if name.startswith(("utun", "lo", "gif", "stf", "bridge", "awdl", "llw")):
                continue
            m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", body)
            if not m:
                continue
            ip = m.group(1)
            if ip.startswith(("169.254.", "198.18.", "198.19.")):
                continue
            candidates.append((name, ip))
        # 优先 en0（WiFi/有线主网卡）
        for name, ip in candidates:
            if name == "en0":
                p = ip.split(".")
                return ".".join(p[:3] + ["255"])
        if candidates:
            p = candidates[0][1].split(".")
            return ".".join(p[:3] + ["255"])
        # 兜底
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        except Exception:
            return "255.255.255.255"
        finally:
            s.close()
        p = ip.split(".")
        return ".".join(p[:3] + ["255"]) if len(p) == 4 else "255.255.255.255"

    def send(self, pkt):
        try:
            self.sock.sendto(pkt, self.addr)
        except OSError:
            pass

    def poll(self):
        out = []
        try:
            while True:
                data, _addr = self.sock.recvfrom(256)
                t = parse_telemetry(data)
                if t:
                    out.append(t)
        except BlockingIOError:
            pass
        return out

    def close(self):
        self.sock.close()


class SerialLink:
    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0)
        self.ser.dtr = False
        self.ser.rts = False
        self.buf = bytearray()
        self.name = f"SERIAL {port} @ {baud}"

    def send(self, pkt):
        self.ser.write(pkt)

    def poll(self):
        n = self.ser.in_waiting
        if n:
            self.buf.extend(self.ser.read(n))
        frames, self.buf = scan_frames(self.buf)
        return frames

    def close(self):
        self.ser.close()


def open_rc2():
    import hid
    d = hid.device()
    d.open(VID, PID)
    return d


def read_report(dev, timeout_ms=2):
    """取最新一帧。注意：hidapi 的 timeout=0 在这里是阻塞的，只能用 >=1"""
    latest = None
    rep = dev.read(REPORT_LEN, timeout_ms)
    if rep and len(rep) >= REPORT_LEN:
        latest = bytes(rep)
        for _ in range(8):          # 快速追几帧，拿最新的
            rep = dev.read(REPORT_LEN, 1)
            if not rep or len(rep) < REPORT_LEN:
                break
            latest = bytes(rep)
    if latest is None:
        return None
    b = latest
    return dict(buttons=b[0] | (b[1] << 8) | (b[2] << 16),
                x=struct.unpack_from("<h", b, 3)[0],
                y=struct.unpack_from("<h", b, 5)[0],
                z=struct.unpack_from("<h", b, 7)[0],
                rz=struct.unpack_from("<h", b, 9)[0])


def norm(v):
    if abs(v) < DEADBAND:
        return 0.0
    return max(-1.0, min(1.0, v / FULL))


def button_mode():
    dev = open_rc2()
    print("按键监视：拨动你想用来上锁的开关，看哪个 bit 在变。Ctrl-C 退出。\n")
    last = None
    try:
        while True:
            r = read_report(dev)
            if not r:
                continue
            m = r["buttons"]
            if m != last:
                bits = [str(i) for i in range(24) if m & (1 << i)]
                print(f"mask 0x{m:06X}  bits: {','.join(bits) if bits else '-'}   "
                      f"X{r['x']:+5d} Y{r['y']:+5d} Z{r['z']:+5d} Rz{r['rz']:+5d}")
                last = m
    except KeyboardInterrupt:
        print("\n退出。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", metavar="PORT", help="有线：串口设备路径")
    ap.add_argument("--host", default=None, help="WiFi：直接指定车 IP")
    ap.add_argument("--test", action="store_true", help="不接遥控器，自动斜坡")
    ap.add_argument("--buttons", action="store_true", help="只打印按键位")
    ap.add_argument("--max", type=int, default=600, help="油门上限 0-1000")
    ap.add_argument("--invert-throttle", action="store_true")
    ap.add_argument("--invert-steer", action="store_true")
    args = ap.parse_args()

    if args.buttons:
        button_mode()
        return

    link = SerialLink(args.serial) if args.serial else UdpLink(args.host)
    print(f"传输：{link.name}")

    rc2 = None
    if not args.test:
        try:
            rc2 = open_rc2()
            print("RC2 已连接")
        except Exception as exc:
            sys.exit(f"打不开 RC2：{exc}\n（只验链路可用 --test）")

    print("右摇杆自回中：松手即停。Ctrl-C 退出。\n")

    seq = 0
    centered_since = None
    armed = False
    t0 = time.time()
    last_report = None
    y = rz = 0
    raw_last = False
    locked = False
    tel = None
    tel_t = 0.0
    last_print = 0.0
    interval = 1.0 / SEND_HZ

    try:
        while True:
            now = time.time()

            if args.test:
                t = now - t0
                thr = int(args.max * 0.5) if 2.0 <= t < 5.0 else 0
                st = 0
            else:
                r = read_report(rc2)
                if r is None:
                    r = last_report
                else:
                    last_report = r
                if r is None:
                    thr = st = 0
                    raw_last = False
                else:
                    y, rz = r["y"], r["rz"]
                    raw_last = True
                    ty = norm(r["y"])
                    if args.invert_throttle:
                        ty = -ty
                    sx = norm(r["rz"])
                    if args.invert_steer:
                        sx = -sx
                    thr = int(ty * args.max)
                    st = int(sx * args.max)

            # 开关上锁：拨到 LOCK_BIT 那一侧就不输出
            locked = False
            if r is not None:
                locked = bool(r["buttons"] & (1 << LOCK_BIT)) == LOCK_WHEN_SET
            if locked:
                thr = st = 0
                armed = False
            else:
                armed = True

            seq += 1
            link.send(build_cmd(seq, thr, st, armed))

            for t in link.poll():
                tel, tel_t = t, now

            if now - last_print > 0.4:
                last_print = now
                if tel and now - tel_t < 1.0:
                    st_name = {0: "DISARM", 1: "NET", 2: "ARMED"}.get(tel["state"], "?")
                    raw = f"Y{y:+5d} Rz{rz:+5d}" if raw_last else "Y  --- Rz  ---"
                    lock = "LOCK" if locked else "----"
                    print(f"\r链路 {int((now-tel_t)*1000):3d}ms | {lock} {st_name:6s} | "
                          f"L {tel['rpm_l']:+5d} R {tel['rpm_r']:+5d} rpm | "
                          f"{tel['vbat']/1000:4.2f}V | "
                          f"油门 {thr:+4d} 转向 {st:+4d} | {raw}   ",
                          end="", flush=True)
                else:
                    raw = f"Y{y:+5d} Rz{rz:+5d}" if raw_last else "Y  --- Rz  ---"
                    print(f"\r链路 丢失 | 油门 {thr:+4d} 转向 {st:+4d} | {raw}   ",
                          end="", flush=True)

            time.sleep(max(0.0, interval - (time.time() - now)))

    except KeyboardInterrupt:
        print("\n\n退出：发停止 1 秒 ...")
        for _ in range(50):
            seq += 1
            link.send(build_cmd(seq, 0, 0, False))
            time.sleep(0.02)
        link.close()
        print("已停。")


if __name__ == "__main__":
    main()
