# -*- coding: utf-8 -*-
r"""l103_can.py - 通过 COM5 上的 USB-CAN (DM-tools 主机协议) 控制 L103 + DRV8313 FOC 板。

零依赖 (ctypes 直接开串口), 主机协议沿用 motor_bench/dm_pv_swing.py 里那套 30B/16B 帧。

报文 (标准帧, 小端):
  0x100 主机->板: PV 命令
        b0..3 int32 目标位置 [mrad]      b4..5 int16 速度上限 [0.01 rad/s]
        b6 flags: bit0 1=相对位置 / bit1 1=运行 / bit2 1=停止时松轴
                  bit3 1=清堵转故障 / bit4 1=立刻存 Flash
  0x101 主机->板: 速度命令
        b0..1 int16 速度 [0.01 rad/s]    b2 flags 同 b6 (bit0 忽略)
  0x180 板->主机: 每 10ms 反馈 (仅在固件 debug 开时才有)
        b0..3 int32 当前位置 [mrad]      b4..5 int16 当前速度 [0.01 rad/s]
        b6 状态位   b7 int8 相电流 [0.1A]

  ★ 板子固件是**被动设备**: debug 默认关时不发 0x180。
    所以本脚本默认每 10ms 主动发一次 0x113(b0=0), 用 0x181 应答拿数据
    (0x181 布局与 0x180 相同, 见 can_bus.c:407)。确认板子会自己发才用 --no-poll。

用法:
  python l103_can.py mon                       只监听反馈
  python l103_can.py pos --turns 2.5 --rpm 60  走到 +2.5 圈, 速度上限 60rpm
  python l103_can.py pos --deg 720  --rpm 60   走到 +720 度
  python l103_can.py pos --rad 1.5  --rpm 60 --rel
  python l103_can.py vel --rpm 100             速度模式 100rpm
  python l103_can.py stop                       停止并保持在原位
  python l103_can.py stop --release             停止并松轴
  python l103_can.py clear                      清堵转故障
  python l103_can.py save                       把当前位置存进 Flash
  python l103_can.py raw --id 0x100 --data 0102030405060708
  python l103_can.py seq --preset list            现成的往返模板
  python l103_can.py seq --preset roundtrip       0 -> +1圈 -> +2圈 -> 回机械0
  python l103_can.py seq --segs "..." --dwell 0.5   自己排段(机械坐标 rad:rpm)

标准命令(不用记参数, 从任何目录都能跑):
  l103_seq.ps1（与本脚本同目录）
"""
import ctypes
import io, ctypes.wintypes as wt, struct, time, sys, math, argparse

PORT = "COM5"
BAUD = 921600

ID_CMD_PV      = 0x100
ID_CMD_VEL     = 0x101
ID_CMD_ENABLE  = 0x110
ID_CMD_HOME    = 0x111
ID_CMD_SETZERO = 0x112
ID_CMD_QUERY   = 0x113
ID_CMD_FAULTCFG= 0x114
ID_FB          = 0x180
ID_FB_QUERY    = 0x181
ID_FB_CFG      = 0x182
ID_FB_FAULT    = 0x183
ID_EVT_FAULT   = 0x1A0

HOST_CMD_FWD = 0x01

ST = [(0x01, "run"), (0x02, "arrived"), (0x04, "STALL"),
      (0x08, "sensor-err"), (0x10, "aligned"), (0x20, "pos-restored"),
      (0x40, "enabled"), (0x80, "zero-set")]

CFG_F = [(0x01, "zero-set"), (0x02, "aligned"), (0x04, "VM-sense"),
         (0x08, "enabled"), (0x10, "report-on")]

# seq 的现成模板 -- 有了它, "标准命令"就不用记那串 --segs
# 段写法: 目标(机械坐标 rad, 0 = 机械零点, 得先用 UART F 设过):速度上限(rpm)
PRESETS = [
    ("roundtrip", "6.2832:100,12.5664:100,0:100",
     "0 -> +1圈 -> +2圈 -> 回机械0 (100rpm); 起点最好在机械0"),
    ("home", "0:100",
     "直接回机械0 (100rpm)"),
    ("nudge", "0.5236:60,0:60",
     "0 -> +30度 -> 回机械0 (60rpm), 小幅试探用"),
]
PRESET_DICT = {_n: _s for _n, _s, _d in PRESETS}
# 中文也叫得动 (家人惯用中文):  往返 / 回零 / 试探
PRESET_ALIAS = {"往返": "roundtrip", "回零": "home", "试探": "nudge"}

FAULT_NAMES = [(0x0001, "STALL"), (0x0002, "OVERCURRENT"), (0x0004, "DISABLED"),
               (0x0008, "VM_LOW"), (0x0010, "SENSOR"), (0x0020, "NOT_ALIGNED"),
               (0x0040, "FLASH_FAIL"), (0x0080, "DRV_NFAULT"), (0x0100, "CMD_REJECT")]
FAULT_DICT = dict(FAULT_NAMES)

kernel32 = ctypes.windll.kernel32
GENERIC_READ = 0x80000000; GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3; BAD_H = wt.HANDLE(-1).value


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wt.DWORD), ("BaudRate", wt.DWORD), ("fBits", wt.DWORD),
        ("wReserved", wt.WORD), ("XonLim", wt.WORD), ("XoffLim", wt.WORD),
        ("ByteSize", wt.BYTE), ("Parity", wt.BYTE), ("StopBits", wt.BYTE),
        ("XonChar", ctypes.c_char), ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char), ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char), ("wReserved1", wt.WORD),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [("ReadIntervalTimeout", wt.DWORD), ("ReadTotalTimeoutMultiplier", wt.DWORD),
                ("ReadTotalTimeoutConstant", wt.DWORD), ("WriteTotalTimeoutMultiplier", wt.DWORD),
                ("WriteTotalTimeoutConstant", wt.DWORD)]


kernel32.CreateFileW.restype = wt.HANDLE
kernel32.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, ctypes.c_void_p, wt.DWORD, wt.DWORD, wt.HANDLE]
kernel32.GetCommState.argtypes = [wt.HANDLE, ctypes.POINTER(DCB)]
kernel32.SetCommState.argtypes = [wt.HANDLE, ctypes.POINTER(DCB)]
kernel32.SetCommTimeouts.argtypes = [wt.HANDLE, ctypes.POINTER(COMMTIMEOUTS)]
kernel32.ReadFile.argtypes = [wt.HANDLE, ctypes.c_void_p, wt.DWORD, ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
kernel32.WriteFile.argtypes = [wt.HANDLE, ctypes.c_void_p, wt.DWORD, ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
kernel32.CloseHandle.argtypes = [wt.HANDLE]
kernel32.PurgeComm.argtypes = [wt.HANDLE, wt.DWORD]


def open_port(name=PORT, baud=BAUD):
    h = kernel32.CreateFileW("\\\\.\\" + name, GENERIC_READ | GENERIC_WRITE, 0, None,
                             OPEN_EXISTING, 0, None)
    if h == BAD_H:
        raise OSError("cannot open %s (winerr=%d): 关掉占用 %s 的其它程序"
                      % (name, ctypes.WinError().winerror or 0, name))
    dcb = DCB(); dcb.DCBlength = ctypes.sizeof(DCB)
    if not kernel32.GetCommState(h, ctypes.byref(dcb)):
        raise OSError("GetCommState failed")
    dcb.BaudRate = baud; dcb.ByteSize = 8; dcb.Parity = 0; dcb.StopBits = 0; dcb.fBits = 1
    if not kernel32.SetCommState(h, ctypes.byref(dcb)):
        raise OSError("SetCommState failed")
    # 非阻塞读: MAXDWORD + 两个 total=0 = "有就立刻给, 没有就立刻返回 0"
    # (原来 ReadTotalTimeoutConstant=50 会在没数据时每次卡满 50ms)
    to = COMMTIMEOUTS()
    to.ReadIntervalTimeout = 0xFFFFFFFF
    to.ReadTotalTimeoutMultiplier = 0
    to.ReadTotalTimeoutConstant = 0
    to.WriteTotalTimeoutMultiplier = 0
    to.WriteTotalTimeoutConstant = 1000
    kernel32.SetCommTimeouts(h, ctypes.byref(to))
    kernel32.PurgeComm(h, 0x000F)
    return h


def write_all(h, data):
    n = wt.DWORD(0)
    buf = ctypes.create_string_buffer(bytes(data))
    if not kernel32.WriteFile(h, buf, len(data), ctypes.byref(n), None):
        raise OSError("write failed")
    return n.value


def read_avail(h, wait_s):
    end = time.time() + wait_s
    out = bytearray()
    while time.time() < end:
        n = wt.DWORD(0)
        tmp = ctypes.create_string_buffer(4096)
        ok = kernel32.ReadFile(h, tmp, 4096, ctypes.byref(n), None)
        if ok and n.value:
            out += tmp.raw[:n.value]
            end = max(end, time.time() + 0.02)
        else:
            time.sleep(0.002)
    return bytes(out)


def host_frame(cmd, can_id, data, times=1, interval_ms=5):
    b = bytearray(b"\x55\xaa\x1e")
    b.append(cmd)
    b += (times & 0xFFFFFFFF).to_bytes(4, "little")
    b += (interval_ms & 0xFFFFFFFF).to_bytes(4, "little")
    b.append(0)                                   # ID type: standard
    b += (can_id & 0x1FFFFFFF).to_bytes(4, "little")
    b.append(0)                                   # frame type: data
    b.append(8); b.append(0); b.append(0)         # len / idAcc / dataAcc
    b += bytes(data).ljust(8, b"\x00")[:8]
    b.append(0)                                   # CRC ignored
    assert len(b) == 30
    return bytes(b)


# DM-USB2CAN 接收帧结构 (模块说明书 / dm_usb2can_frames.txt 里的 CAN_Fream 结构体):
#   [0] 0xAA 帧头        [1] CMD  00 心跳 / 0x11 接收成功 / 0x12 发送成功
#                               0x01 接收失败 / 0x02 发送失败 / 0xEE 通讯错误
#   [2] canDataLen:6 | canIde:1<<6 | canRtr:1<<7
#   [3..6] CANID  uint32 **小端**   [7..14] data[8]   [15] 0x55 帧尾
# 2026-09-19 实测: 板子发的 0x180 在 COM5 上就是 80 01 00 00 这样排的,
# 按大端读会得到 0x10000 (旧脚本 dm_pv_swing.py 的 parse_rx 就是这么写的)。
def parse_rx(buf):
    """16B 主机接收帧 -> [(cmd, can_id, data8)]"""
    frames = []
    i = 0
    while i + 15 < len(buf):
        if buf[i] != 0xAA or buf[i + 15] != 0x55:
            i += 1
            continue
        cmd = buf[i + 1]
        can_id = int.from_bytes(buf[i + 3:i + 7], "little")
        data = bytes(buf[i + 7:i + 15])
        frames.append((cmd, can_id, data))
        i += 16
    return frames


CMD_NAME = {0x00: "heartbeat", 0x01: "rx-fail", 0x11: "rx-ok",
            0x02: "tx-fail", 0x12: "tx-ok", 0x03: "brate-fail",
            0x13: "brate-ok", 0xEE: "comm-err"}


def send(h, can_id, data, times=1, interval_ms=5):
    f = host_frame(HOST_CMD_FWD, can_id, data, times, interval_ms)
    write_all(h, f)


# ---------------- CAN 发送帧的屏幕显示 ----------------
CMD_TITLE = {0x100: "PV    位置-速度", 0x101: "VEL   纯速度",
             0x110: "EN    使能/失能", 0x111: "HOME  回机械原点",
             0x112: "ZERO  设机械零点", 0x113: "QUERY 询问",
             0x114: "FCFG  故障上报配置"}

PV_FLAGS = [(0x01, "相对位置"), (0x02, "运行"), (0x04, "停止时松轴"),
            (0x08, "清堵转"), (0x10, "立刻存Flash"), (0x20, "目标=机械坐标")]


def show_tx(can_id, data, dec=None):
    """把即将发出的 CAN 帧打到屏幕上 (ID + 8 字节 + 逐字段解码)"""
    d = bytes(data).ljust(8, b"\x00")[:8]
    print("")
    print("  +---------------------- CAN 发送帧 ----------------------+")
    print("  | ID    0x%03X   标准帧   8 字节   1 Mbps" % can_id)
    print("  | 名称  %s" % CMD_TITLE.get(can_id, "?"))
    print("  | DATA  %s" % " ".join("%02X" % b for b in d))
    print("  |       b0 b1 b2 b3 b4 b5 b6 b7")
    for ln in (dec or []):
        print("  | 解码  %s" % ln)
    print("  +--------------------------------------------------------+")


def dec_flags(fl):
    return ",".join(n for b, n in PV_FLAGS if fl & b) or "-"


def dec_pv(d):
    tgt  = int.from_bytes(d[0:4], "little", signed=True)
    vlim = int.from_bytes(d[4:6], "little", signed=True)
    fl   = d[6]
    return ["目标位置 %+d mrad = %+.4f rad = %+.3f 圈 = %+.2f 度"
            % (tgt, tgt / 1000.0, tgt / 1000.0 / (2 * math.pi),
               math.degrees(tgt / 1000.0)),
            "速度上限 %+d x0.01rad/s = %+.2f rad/s = %+.2f rpm"
            % (vlim, vlim / 100.0, vlim / 100.0 * 60.0 / (2 * math.pi)),
            "flags  0x%02X -> %s" % (fl, dec_flags(fl))]


def dec_vel(d):
    v = int.from_bytes(d[0:2], "little", signed=True)
    return ["速度 %+d x0.01rad/s = %+.2f rad/s = %+.2f rpm"
            % (v, v / 100.0, v / 100.0 * 60.0 / (2 * math.pi)),
            "flags  0x%02X -> %s" % (d[2], dec_flags(d[2]))]


def decode_fb(d):
    pos_mrad = int.from_bytes(d[0:4], "little", signed=True)
    vel_crad = int.from_bytes(d[4:6], "little", signed=True)
    st = d[6]
    cur = int.from_bytes(d[7:8], "little", signed=True)
    return pos_mrad, vel_crad, st, cur


def fmt_fb(d):
    pos, vel, st, cur = decode_fb(d)
    names = ",".join(n for b, n in ST if st & b) or "-"
    return ("pos=%+9.3f rad (%+8.4f trn)  vel=%+8.3f rad/s (%+7.2f rpm)  "
            "I=%+5.2fA  [%s]" % (pos / 1000.0, pos / 1000.0 / (2 * math.pi),
                                 vel / 100.0, vel / 100.0 * 60.0 / (2 * math.pi),
                                 cur / 10.0, names))


def fmt_cfg(d):
    home = int.from_bytes(d[0:4], "little", signed=True) / 1000.0
    vm   = int.from_bytes(d[4:6], "little") / 10.0
    f, per = d[6], d[7]
    fl = ",".join(n for b, n in CFG_F if f & b) or "-"
    return "home=%+9.4f rad  VM=%.1fV  rep=%d(x100ms)  [%s]" % (home, vm, per, fl)


def fmt_fault(d):
    act = int.from_bytes(d[0:2], "little")
    lat = int.from_bytes(d[2:4], "little")
    ev  = int.from_bytes(d[4:8], "little")
    an = ",".join(n for b, n in FAULT_NAMES if act & b) or "-"
    ln = ",".join(n for b, n in FAULT_NAMES if lat & b) or "-"
    return "active=[%s] latched=[%s] events=%d" % (an, ln, ev)


def fmt_evt(d):
    bit   = int.from_bytes(d[0:2], "little")
    isclr = d[2]
    kind  = d[3]
    info  = int.from_bytes(d[4:6], "little", signed=True)
    ms    = int.from_bytes(d[6:8], "little")
    name  = FAULT_DICT.get(bit, "0x%04X" % bit)
    extra = ""
    if kind == 1:   extra = " (%.2fA)" % (info / 100.0)
    elif kind == 2: extra = " (%.1fV)" % (info / 10.0)
    elif kind == 3: extra = " (cmd 0x%03X)" % (info & 0x7FF)
    return "%s %s%s  t=%dms" % (name, "CLEARED" if isclr else "RAISED", extra, ms)


POLL        = True     # 板子是被动设备: 不问就一个反馈帧都不发, 所以默认主动轮询
POLL_PERIOD = 0.01     # 轮询间隔 [s]


def _pump(h, deadline, on_frame=None, on_module=None, poll=None, poll_period=None):
    """读到 deadline 为止, 把每个完整 CAN 帧交给回调; 回调返回真值则提前结束。
    返回 True = 被回调喊停, False = 自然超时。

    poll=True 时每 poll_period 秒主动发一次 0x113(b0=0), 板子回 0x181。
    2026-09-29: 固件 debug 默认关之后板子不再自发 0x180, 光听串口听不到东西,
    所以默认开轮询; 板子自己会发时用 --no-poll 关掉。
    """
    if poll is None:
        poll = POLL
    if poll_period is None:
        poll_period = POLL_PERIOD
    pending = bytearray()
    next_poll = 0.0
    while True:
        now = time.time()
        if now >= deadline:
            return False
        if poll and now >= next_poll:
            send(h, ID_CMD_QUERY, b"\x00")
            next_poll = now + poll_period
        n = wt.DWORD(0)
        tmp = ctypes.create_string_buffer(4096)
        ok = kernel32.ReadFile(h, tmp, 4096, ctypes.byref(n), None)
        if not ok or not n.value:
            time.sleep(0.001)
            continue
        pending += tmp.raw[:n.value]
        i = 0
        while i + 15 < len(pending):
            if pending[i] != 0xAA or pending[i + 15] != 0x55:
                i += 1
                continue
            cmd = pending[i + 1]
            can_id = int.from_bytes(pending[i + 3:i + 7], "little")
            data = bytes(pending[i + 7:i + 15])
            i += 16              # 只裁到最后一个完整帧的末尾, 残字节留到下一批
            if cmd != 0x11:
                if on_module is not None and on_module(cmd, data):
                    return True
            elif len(data) == 8 and on_frame is not None and on_frame(can_id, data):
                return True
        del pending[:i]


def monitor(h, secs, quiet=False, watch_id=ID_FB, every=1, minvel=0.0):
    """按固定截止时间收帧。
    注意: 不能顺手用 read_avail() —— 那个函数每收到数据就把截止时间往后推,
    板上是 100 帧/s 的持续反馈, 用它会永远退不出来 (2026-09-19 踩过)。
    2026-09-29: 走 _pump, 默认靠 0x113 轮询 0x181 拿数据 (板子是被动的)。"""
    t0 = time.time()
    nfb = 0
    nmod = 0
    seen = {}

    def on_frame(can_id, data):
        nonlocal nfb
        seen[can_id] = seen.get(can_id, 0) + 1
        if can_id in (ID_FB, ID_FB_QUERY) and len(data) == 8:
            nfb += 1
            # minvel: 只打运动中的帧。USB-CAN 适配器会把上一次进程关掉后的帧
            # 积压在内部缓冲里, 开端口时一次性吐出来 (2026-09-19 实测),
            # 那一波会把真正在走的帧淹没, 所以量速度要用这个过滤。
            if minvel > 0.0 and abs(decode_fb(data)[1]) / 100.0 < minvel:
                return False
            if not quiet and (nfb % every == 0):
                tag = "fb  " if can_id == ID_FB else "stat"
                print("[%7.3fs] %s %s" % (time.time() - t0, tag, fmt_fb(data)))
        elif can_id == ID_FB_CFG and len(data) == 8:
            if not quiet:
                print("[%7.3fs] cfg  %s" % (time.time() - t0, fmt_cfg(data)))
        elif can_id == ID_FB_FAULT and len(data) == 8:
            if not quiet:
                print("[%7.3fs] flt  %s" % (time.time() - t0, fmt_fault(data)))
        elif can_id == ID_EVT_FAULT and len(data) == 8:
            if not quiet:
                print("[%7.3fs] EVT! %s" % (time.time() - t0, fmt_evt(data)))
        elif not quiet:
            print("[%7.3fs] RX id=0x%03X data=%s"
                  % (time.time() - t0, can_id, data.hex(" ")))
        return False

    def on_module(cmd, data):
        nonlocal nmod
        nmod += 1
        # 0x12 tx-ok / 0x00 心跳 是适配器对我们每次发送的例行 ack, 轮询时会刷屏
        if not quiet and cmd not in (0x12, 0x00):
            print("[%7.3fs] module %s (cmd=0x%02X) data=%s"
                  % (time.time() - t0, CMD_NAME.get(cmd, "?"), cmd, data.hex(" ")))
        return False

    _pump(h, t0 + secs, on_frame, on_module)
    if nmod and not quiet:
        print("(模块自身状态帧 %d 条, 不算总线数据)" % nmod)
    return nfb, seen


def wait_arrive(h, target_mrad, timeout=8.0, tol_mrad=30, settle=3,
                blend_mrad=0, stats=None):
    """收帧直到 "到位" 为止。
    判定 = status 带 arrived 且 |pos - 目标| <= tol。
    **必须带位置校验**: USB-CAN 适配器会把上一次进程关掉后的帧
    积压在内部缓冲里, 开端口时一次性吐出来 (2026-09-19 实测),
    那些旧帧的 arrived 也是 1。光看 arrived 会误判成"一发就到位"。
    还顺手统计这一趟的峰值速度/峰值电流。

    2026-09-29: 走 _pump, 默认靠 0x113 轮询 0x181 拿数据 (板子是被动的)。
    一帧都没收到时 nsamp=0 —— 峰值那两项就是初值, 不是测量值, 调用方要看清。

    返回 (dt, 终点mrad, 峰值rad/s, 峰值A, status, 末速度rad/s, 样本数)"""
    t0 = time.time()
    s = {"pos": 0, "st": 0, "vel": 0.0, "vmax": 0.0, "imax": 0.0, "hit": 0, "n": 0}

    def on_frame(can_id, data):
        if can_id is not None and can_id in (ID_FB, ID_FB_QUERY):
            pos, vel, st, cur = decode_fb(data)
            s["pos"], s["st"], s["vel"] = pos, st, vel / 100.0
            s["n"] += 1
            if abs(vel) / 100.0 > s["vmax"]:
                s["vmax"] = abs(vel) / 100.0
            if abs(cur) / 10.0 > s["imax"]:
                s["imax"] = abs(cur) / 10.0
            if blend_mrad > 0:
                near = abs(pos - target_mrad) <= blend_mrad
            else:
                near = bool(st & 0x02) and abs(pos - target_mrad) <= tol_mrad
            if near:
                s["hit"] += 1
                if s["hit"] >= settle:
                    return True
            else:
                s["hit"] = 0
        elif can_id == ID_EVT_FAULT:
            print("   [事件] %s" % fmt_evt(data))
        return False

    _pump(h, t0 + timeout, on_frame)
    if stats is not None:
        stats.update(s)
    return (time.time() - t0, s["pos"], s["vmax"], s["imax"], s["st"], s["vel"], s["n"])


def parse_segs(s):
    """'5.236:300,193.732:600' -> [(5.236, 300.0), (193.732, 600.0)]  (第二列是 rpm 上限)"""
    out = []
    for item in s.replace(";", ",").split(","):
        item = item.strip()
        if not item:
            continue
        tt, vv = item.split(":", 1) if ":" in item else (item, "30")
        out.append((float(tt), float(vv)))
    assert out, "segs 解析出来是空的"
    return out


def read_pos(h, secs):
    """静置期间读 secs 秒, 返回**最后一帧**(缓冲区里最新的) -> (pos_mrad, vel_rad_s, st, cur_A)。
    USB-CAN 适配器会积压旧帧, 但帧是有序的, 取最后一条就是最新的。
    2026-09-29: 走 _pump, 默认靠 0x113 轮询 0x181 拿数据 (板子是被动的)。"""
    last = {"v": None}

    def on_frame(can_id, data):
        if can_id == ID_EVT_FAULT:
            print("   [事件] %s" % fmt_evt(data))
        elif can_id in (ID_FB, ID_FB_QUERY):
            pos, vel, st, cur = decode_fb(data)
            last["v"] = (pos, vel / 100.0, st, cur / 10.0)
        return False

    _pump(h, time.time() + secs, on_frame)
    return last["v"] or (0, 0.0, 0, 0.0)


def build_pv(turns=None, deg=None, rad=None, rpm=30.0, rel=False, run=True,
             release=False, clear=False, save=False, mech=False):
    if turns is not None:
        rad = turns * 2 * math.pi
    elif deg is not None:
        rad = math.radians(deg)
    elif rad is None:
        rad = 0.0
    flags = 0
    if rel:     flags |= 0x01
    if run:     flags |= 0x02
    if release: flags |= 0x04
    if clear:   flags |= 0x08
    if save:    flags |= 0x10
    if mech:    flags |= 0x20
    mrad = int(round(rad * 1000.0))
    vc = int(round(abs(rpm) * 2 * math.pi / 60.0 * 100.0))
    mrad = max(-2147483648, min(2147483647, mrad))
    vc = max(-32768, min(32767, vc))
    return struct.pack("<ihB", mrad, vc, flags) + b"\x00"


def build_vel(rpm, run=True, release=False, clear=False):
    flags = 0
    if run:     flags |= 0x02
    if release: flags |= 0x04
    if clear:   flags |= 0x08
    vc = int(round(abs(rpm) * 2 * math.pi / 60.0 * 100.0))
    vc = max(-32768, min(32767, vc))
    return struct.pack("<hB", vc, flags) + b"\x00" * 5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["mon", "pos", "vel", "stop", "clear", "save", "raw", "ping",
                                       "enable", "disable", "home", "zero", "query", "faults", "fcfg",
                                       "swing", "seq"])
    ap.add_argument("--mech", action="store_true", help="0x100 的目标位置用机械坐标")
    ap.add_argument("--what", type=lambda x: int(x, 0), default=0xFF, help="0x113 问什么")
    ap.add_argument("--fault-on", type=int, default=1, help="0x114 b0")
    ap.add_argument("--period", type=int, default=10, help="0x114 b1, 单位 100ms")
    ap.add_argument("--turns", type=float)
    ap.add_argument("--deg", type=float)
    ap.add_argument("--rad", type=float)
    ap.add_argument("--rpm", type=float, default=30.0)
    ap.add_argument("--rel", action="store_true", help="相对当前位置")
    ap.add_argument("--release", action="store_true", help="停止时松轴")
    ap.add_argument("--secs", type=float, default=10.0)
    ap.add_argument("--id", type=lambda x: int(x, 0), default=0x100)
    ap.add_argument("--data", default="0000000000000000")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--every", type=int, default=1, help="每 N 帧打印一次反馈")
    ap.add_argument("--a", type=float, default=0.0, help="swing 近端 (机械坐标 rad)")
    ap.add_argument("--b", type=float, default=0.0, help="swing 远端 (机械坐标 rad)")
    ap.add_argument("--cycles", type=int, default=1, help="swing 来回次数")
    ap.add_argument("--timeout", type=float, default=8.0, help="单次走位等待超时 s")
    ap.add_argument("--save", default="swing_log.txt", help="swing 逐次数据输出文件")
    ap.add_argument("--dwell", type=float, default=0.0,
                    help="每段到位后停多久(s), 并在停顿结束时量一次静置落点")
    ap.add_argument("--blend", type=float, default=0.0,
                    help="seq 用: 距途经点还剩这么多 rad 就切下一段(速度不过零), 末段仍然精确到位")
    ap.add_argument("--segs", default="",
                    help="seq 用: 逗号分隔的 目标rad:rpm, 如 5.236:300,193.732:600,0:300")
    ap.add_argument("--preset", default="",
                    help="seq 用: 现成的模板名 (roundtrip/home/nudge, list 列表)")
    ap.add_argument("--dry-run", action="store_true",
                    help="seq 用: 只算分段打印出来, 一个字节都不发 (先看再发)")
    ap.add_argument("--minvel", type=float, default=0.0,
                    help="只打印 |速度| >= 该值(rad/s) 的反馈帧, 用来过滤适配器积压的静止帧")
    ap.add_argument("--no-poll", action="store_false", dest="poll", default=True,
                    help="不主动发 0x113 轮询 (只在板子自己会发 0x180 时才有用)")
    ap.add_argument("--poll-ms", type=float, default=10.0, help="0x113 轮询间隔 ms")
    a = ap.parse_args()

    global POLL, POLL_PERIOD
    POLL = a.poll
    POLL_PERIOD = max(1.0, a.poll_ms) / 1000.0

    # ---- seq: --preset 就是那个"标准命令"的入口; 全在这里解决, 不碰串口 ----
    if a.action == "seq":
        if a.preset == "list":
            print("seq 现成的模板 (段 = 机械坐标 rad : 速度上限 rpm):")
            for _n, _s, _d in PRESETS:
                print("  --preset %-10s %-34s %s" % (_n, _s, _d))
            print("  自己排: --segs \"目标rad:rpm,目标rad:rpm,...\"")
            return
        if a.preset:
            a.preset = PRESET_ALIAS.get(a.preset, a.preset)
            if a.segs:
                print("--preset 和 --segs 只能给一个")
                return
            if a.preset not in PRESET_DICT:
                print("没有这个 preset: %s   (--preset list 看现成的)" % a.preset)
                return
            a.segs = PRESET_DICT[a.preset]
        if not a.segs:
            print("seq 要么给 --preset <名字>, 要么给 --segs \"...\"")
            return
        if a.dry_run:
            print("dry-run: 只算不发 (%s)" % ("preset=" + a.preset if a.preset else "自己排的段"))
            _p0 = 0.0
            for _k, (_t, _r) in enumerate(parse_segs(a.segs), 1):
                print("  段 %d: 机械 %+.4f rad = %+.4f 圈 = %+.2f 度, 上限 %.0f rpm, 这一腿走 %+.4f rad"
                      % (_k, _t, _t / (2 * math.pi), math.degrees(_t), _r, _t - _p0))
                _p0 = _t
            print("  末段落点 = 机械 %+.4f rad (0 = 机械零点)" % _p0)
            return

    h = open_port(a.port)
    try:
        if a.action == "mon":
            print("listening %s for %.1fs (Ctrl-C to stop) ..." % (a.port, a.secs))
            n, seen = monitor(h, a.secs, every=a.every, minvel=a.minvel)
            print("feedback frames: %d   ids seen: %s"
                  % (n, ", ".join("0x%03X=%d" % kv for kv in sorted(seen.items())) or "-"))
            return

        if a.action == "ping":
            # 只发一帧, 看板子回不回 0x180
            send(h, ID_CMD_VEL, build_vel(0.0, run=False, release=True))
            print("sent VEL stop; listening 1.5s ...")
            monitor(h, 1.5)
            return

        if a.action == "enable":
            send(h, ID_CMD_ENABLE, bytes([1]))
            show_tx(ID_CMD_ENABLE, bytes([1]),
                    ["b0 = 1 -> 使能: 合 EN + 角度环保持当前位置"])
        elif a.action == "disable":
            send(h, ID_CMD_ENABLE, bytes([0]))
            show_tx(ID_CMD_ENABLE, bytes([0]),
                    ["b0 = 0 -> 失能: 断 EN, 输出高阻, 轴自由, 存一次位置"])
        elif a.action == "home":
            vc = int(round(abs(a.rpm) * 2 * math.pi / 60.0 * 100.0))
            vc = max(0, min(32767, vc))
            send(h, ID_CMD_HOME, struct.pack("<h", vc) + b"\x00" * 6)
            show_tx(ID_CMD_HOME, struct.pack("<h", vc) + b"\x00" * 6,
                    ["速度上限 %+d x0.01rad/s = %+.2f rpm (0 表示用默认 30rpm)" % (vc, a.rpm)])
        elif a.action == "zero":
            send(h, ID_CMD_SETZERO, b"\x00" * 8)
            show_tx(ID_CMD_SETZERO, b"\x00" * 8,
                    ["把此刻的位置记成机械坐标 0, 立即写 Flash"])
        elif a.action == "query":
            send(h, ID_CMD_QUERY, bytes([a.what & 0xFF]))
            show_tx(ID_CMD_QUERY, bytes([a.what & 0xFF]),
                    ["b0 = 0x%02X -> %s" % (a.what & 0xFF,
                     {0: "要状态帧 0x181", 1: "要配置帧 0x182", 2: "要故障帧 0x183",
                      0xFF: "三帧全要"}.get(a.what & 0xFF, "未知") )])
        elif a.action == "faults":
            send(h, ID_CMD_QUERY, bytes([2]))
            show_tx(ID_CMD_QUERY, bytes([2]), ["b0 = 2 -> 要故障帧 0x183"])
        elif a.action == "fcfg":
            send(h, ID_CMD_FAULTCFG, bytes([1 if a.fault_on else 0, a.period & 0xFF]))
            show_tx(ID_CMD_FAULTCFG, bytes([1 if a.fault_on else 0, a.period & 0xFF]),
                    ["b0 = %d -> 主动上报%s" % (a.fault_on, "开" if a.fault_on else "关"),
                     "b1 = %d -> 周期重报 %.1f s (0 = 只在故障变化时报)"
                     % (a.period, a.period * 0.1)])
        elif a.action == "pos":
            payload = build_pv(a.turns, a.deg, a.rad, a.rpm, a.rel, mech=a.mech)
            send(h, ID_CMD_PV, payload)
            show_tx(ID_CMD_PV, payload, dec_pv(payload))
        elif a.action == "vel":
            payload = build_vel(a.rpm)
            send(h, ID_CMD_VEL, payload)
            show_tx(ID_CMD_VEL, payload, dec_vel(payload))
        elif a.action == "stop":
            payload = build_pv(0.0, 0.0, 0.0, 0.0, run=False, release=a.release)
            send(h, ID_CMD_PV, payload)
            show_tx(ID_CMD_PV, payload, ["停止 (run=0), " + ("松轴 release" if a.release else "保持在原位")])
        elif a.action == "clear":
            payload = build_pv(0.0, 0.0, 0.0, 0.0, run=False, clear=True)
            send(h, ID_CMD_PV, payload)
            show_tx(ID_CMD_PV, payload, ["清堵转故障并重新使能 (bit3=1)"])
        elif a.action == "save":
            payload = build_pv(0.0, 0.0, 0.0, 0.0, run=False, save=True)
            send(h, ID_CMD_PV, payload)
            show_tx(ID_CMD_PV, payload, ["立刻把当前位置存进 Flash (bit4=1)"])
        elif a.action == "swing":
            ta = int(round(a.a * 1000.0))
            tb = int(round(a.b * 1000.0))
            total = a.cycles * 2
            rows = []
            print("swing: %d 个来回 (共 %d 次单程)  机械坐标 %+.4f <-> %+.4f rad  速度上限 %.0f rpm"
                  % (a.cycles, total, a.a, a.b, a.rpm))
            if a.dwell > 0.0:
                print("每段到位后停 %.1fs, 停稳后再量一次落点" % a.dwell)
            k = 0
            for _ in range(a.cycles):
                for tgt, nm in ((tb, "B"), (ta, "A")):
                    k += 1
                    payload = build_pv(rad=tgt / 1000.0, rpm=a.rpm, mech=True)
                    send(h, ID_CMD_PV, payload)
                    print("[%02d/%02d] TX 0x100  DATA %s   目标 %+.4f rad(机械) %s  上限 %.0f rpm"
                          % (k, total, payload.hex(" ").upper(), tgt / 1000.0, nm, a.rpm))
                    dt, pos, vmax, imax, st, _vret, ns = wait_arrive(h, tgt, a.timeout)
                    print("           -> %.2fs  峰值 %6.1f rpm  峰值电流 %.2f A  终点 %+.4f rad  样本 %d  %s"
                          % (dt, vmax * 60.0 / (2 * math.pi), imax, pos / 1000.0, ns,
                             ("[!! 一帧都没收到, 上面两个峰值是初值]"
                              if ns == 0 else
                              ("[arrived]" if st & 0x02 else "[!! 未到位/超时]"))))
                    rest = pos
                    if a.dwell > 0.0:
                        time.sleep(a.dwell)
                        rp, rv, rst, rc = read_pos(h, 0.6)
                        rest = rp
                        print("             停 %.1fs 后落点 %+.4f rad (距目标 %+.4f rad)  "
                              "vel=%+.3f rad/s  I=%+.2f A  %s"
                              % (a.dwell, rp / 1000.0, (rp - tgt) / 1000.0, rv, rc,
                                 "[到位]" if (rst & 0x02) else "[!! 没到位]"))
                    rows.append((k, tgt / 1000.0, dt, vmax * 60.0 / (2 * math.pi), imax,
                                 pos / 1000.0, rest / 1000.0))

            n = len(rows)
            print("---- 汇总 (%d 次单程) ----" % n)
            print("  到位耗时   min %.2fs  max %.2fs  均 %.2fs"
                  % (min(r[2] for r in rows), max(r[2] for r in rows),
                     sum(r[2] for r in rows) / n))
            print("  峰值速度   min %.1f rpm  max %.1f rpm  均 %.1f rpm"
                  % (min(r[3] for r in rows), max(r[3] for r in rows),
                     sum(r[3] for r in rows) / n))
            print("  峰值电流   max %.2f A" % max(r[4] for r in rows))
            print("  终点误差   max %.4f rad  (判定到位那一刻)"
                  % max(abs(r[5] - r[1]) for r in rows))
            print("  静置落点差 max %.4f rad  (停稳之后, 才是真本事)"
                  % max(abs(r[6] - r[1]) for r in rows))
            json_dump = None
            with io.open(a.save, "w", encoding="utf-8") as f:
                f.write("k\ttarget_rad\tdt_s\tvpeak_rpm\tipeak_A\tend_rad\trest_rad\n")
                for r in rows:
                    f.write("%d\t%.4f\t%.3f\t%.1f\t%.2f\t%.4f\t%.4f\n" % r)
            print("  逐次数据 -> %s" % a.save)
            return

        elif a.action == "seq":
            segs = parse_segs(a.segs)
            n = len(segs)
            rows = []
            print("seq: %d 段, 全部按机械坐标下发, 从当前机械坐标起步" % n)
            for k, (tgt_rad, rpm) in enumerate(segs, 1):
                tgt_mrad = int(round(tgt_rad * 1000.0))
                seg_len = tgt_rad - (float(segs[k - 2][0]) if k > 1 else 0.0)
                blend_eff = 0.0
                if a.blend > 0.0 and k < n:
                    blend_eff = min(a.blend, 0.5 * abs(seg_len))
                blend_mrad = int(round(blend_eff * 1000.0))
                payload = build_pv(rad=tgt_rad, rpm=rpm, mech=True)
                send(h, ID_CMD_PV, payload)
                show_tx(ID_CMD_PV, payload,
                        ["段 %d/%d   目标位置(机械) %+.4f rad = %+.4f 圈 = %+.2f 度"
                         % (k, n, tgt_rad, tgt_rad / (2 * math.pi), math.degrees(tgt_rad)),
                         "速度上限 %.0f rpm = %.3f rad/s" % (rpm, abs(rpm) * 2 * math.pi / 60.0)])
                dt, pos, vmax, imax, st, vret, ns = wait_arrive(h, tgt_mrad, a.timeout,
                                                            blend_mrad=blend_mrad)
                if blend_mrad > 0:
                    print("[%d/%d] -> %.2fs  峰值 %6.1f rpm (上限 %.0f)  峰值电流 %.2f A  "
                          "切换点 %+.4f rad (距目标 %+.4f rad), 切换时还有 %6.1f rpm, 没停"
                          % (k, n, dt, vmax * 60.0 / (2 * math.pi), rpm, imax, pos / 1000.0,
                             (pos - tgt_mrad) / 1000.0, vret * 60.0 / (2 * math.pi)))
                else:
                    print("[%d/%d] -> %.2fs  峰值 %6.1f rpm (上限 %.0f)  峰值电流 %.2f A  "
                          "终点 %+.4f rad  样本 %d  %s"
                          % (k, n, dt, vmax * 60.0 / (2 * math.pi), rpm, imax, pos / 1000.0, ns,
                             ("[!! 一帧都没收到, 上面两个峰值是初值]"
                              if ns == 0 else
                              ("[arrived]" if st & 0x02 else "[!! 未到位/超时]"))))
                rest = pos
                if a.dwell > 0.0:
                    time.sleep(a.dwell)
                    rp, rv, rst, rc = read_pos(h, 0.6)
                    rest = rp
                    print("        停 %.1fs 后落点 %+.4f rad (距目标 %+.4f rad)  vel=%+.3f rad/s  I=%+.2f A"
                          % (a.dwell, rp / 1000.0, (rp - tgt_mrad) / 1000.0, rv, rc))
                rows.append((k, tgt_rad, rpm, dt, vmax * 60.0 / (2 * math.pi), imax,
                             pos / 1000.0, rest / 1000.0))

            print("---- 汇总 (%d 段) ----" % n)
            last_hdr = "切换点rad" if (a.blend > 0.0 and n > 1) else "落点rad"
            print("  %-3s %-11s %-9s %-8s %-10s %-8s %-11s" %
                  ("段", "目标rad", "上限rpm", "耗时s", "峰值rpm", "峰值A", last_hdr))
            for r in rows:
                print("  %-3d %-11.4f %-9.0f %-8.2f %-10.1f %-8.2f %-11.4f"
                      % (r[0], r[1], r[2], r[3], r[4], r[5], r[7]))
            if a.blend > 0.0 and n > 1:
                print("  总耗时 %.1fs   最大峰值电流 %.2f A   末段落点误差 %.4f rad"
                      "  (前 %d 段是提前切换, 不算落点)"
                      % (sum(r[3] for r in rows), max(r[5] for r in rows),
                         abs(rows[-1][7] - rows[-1][1]), n - 1))
            else:
                print("  总耗时 %.1fs   最大峰值电流 %.2f A   最大落点误差 %.4f rad"
                      % (sum(r[3] for r in rows), max(r[5] for r in rows),
                         max(abs(r[7] - r[1]) for r in rows)))
            with io.open(a.save, "w", encoding="utf-8") as f:
                f.write("k\ttarget_rad\trpm_cap\tdt_s\tvpeak_rpm\tipeak_A\tend_rad\trest_rad\n")
                for r in rows:
                    f.write("%d\t%.4f\t%.0f\t%.3f\t%.1f\t%.2f\t%.4f\t%.4f\n" % r)
            print("  逐段数据 -> %s" % a.save)
            return

        elif a.action == "raw":
            data = bytes.fromhex(a.data.replace(" ", ""))
            send(h, a.id, data)
            print("TX id=0x%03X data=%s" % (a.id, data.hex(" ")))

        print("listening %.1fs ..." % a.secs)
        n, seen = monitor(h, a.secs, every=a.every, minvel=a.minvel)
        print("feedback frames: %d   ids seen: %s"
              % (n, ", ".join("0x%03X=%d" % kv for kv in sorted(seen.items())) or "-"))
    finally:
        kernel32.CloseHandle(h)


if __name__ == "__main__":
    main()
