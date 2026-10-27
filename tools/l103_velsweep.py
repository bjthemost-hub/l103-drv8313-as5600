# -*- coding: utf-8 -*-
"""分档升速扫描: 找 20V 母线 + 当前管压限幅下的实际转速天花板
用法: python l103_velsweep.py <port> <rpm1,rpm2,...> <每档保持秒数> <存档文件>
"""
import ctypes, io, math, sys, time

import l103_can as L

kernel32 = L.kernel32
wt = L.wt


def collect(h, secs):
    t0 = time.time()
    pending = bytearray()
    nfb = 0
    vmax = 0.0
    imax = 0.0
    samples = []
    st_last = 0
    faults = []
    while True:
        if time.time() - t0 >= secs:
            break
        n = wt.DWORD(0)
        tmp = ctypes.create_string_buffer(4096)
        ok = kernel32.ReadFile(h, tmp, 4096, ctypes.byref(n), None)
        if not ok or not n.value:
            time.sleep(0.002)
            continue
        pending += tmp.raw[:n.value]
        i = 0
        while i + 15 < len(pending):
            if pending[i] != 0xAA or pending[i + 15] != 0x55:
                i += 1
                continue
            cmd = pending[i + 1]
            cid = int.from_bytes(pending[i + 3:i + 7], "little")
            data = bytes(pending[i + 7:i + 15])
            i += 16
            if cmd != 0x11:
                continue
            if cid in (0x180, 0x181) and len(data) == 8:
                pos, vel, st, cur = L.decode_fb(data)
                v = vel / 100.0
                nfb += 1
                samples.append((time.time() - t0, v, cur / 10.0, st, pos / 1000.0))
                if abs(v) > abs(vmax):
                    vmax = v
                if abs(cur / 10.0) > imax:
                    imax = abs(cur / 10.0)
                st_last = st
            elif cid in (0x183, 0x1A0) and len(data) == 8:
                faults.append((cid, data.hex(" ")))
        del pending[:i]
    return samples, vmax, imax, st_last, faults


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    targets = [float(x) for x in sys.argv[2].split(",")]
    hold = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    save = sys.argv[4] if len(sys.argv) > 4 else "velsweep.txt"

    h = L.open_port(port)
    time.sleep(0.3)
    kernel32.PurgeComm(h, 0x0004 | 0x0008)
    # 适配器内部还有自己的缓冲, 开端口后会一次性吐出来 (2026-09-19 实测),
    # 先干收 2s 把这波旧帧排空, 否则会把真实数据的峰值带偏。
    collect(h, 2.0)

    L.send(h, L.ID_CMD_PV, L.build_pv(0.0, 0.0, 0.0, 0.0, run=False, clear=True))
    time.sleep(0.4)
    L.send(h, L.ID_CMD_ENABLE, bytes([1]))
    time.sleep(0.8)

    rows = []
    print("分档升速: %s   每档 %.1fs" % (targets, hold))
    for tgt in targets:
        L.send(h, L.ID_CMD_VEL, L.build_vel(tgt))
        time.sleep(0.05)
        smp, vmax, imax, st, flt = collect(h, hold)
        n = len(smp)
        tail = [s[1] for s in smp if s[0] > hold * 0.5]
        vavg = sum(tail) / len(tail) if tail else 0.0
        en = (st & 0x40) != 0
        stall = (st & 0x04) != 0
        print("  目标 %6.0f rpm | 帧 %4d | 峰值 %7.1f rpm | 后半均值 %7.1f rpm | "
              "峰值I %4.1f A | en=%d stall=%d" %
              (tgt, n, vmax * 60.0 / (2 * math.pi), vavg * 60.0 / (2 * math.pi), imax,
               1 if en else 0, 1 if stall else 0))
        for cid, hx in flt:
            print("      !! 0x%03X %s" % (cid, hx))
        for s in smp:
            if s[0] > hold * 0.5:
                rows.append((tgt, s[0], s[1] * 60.0 / (2 * math.pi), s[2], s[3], s[4]))
        if not en:
            print("  -> 已失能, 停止扫描")
            break

    L.send(h, L.ID_CMD_VEL, L.build_vel(0.0, run=False))
    time.sleep(0.3)
    L.send(h, L.ID_CMD_ENABLE, bytes([1]))
    print("已停车并重新使能(角度环保持当前位置)")

    with io.open(save, "w", encoding="utf-8", newline="") as f:
        f.write("rpm_target\tt_s\trpm\tI_A\tstatus\tpos_rad\n")
        for r in rows:
            f.write("%.0f\t%.3f\t%.2f\t%.2f\t%d\t%.4f\n" % r)
    print("稳态数据 -> %s" % save)


main()
