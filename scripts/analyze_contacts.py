#!/usr/bin/env python3
"""Phan tich ~/contacts.txt (gz topic -e /gazebo/default/physics/contacts): luc tiep xuc giua ngon tay va cube."""
import math
import os
import re
import sys
from collections import Counter

path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/contacts.txt")
num = r"([-\d.eE+]+)"
re_c1 = re.compile(r'collision1:\s*"([^"]+)"')
re_c2 = re.compile(r'collision2:\s*"([^"]+)"')
re_n = re.compile(r"normal\s*\{\s*x:\s*" + num + r"\s*y:\s*" + num + r"\s*z:\s*" + num)
re_d = re.compile(r"depth:\s*" + num)
re_f = re.compile(r"body_1_wrench\s*\{\s*force\s*\{\s*x:\s*" + num + r"\s*y:\s*" + num + r"\s*z:\s*" + num)


def handle(block, pairs, rows):
    m1, m2 = re_c1.search(block), re_c2.search(block)
    if not (m1 and m2):
        return
    a, b = m1.group(1), m2.group(1)
    pairs[(a, b)] += 1
    if ("finger" in a + b) and ("cube" in a + b):
        n, d, f = re_n.search(block), re_d.search(block), re_f.search(block)
        if n and f:
            nv = tuple(float(v) for v in n.groups())
            fv = tuple(float(v) for v in f.groups())
            fn = sum(x * y for x, y in zip(fv, nv))
            ft = math.sqrt(max(0.0, sum(x * x for x in fv) - fn * fn))
            rows.append((a, b, abs(fn), ft, float(d.group(1)) if d else float("nan"), nv, fv))


pairs, rows, buf = Counter(), [], []
with open(path, errors="ignore") as fh:
    for line in fh:
        if line.lstrip().startswith("contact {"):
            if buf:
                handle("".join(buf), pairs, rows)
            buf = [line]
        elif buf:
            buf.append(line)
            if len(buf) > 80:  # khoi contact khong dai hon the
                handle("".join(buf), pairs, rows)
                buf = []
if buf:
    handle("".join(buf), pairs, rows)

print("so cap va cham khac nhau:", len(pairs), "| tong block:", sum(pairs.values()))
print("10 cap xuat hien nhieu nhat:")
for (a, b), c in pairs.most_common(10):
    print(f"  {c:7d}  {a}  <->  {b}")
fc = [(k, v) for k, v in pairs.items() if "finger" in k[0] + k[1] and "cube" in k[0] + k[1]]
print("cap finger-cube:", fc if fc else "KHONG CO")
if rows:
    rows.sort(key=lambda r: r[2])
    fn_med = rows[len(rows) // 2][2]
    ft_med = sorted(r[3] for r in rows)[len(rows) // 2]
    print(f"so mau finger-cube: {len(rows)}")
    print(f"luc phap tuyen (N): min {rows[0][2]:.3f} median {fn_med:.3f} max {rows[-1][2]:.3f}")
    print(f"luc tiep tuyen/ma sat (N): median {ft_med:.3f} max {max(r[3] for r in rows):.3f}")
    print("do xam nhap (m): median", sorted(r[4] for r in rows)[len(rows) // 2])
    print("vi du 3 mau lon nhat:")
    for r in rows[-3:]:
        print("  ", r[0], "<->", r[1], f"Fn={r[2]:.2f} Ft={r[3]:.2f} depth={r[4]:.5f} n={r[5]} F={r[6]}")


# ---- Dong thoi gian: tong luc finger-cube theo moi 0.5 giay (thoi gian mo phong), moi ngon ----
def timeline(path):
    re_sec = re.compile(r"^\s*sec:\s*(\d+)")
    re_nsec = re.compile(r"^\s*nsec:\s*(\d+)")
    t_cur, in_time, state = None, False, None
    sec = None
    buckets = {}  # (bucket, finger) -> [n, sum_fn, sum_ft, sum_|nz|, max_fn]
    blk = []

    def flush():
        if not blk or t_cur is None:
            return
        text = "".join(blk)
        m1, m2 = re_c1.search(text), re_c2.search(text)
        if not (m1 and m2):
            return
        names = m1.group(1) + " " + m2.group(1)
        if "finger" not in names or "cube" not in names:
            return
        n, f = re_n.search(text), re_f.search(text)
        if not (n and f):
            return
        nv = tuple(float(v) for v in n.groups())
        fv = tuple(float(v) for v in f.groups())
        fn = abs(sum(x * y for x, y in zip(fv, nv)))
        ft = math.sqrt(max(0.0, sum(x * x for x in fv) - fn * fn))
        which = "L" if "left_finger" in names else "R"
        key = (int(t_cur * 2), which)
        b = buckets.setdefault(key, [0, 0.0, 0.0, 0.0, 0.0])
        b[0] += 1
        b[1] += fn
        b[2] += ft
        b[3] += abs(nv[2])
        b[4] = max(b[4], fn)

    with open(path, errors="ignore") as fh:
        for line in fh:
            s = line.lstrip()
            if s.startswith("time {"):
                flush()
                blk = []
                in_time = True
                sec = None
                continue
            if in_time:
                ms, mn = re_sec.match(line), re_nsec.match(line)
                if ms:
                    sec = int(ms.group(1))
                elif mn and sec is not None:
                    t_cur = sec + int(mn.group(1)) * 1e-9
                    in_time = False
                continue
            if s.startswith("contact {"):
                flush()
                blk = [line]
            elif blk:
                blk.append(line)
                if len(blk) > 80:
                    flush()
                    blk = []
    flush()
    if not buckets:
        print("khong co du lieu theo thoi gian")
        return
    t0 = min(k[0] for k in buckets)
    print("\nDONG THOI GIAN (moi 0.5 giay mo phong; N=so mau, Fn=tong luc phap tuyen TB, Ft=ma sat TB, |nz|=thanh phan doc cua phap tuyen)")
    print("  t(s)  ngon   N     Fn_tb    Ft_tb   |nz|_tb  Fn_max")
    for key in sorted(buckets):
        n, sfn, sft, snz, mx = buckets[key]
        print(f"  {(key[0] - t0) / 2:5.1f}   {key[1]}   {n:5d}  {sfn / n:7.2f}  {sft / n:7.2f}   {snz / n:5.2f}   {mx:7.2f}")


timeline(path)
