#!/usr/bin/env python3
"""Replay tof_safety_capture.py recordings (per-zone classes) through candidate table-edge rules (2026-10-26).

The record of why the governor has no edge stop (removed 2026-10-27): the continuity rule below calls the real
edges 1.2-1.4 s early, but run over whole recordings it also stops while standing on solid table -- row 6 meets the
surface at ~11 deg and goes blank as the body rocks back past ~4 deg (see rider_tof_safety.py's docstring).

Per frame: cls = 64 chars, row 0 far .. row 7 near, col 0 = robot's left. 'f' floor, 'd' dropped, 'b' just below
floor, '.' no return, 'o' obstacle, 'h' overhead. Center columns 2-5 are the drive corridor.

Continuity rule: in a corridor column whose near row (7) is floor, the surface ENDS if the next row out (6), or the
one after (5, variant B), is not floor and not an obstacle (d / b / .). Edge = >= NCOL such columns, for DEB
consecutive frames. Reported per drive: when each rule would first have called the edge, relative to the drop
(first frame with |pitch - upright| > 8 deg while driving), and the old rule's first 'would = 0'.

Run:  /usr/bin/python3 tools/tof/tof_edge_replay.py data/captures/<tof_raw_capture recording>.csv [more.csv]
"""
import csv
import sys

COLS = (2, 3, 4, 5)
END = set("db.")


def col_ends(c, j, rows):
    if c[7 * 8 + j] != "f":
        return False
    for i in rows:
        z = c[i * 8 + j]
        if z == "f":
            continue                     # surface goes on past this row
        return z in END                  # first non-floor zone outward: an end (not an obstacle)
    return False


def first_call(R, i0, i1, rows, ncol, deb):
    run = 0
    for i in range(i0, i1):
        n = sum(col_ends(R[i]["cls"], j, rows) for j in COLS)
        run = run + 1 if n >= ncol else 0
        if run >= deb:
            return i
    return None


for f in sys.argv[1:]:
    R = list(csv.DictReader(open(f)))
    print("===", f.split("/")[-1])
    i = 0
    while i < len(R):
        if float(R[i]["vdes"] or 0) <= 0.05:
            i += 1
            continue
        s = i
        while i < len(R) and float(R[i]["vdes"] or 0) > 0.05:
            i += 1
        if i - s < 8:
            continue
        drop = next((k for k in range(s, min(len(R), i + 5)) if R[k]["th"] and abs(float(R[k]["th"]) - 3.5) > 8),
                    None)
        if drop is None:
            print(f"  drive {s}-{i}: no drop")
            continue
        td = float(R[drop]["t"])
        old = next((k for k in range(s, drop) if float(R[k]["would"]) < 0.01), None)
        v = sum(float(R[k]["vdes"]) for k in range(s, drop)) / (drop - s)
        print(f"  drive {s}-{i}, drop at frame {drop}, mean speed {v:.2f} m/s; "
              + (f"old rule first stop {td - float(R[old]['t']):.2f} s before" if old is not None
                 else "old rule never"))
        for name, rows in (("A row 6", (6,)), ("B rows 6-5", (6, 5))):
            for ncol in (2, 3):
                for deb in (1, 2):
                    k = first_call(R, s, drop + 1, rows, ncol, deb)
                    print(f"    {name:10s} cols>={ncol} frames={deb}: "
                          + (f"{td - float(R[k]['t']):.2f} s before the drop" if k is not None else "never"))
