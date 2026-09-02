#!/usr/bin/env python3
"""Read a controller x planner result tree and rank the cells.

    matrix_report.py <results/matrix> [--course NAME] [--csv FILE]

Prints one table per planner, the three controllers side by side, then the
per-course detail underneath.

Ranks on arrival first
----------------------
The physical robot's controller day (devlog 2026-08-29) ended with MPPI
winning nearly every quality metric and being rejected anyway, because at the
toilet goal DWB was 2/2 and MPPI 0/4. The written conclusion was that for a
guide robot arrival is not negotiable. So `reached` is the first sort key here
and everything else breaks ties, rather than all of them going into one score
that can trade a goal away for smoothness.

What the columns are
--------------------
    reached     goals the action returned success for
    stuck       failed having moved more than 0.1 m: got in, could not finish
    no-start    failed having moved less than that: planner refused, or the
                controller never produced a command
    timeout     neither, inside the limit
    nav2-down   the stack did not come up; not a result about driving, and
                counted separately so it cannot be read as one

A cell with nav2-down passes is reported with a warning rather than dropped.
Silently dropping them is how a sweep that measured five widths out of nine
came to look like a sweep that found five widths.
"""

import argparse
import collections
import csv
import json
import os
import sys

ORDER = ("reached", "stuck", "no-start", "timeout", "nav2-down", "other")


def cells(root):
    """Every result file, keyed by (course, controller, planner)."""
    out = collections.defaultdict(list)
    for course in sorted(os.listdir(root)):
        cdir = os.path.join(root, course)
        if not os.path.isdir(cdir):
            continue
        for cell in sorted(os.listdir(cdir)):
            d = os.path.join(cdir, cell)
            if not os.path.isdir(d):
                continue
            # <controller>_<planner>_infl<value>
            parts = cell.split("_")
            if len(parts) < 3:
                continue
            controller, planner = parts[0], parts[1]
            for f in sorted(os.listdir(d)):
                if not f.endswith(".json"):
                    continue
                try:
                    data = json.load(open(os.path.join(d, f)))
                except Exception as exc:
                    print(f"  [건너뜀] {f}: {exc}", file=sys.stderr)
                    continue
                out[(course, controller, planner)].append(data)
    return out


def tally(runs):
    counts = collections.Counter()
    widths = set()
    for r in runs:
        w = r.get("width")
        if w is not None:
            widths.add(float(w))
        for rec in r.get("records", []):
            counts[rec.get("result", "other")] += 1
    return counts, widths


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root")
    ap.add_argument("--course", default=None, help="only this course")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    if not os.path.isdir(args.root):
        raise SystemExit(f"결과 디렉터리가 없습니다: {args.root}")

    data = cells(args.root)
    if not data:
        raise SystemExit(f"결과가 없습니다: {args.root}")

    rows = []
    for (course, controller, planner), runs in data.items():
        if args.course and course != args.course:
            continue
        counts, widths = tally(runs)
        total = sum(counts.values())
        rows.append({
            "course": course,
            "controller": controller,
            "planner": planner,
            "passes": total,
            "widths": len(widths),
            **{k: counts.get(k, 0) for k in ORDER},
        })

    # Arrival first, then fewer no-starts, then fewer stucks.
    rows.sort(key=lambda r: (-r["reached"], r["no-start"], r["stuck"]))

    # The primary view: one table per planner, the three controllers side by
    # side across every course.
    #
    # Asked for in that shape because that is the decision being made. The
    # controller and the planner are not independent -- Lattice proposes
    # in-place rotations that DWB structurally will not execute, and Hybrid
    # proposes none at all -- so "which controller is best" has no answer
    # except per planner. A flat ranking hides exactly the interaction the
    # experiment exists to find.
    courses = sorted({r["course"] for r in rows})
    index = {(r["planner"], r["controller"], r["course"]): r for r in rows}
    planners = sorted({r["planner"] for r in rows})
    controllers = [c for c in ("dwb", "rpp", "mppi")
                   if any(r["controller"] == c for r in rows)]

    short = {c: c.replace("vica_", "").replace("course", "") for c in courses}
    W = 13

    for planner in planners:
        print(f"\n=== planner: {planner}")
        head = f"  {'controller':12s}"
        for c in courses:
            head += f"{short[c]:>{W}s}"
        head += f"{'합계':>9s}"
        print(head)
        print(f"  {'':12s}" + "".join(f"{'도달/시도':>{W}s}" for _ in courses)
              + f"{'':>9s}")
        for ctrl in controllers:
            line = f"  {ctrl:12s}"
            tr = ta = 0
            for c in courses:
                r = index.get((planner, ctrl, c))
                if r is None:
                    line += f"{'-':>{W}s}"
                    continue
                a = r["reached"] + r["stuck"] + r["no-start"] + r["timeout"]
                tr += r["reached"]; ta += a
                cell = f"{r['reached']}/{a}"
                if r["nav2-down"]:
                    cell += "!"
                line += f"{cell:>{W}s}"
            pct = f"{100*tr/ta:.0f}%" if ta else "-"
            print(line + f"{tr}/{ta} {pct:>4s}".rjust(9))
        # Only worth printing when something actually failed to come up.
        if any(index.get((planner, ctrl, c), {}).get("nav2-down")
               for ctrl in controllers for c in courses):
            print("    ! = nav2 가 뜨지 않은 회차 포함 (주행 결과 아님)")

    print("\n" + "=" * 60)
    print("코스별 상세 (도달 1순위 정렬)")

    by_course = collections.defaultdict(list)
    for r in rows:
        by_course[r["course"]].append(r)

    for course, rs in by_course.items():
        print(f"\n=== {course}")
        print(f"  {'controller':11s}{'planner':10s}{'회차':>5s}{'도달':>6s}"
              f"{'갇힘':>6s}{'못출발':>8s}{'시간초과':>9s}{'nav2다운':>10s}")
        for r in rs:
            warn = "  <- nav2 안 뜸" if r["nav2-down"] else ""
            print(f"  {r['controller']:11s}{r['planner']:10s}{r['passes']:5d}"
                  f"{r['reached']:6d}{r['stuck']:6d}{r['no-start']:8d}"
                  f"{r['timeout']:9d}{r['nav2-down']:10d}{warn}")

    # The screen's actual output: which cells arrived anywhere at all.
    survivors = collections.Counter()
    attempts = collections.Counter()
    for r in rows:
        key = f"{r['controller']}:{r['planner']}"
        survivors[key] += r["reached"]
        attempts[key] += r["reached"] + r["stuck"] + r["no-start"] + r["timeout"]
    print("\n=== 조합별 합계 (도달 / 시도)")
    for key, n in sorted(survivors.items(), key=lambda kv: -kv[1]):
        a = attempts[key]
        pct = f"{100*n/a:.0f}%" if a else "-"
        print(f"  {key:16s} {n:3d} / {a:3d}   {pct}")

    if args.csv:
        with open(args.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {args.csv}")


if __name__ == "__main__":
    main()
