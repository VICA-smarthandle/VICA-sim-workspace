#!/usr/bin/env python3
"""Read width sweep results and refuse to put incomparable ones in one table.

    ros2 run vica_description width_report /tmp/vica_width_results/*/
    ros2 run vica_description width_report --force <dirs...>

Every table this prints is one course, so a row from one sweep can be read
against a row from another. That is not a formatting preference. It is the
whole reason this file exists:

    A DWB sweep ran at 20:10. The course was rebuilt at 20:26 -- three widths
    were added and every lane moved. An MPPI sweep ran at 22:58. The two were
    then compared, and "1.20 m" meant a goal at x=6.10 in one and x=11.20 in
    the other. DWB scored 3/3 and MPPI 0/3, which reads as a controller
    result and is not one. Nothing in either table said which course it was
    about.

So the stamp travels with the numbers and this refuses to mix stamps. --force
prints anyway, with the mismatch spelled out, for when you know what you are
looking at.

It also prints the spread of actual start positions, because that number
decides whether a row means anything. Where the three repeats started within
0.1 m of each other the results were identical every time -- 3/3 reached, or
3/3 stuck. Where they started metres apart the results were mixed, and the
mixture was about the start, not the lane. A 1.40 m lane scored worse than a
1.20 m one, which is physically impossible and was the signal that the start
had moved 12 m between repeats.
"""

import argparse
import collections
import glob
import json
import math
import os
import sys

# Above this, the three repeats did not begin in the same place and the row
# is about the start rather than the lane. 0.1 m is what a clean set produced.
SPREAD_TRUSTED = 0.30


def load(dirs):
    runs = []
    for d in dirs:
        for path in sorted(glob.glob(os.path.join(d, "*.json"))):
            try:
                runs.append((path, json.load(open(path))))
            except (ValueError, OSError) as exc:
                print(f"  건너뜀 {path}: {exc}", file=sys.stderr)
    return runs


def group_key(payload):
    """What must match for two runs to belong in the same table."""
    return payload.get("course_stamp")


def summarise(runs):
    """width -> list of records, plus the start spread for that width."""
    by_width = collections.defaultdict(list)
    for _, p in runs:
        for r in p.get("records", []):
            by_width[p.get("width")].append(r)
    out = []
    for w in sorted(by_width, key=lambda v: (v is None, v), reverse=True):
        recs = by_width[w]
        starts = [r["from"] for r in recs if r.get("from")]
        spread = 0.0
        if len(starts) > 1:
            spread = max(math.dist(a, b) for a in starts for b in starts)
        reached = sum(1 for r in recs if r.get("result") == "reached")
        out.append((w, reached, len(recs), spread, recs))
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+", help="result directories")
    ap.add_argument("--force", action="store_true",
                    help="print even when the runs drove different courses")
    args = ap.parse_args()

    runs = load(args.dirs)
    if not runs:
        print("  결과가 없습니다.", file=sys.stderr)
        return 1

    groups = collections.defaultdict(list)
    for path, p in runs:
        groups[group_key(p)].append((path, p))

    unstamped = groups.pop(None, [])
    if unstamped:
        # The unstamped runs are not stray files. They are width_sweep.sh's own
        # failure stubs: it writes {"result": "refused"} or {"result":
        # "nav2-down"} when width_trials produced no file, and those stubs
        # carry no course_stamp because the sweep never had one to copy.
        #
        # Dropping them is the defect this module exists to prevent, pointed
        # the other way. A width whose three repeats were two refusals and one
        # pass printed 1/1 -- a perfect score built out of the two runs that
        # were thrown away. The docstring's complaint about a sweep returning
        # five widths out of nine reading like a sweep that found five widths
        # is the same failure at the row level.
        #
        # They can be placed when there is only one course in the set: a stub
        # sitting beside runs that all name the same stamp came off that
        # course. With more than one stamp there is nothing to place them
        # against, so they stay out and the count is printed instead.
        if len(groups) == 1:
            only = next(iter(groups))
            groups[only].extend(unstamped)
            print(f"  course_stamp 없는 결과 {len(unstamped)}건 -- 이 결과 묶음의 "
                  f"코스가 하나뿐이므로 그 코스의 실패로 셉니다.")
            print(f"          (width_sweep.sh 가 결과 파일이 안 나왔을 때 남기는 "
                  f"refused / nav2-down 기록입니다)")
        else:
            print(f"  [경고] course_stamp 없는 결과 {len(unstamped)}건 -- "
                  f"코스가 둘 이상이라 어느 쪽인지 알 수 없어 제외합니다. "
                  f"통과율의 분모가 그만큼 작습니다.")
        for path, _ in unstamped[:4]:
            print(f"          {os.path.basename(os.path.dirname(path))}/"
                  f"{os.path.basename(path)}")
        if len(unstamped) > 4:
            print(f"          ... 외 {len(unstamped) - 4}건")
        print()

    if len(groups) > 1 and not args.force:
        print("  서로 다른 코스에서 나온 결과입니다. 한 표에 넣지 않습니다.\n")
        for stamp, rs in groups.items():
            widths = sorted({p.get("width") for _, p in rs},
                            key=lambda v: (v is None, v))
            ctrls = sorted({p.get("controller") for _, p in rs})
            print(f"    코스 {stamp}")
            print(f"      컨트롤러 {', '.join(str(c) for c in ctrls)}   "
                  f"폭 {', '.join(f'{w}' for w in widths)}   {len(rs)}회")
        print("\n  같은 코스에서 다시 측정하거나, 알고 보는 것이면 --force.")
        return 2

    for stamp, rs in sorted(groups.items(), key=lambda kv: str(kv[0])):
        by_ctrl = collections.defaultdict(list)
        for path, p in rs:
            by_ctrl[(p.get("controller"), _infl(p))].append((path, p))

        print(f"\n=== 코스 {stamp}")
        for (ctrl, infl), crs in sorted(by_ctrl.items(), key=lambda kv: str(kv[0])):
            print(f"\n--- {ctrl}   inflation {infl}")
            print(f"    {'폭':>6}  {'통과':>5}  {'출발산포':>8}   결과")
            for w, ok, n, spread, recs in summarise(crs):
                flag = "" if spread <= SPREAD_TRUSTED else "  <- 출발이 흩어짐"
                detail = " ".join(
                    f"{r.get('result')}({r.get('moved_m')})" for r in recs)
                wtxt = f"{w:.2f}" if isinstance(w, (int, float)) else str(w)
                print(f"    {wtxt:>6}  {ok:>2}/{n:<2}  {spread:7.2f} m   "
                      f"{detail}{flag}")

    print(f"\n  출발산포 {SPREAD_TRUSTED} m 초과 행은 폭이 아니라 출발 위치를 "
          f"측정한 것입니다.")
    return 0


def _infl(payload):
    # width_trials writes "inflation_radius"; width_sweep.sh's failure stubs
    # wrote "inflation" for a while. Reading only the first put the stubs in a
    # row of their own labelled 미기록, so a width with one pass and two
    # refusals printed 1/1 beside 0/2 instead of 1/3 -- the refusals were
    # counted and still did not reach the number anyone reads.
    v = payload.get("inflation_radius", payload.get("inflation"))
    if v is None:
        return "미기록"
    if isinstance(v, list):
        return f"불일치 {v}"
    return f"{v}"


if __name__ == "__main__":
    sys.exit(main())
