#!/usr/bin/env bash
# Turn a finished screening into the pictures that go in the report.
#
#     matrix_media.sh                 every course, best against worst
#     matrix_media.sh vica_uturncourse
#
# Two renders per course, because they answer different questions:
#
#   compare   the best and worst combination side by side, framed wide enough
#             to hold the whole course feature. This is the one that shows a
#             difference.
#   closeup   the best combination alone, framed tight on the thing being
#             negotiated, so the robot is large enough to see which way it is
#             pointing and how much room it had.
#
# Framing is per course and not a default, because what has to be visible
# differs: the corner in the corner course, the obstacle in the avoid course,
# the turn in the U-turn course. animate_run's default fits the whole run into
# the frame, which over a 27 m approach leaves the robot four pixels wide.
#
# No set -u: ROS's setup.bash reads unset variables and aborts with no output.
set -o pipefail

PKG=${VICA_WS:-$HOME/VICA-smarthandle/vica_ws}
ROOT=${RESULTS_ROOT:-$PKG/results/matrix}
OUT=${MEDIA_ROOT:-$PKG/media/matrix}
WIDTH=${SCREEN_WIDTH:-1.20}

cd "$PKG" || exit 1
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1
SHARE=$(ros2 pkg prefix vica_description)/share/vica_description
mkdir -p "$OUT"

declare -A WIDE=( [vica_cornercourse]=9 [vica_avoidcourse]=9 [vica_uturncourse]=10 )
declare -A NEAR=( [vica_cornercourse]=5 [vica_avoidcourse]=4 [vica_uturncourse]=5 )

# What the closeup is centred on, per course, read out of the course's own
# spec rather than guessed.
#
# The goal is the right centre for the corner course and the wrong one for the
# other two. On the avoid course the goal is 1.2 m past the block, so framing
# on it left the obstacle out of shot entirely -- a picture of a robot arriving
# somewhere, with the thing it had to get past off the edge. On the U-turn
# course the goal is back at the mouth and the turn happens at the dead end.
focus_xy() {   # $1 course
    python3 - "$SHARE/isaac_vica_assets/$1.json" "$WIDTH" <<'PY2'
import json, sys
spec = json.load(open(sys.argv[1]))
w = float(sys.argv[2])
lane = next((l for l in spec.get("lanes", [])
             if abs(float(l.get("width", -1)) - w) < 1e-6), None)
if lane is None:
    print("")
    raise SystemExit
if "gap_centre_x" in lane:                       # avoid: beside the block
    y = spec["block"]["y"]
    print(f'{lane["gap_centre_x"]},{(y[0] + y[1]) / 2.0}')
elif lane.get("turn") == "uturn":                # uturn: where it turns
    # The entry pose, not half a corridor further in. deadend_len measures the
    # corridor from its mouth, and adding half of it put the frame above the
    # dead end wall with the turn itself down in a corner.
    print(",".join(str(v) for v in lane["entry"]))
else:                                            # corner: at the corner
    print(",".join(str(v) for v in lane["exit"]))
PY2
}

COURSES=${1:-"vica_cornercourse vica_avoidcourse vica_uturncourse"}
# One clip per planner, the three controllers side by side.
#
# The best-against-worst clip shows that the combination matters. It does not
# show which controller to pick, because the answer is per planner: Lattice
# proposes in-place rotations DWB will not execute and Hybrid proposes none,
# so a controller ranked across all four planners at once is an average of
# four different questions. These are the tables in picture form.
PLANNERS=${PLANNERS:-"hybrid lattice navfn smac2d"}
CONTROLLERS=${CONTROLLERS:-"dwb rpp mppi"}
BY_PLANNER=${BY_PLANNER:-1}

# Rank the cells of one course: arrivals first, then how far they got.
rank() {   # $1 course   prints "<cell> <reached> <moved>" best first
    python3 - "$ROOT/$1" "$WIDTH" <<'PY'
import glob, json, os, sys
root, width = sys.argv[1], sys.argv[2]
rows = []
for d in sorted(glob.glob(os.path.join(root, "*_infl*"))):
    got = reach = 0
    moved = []
    for f in glob.glob(os.path.join(d, f"lane_{width}_r*.json")):
        for rec in json.load(open(f)).get("records", []):
            got += 1
            reach += rec.get("result") == "reached"
            if isinstance(rec.get("moved_m"), (int, float)):
                moved.append(rec["moved_m"])
    if got:
        rows.append((os.path.basename(d), reach,
                     sum(moved) / len(moved) if moved else 0.0))
rows.sort(key=lambda r: (-r[1], -r[2]))
for name, reach, m in rows:
    print(f"{name} {reach} {m:.2f}")
PY
}

for course in $COURSES; do
    [ -d "$ROOT/$course" ] || { echo "  $course: 결과 없음, 건너뜀"; continue; }
    mapfile -t ranked < <(rank "$course")
    [ "${#ranked[@]}" -ge 1 ] || { echo "  $course: 통과 기록 없음"; continue; }
    best=$(echo "${ranked[0]}" | awk '{print $1}')
    worst=$(echo "${ranked[-1]}" | awk '{print $1}')
    stage=$SHARE/isaac_vica_assets/$course.usd
    short=${course#vica_}; short=${short%course}
    echo
    echo "=== $course   최고 $best   최저 $worst"

    if [ "$best" != "$worst" ]; then
        ros2 run vica_description animate_run --width "$WIDTH" \
            --left "$ROOT/$course/$best" --right "$ROOT/$course/$worst" \
            --stage "$stage" --focus goal --window "${WIDE[$course]:-9}" \
            --out "$OUT/${short}_${WIDTH}_compare" 2>&1 | grep -E "wrote|프레임"
    fi

    if [ "$BY_PLANNER" = "1" ]; then
        pxy=$(focus_xy "$course")
        for planner in $PLANNERS; do
            panels=()
            for ctrl in $CONTROLLERS; do
                d="$ROOT/$course/${ctrl}_${planner}_infl${INFL:-0.55}"
                [ -d "$d" ] || d=$(ls -d "$ROOT/$course/${ctrl}_${planner}_infl"* 2>/dev/null | head -1)
                [ -n "$d" ] && [ -d "$d" ] && panels+=("$d")
            done
            [ "${#panels[@]}" -ge 2 ] || { echo "  $planner: 패널 부족, 건너뜀"; continue; }
            args=(--left "${panels[0]}")
            [ "${#panels[@]}" -ge 2 ] && args+=(--right "${panels[1]}")
            for extra in "${panels[@]:2}"; do args+=(--panel "$extra"); done
            [ -n "$pxy" ] && args+=(--focus point "--focus-xy=$pxy") || args+=(--focus goal)
            echo "  --- $planner: ${#panels[@]} 패널"
            ros2 run vica_description animate_run --width "$WIDTH" \
                "${args[@]}" --stage "$stage" --window "${NEAR[$course]:-5}" \
                --out "$OUT/${short}_${WIDTH}_${planner}" 2>&1 | grep -E "\.gif"
        done
    fi

    xy=$(focus_xy "$course")
    if [ -n "$xy" ]; then
        # "--focus-xy=..." and not "--focus-xy ...". The U-turn course's
        # centre is at x -3.2, and argparse reads a following value that
        # starts with a minus as another option unless it parses as a number.
        # "-3.2,5.0" does not, so the separated form fails with "expected one
        # argument" and the closeup silently never got made.
        focus=(--focus point "--focus-xy=$xy")
    else
        focus=(--focus goal)
    fi
    ros2 run vica_description animate_run --width "$WIDTH" \
        --left "$ROOT/$course/$best" \
        --stage "$stage" "${focus[@]}" --window "${NEAR[$course]:-5}" \
        --out "$OUT/${short}_${WIDTH}_closeup" 2>&1 | grep -E "wrote|프레임"
done

echo
echo "=== $OUT"
