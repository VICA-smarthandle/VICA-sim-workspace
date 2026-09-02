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

# course : wide window : closeup centre x,y : closeup window
declare -A WIDE=( [vica_cornercourse]=9 [vica_avoidcourse]=9 [vica_uturncourse]=9 )
declare -A NEAR=( [vica_cornercourse]=5 [vica_avoidcourse]=5 [vica_uturncourse]=6 )

COURSES=${1:-"vica_cornercourse vica_avoidcourse vica_uturncourse"}

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

    # Closeup centre: the goal is where the difficulty is on every course
    # built here, so it doubles as the thing to frame.
    ros2 run vica_description animate_run --width "$WIDTH" \
        --left "$ROOT/$course/$best" \
        --stage "$stage" --focus goal --window "${NEAR[$course]:-5}" \
        --out "$OUT/${short}_${WIDTH}_closeup" 2>&1 | grep -E "wrote|프레임"
done

echo
echo "=== $OUT"
