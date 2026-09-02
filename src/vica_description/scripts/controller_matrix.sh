#!/usr/bin/env bash
# Drive every controller against every planner, and write one result tree.
#
#     controller_matrix.sh screen              stage 1: one width, three courses
#     controller_matrix.sh depth dwb:lattice mppi:hybrid
#                                              stage 2: named cells, all widths
#     WIDTHS="1.00 1.20" controller_matrix.sh depth dwb:lattice
#
# Two stages, because the full grid does not fit in a day
# -------------------------------------------------------
# Twelve cells (3 controllers x 4 planners) over three courses at five widths
# with three repeats is 540 passes. width_sweep.sh restarts the simulator for
# every pass -- about five minutes -- so that grid is 45 hours of GPU.
#
# So: screen, then go deep. Stage 1 runs all twelve cells at ONE width on all
# three courses, 36 passes, about three hours. Cells that cannot arrive at a
# width the geometry says is comfortable are not going to become interesting
# at a tighter one. Stage 2 takes the survivors through the full width sweep.
#
# This is how the physical robot's own controller day went (devlog
# 2026-08-29): four MPPI runs and three RPP runs to decide, not a grid.
#
# The screening width
# -------------------
# 1.20 m by default. The padded circumscribed diameter is 0.945, so 1.20 has
# 255 mm of daylight -- wide enough that a failure is about the controller,
# tight enough that it is not a car park. 1.00 was the other candidate and it
# is only 55 mm clear, which is the region where a pass is closer to a coin
# toss than a measurement.
#
# What a cell has to do to survive
# --------------------------------
# Arrive. The robot's own conclusion about MPPI was that its quality numbers
# beat DWB nearly everywhere and it still did not reach the goal, and that for
# a guide robot arrival is not negotiable. So the screen ranks on reached
# first and everything else second. summarise.py prints it that way.
#
# No set -u: ROS's setup.bash reads unset variables and aborts with no output.
set -o pipefail

PKG=${VICA_WS:-$HOME/VICA-smarthandle/vica_ws}
SWEEP="$PKG/src/vica_description/scripts/width_sweep.sh"
MODE="${1:?usage: controller_matrix.sh <screen|depth> [cell...]}"
shift

CONTROLLERS=${CONTROLLERS:-"dwb rpp mppi"}
PLANNERS=${PLANNERS:-"lattice hybrid navfn smac2d"}
SCREEN_WIDTH=${SCREEN_WIDTH:-1.20}
ROOT=${RESULTS_ROOT:-$PKG/results/matrix}

# Course -> the widths that course actually carries. The U-turn course starts
# at 1.00 because below the circumscribed diameter of 0.945 the answer is
# arithmetic, not a measurement; the other two go down to 0.80 because driving
# through a gap is not turning around in it.
declare -A COURSE_WIDTHS=(
    [vica_cornercourse]="0.80 0.90 1.00 1.20 1.50"
    [vica_avoidcourse]="0.80 0.90 1.00 1.20 1.50"
    [vica_uturncourse]="1.00 1.20 1.50 1.80 2.00"
)
COURSES=${COURSES:-"vica_cornercourse vica_avoidcourse vica_uturncourse"}

run_cell() {   # $1 controller  $2 planner  $3 course  $4... widths
    local c="$1" p="$2" course="$3"; shift 3
    local widths=("$@")
    local dir="$ROOT/$course"
    echo
    echo "##############################################################"
    echo " $c + $p   $course   폭 ${widths[*]}"
    echo "##############################################################"
    COURSE="$course" RESULTS_DIR="$dir" PLANNER="$p" REPEATS="${REPEATS:-1}" \
        bash "$SWEEP" "$c" "${widths[@]}"
}

case "$MODE" in
screen)
    echo "1단계 선별: 12조합 x ${SCREEN_WIDTH} m x $(echo "$COURSES" | wc -w)코스"
    for course in $COURSES; do
        w="$SCREEN_WIDTH"
        # The U-turn course has no 0.80/0.90 cell, and the screening width has
        # to be one the course was actually built with or lane_spawn has
        # nothing to look up.
        case " ${COURSE_WIDTHS[$course]} " in
            *" $w "*) ;;
            *) w=$(echo "${COURSE_WIDTHS[$course]}" | tr ' ' '\n' | tail -1)
               echo "  ($course 에 ${SCREEN_WIDTH} 레인이 없어 $w 로 대체)" ;;
        esac
        for c in $CONTROLLERS; do
            for p in $PLANNERS; do
                run_cell "$c" "$p" "$course" "$w"
            done
        done
    done
    ;;
depth)
    [ "$#" -gt 0 ] || { echo "심화 단계는 셀을 지정해야 합니다: dwb:lattice ..."; exit 2; }
    for cell in "$@"; do
        c="${cell%%:*}"; p="${cell##*:}"
        for course in $COURSES; do
            read -r -a widths <<< "${WIDTHS:-${COURSE_WIDTHS[$course]}}"
            run_cell "$c" "$p" "$course" "${widths[@]}"
        done
    done
    ;;
*)
    echo "모드는 screen 또는 depth 입니다"; exit 2 ;;
esac

echo
echo "=============================================================="
echo " 끝. 결과 $ROOT"
echo " 요약:  python3 $PKG/src/vica_description/scripts/matrix_report.py $ROOT"
echo "=============================================================="
