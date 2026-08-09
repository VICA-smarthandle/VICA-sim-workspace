#!/usr/bin/env bash
# The inflation A/B: does raising inflation_radius past the circumscribed radius
# change how narrow a lane the robot can drive?
#
#     inflation_ab.sh                 the four runs below, in order
#
# Four runs, two variables, so each can be read against the other three:
#
#                  inflation 0.55        inflation 0.66
#     DWB          run 1                 run 3
#     MPPI         run 2                 run 4
#
# All four are measured here rather than reusing earlier baselines. The
# earlier DWB sweep ran before per-lane spawning existed and before three
# widths were added to the course, so its "1.20 m" was a different lane at a
# different goal from the MPPI sweep's. Both tables were complete and neither
# recorded which course it drove. Reusing either would put that back.
#
# 0.66 is the first value above the padded circumscribed radius 0.650577. Below
# it nav2 logs an error every planner cycle and turns off the potential-field
# fast path, so every scored pose takes the full polygon check. Above it the
# fast path returns.
#
# Widths 1.40 / 1.20 / 1.15 -- 1.40 is the control. At inflation 0.66 a 1.40 m
# lane still has zero cost down its centreline (0.70 m from each wall) while a
# 1.20 m lane does not, so 1.40 isolates "did raising inflation break something
# unrelated" from "did the cost floor in the lane change the driving".
#
# Both controllers, because inflation and controller cannot both move at once
# and still leave a readable result. The robot's config carries a written
# precondition against switching to MPPI before this value is raised; that
# precondition is about the robot, and this measures what raising it does.
set -o pipefail

PKG=/home/sim/vica_ws
SWEEP=$PKG/src/vica_description/scripts/width_sweep.sh
WIDTHS=(1.40 1.20 1.15)
export RESULTS_DIR=${RESULTS_DIR:-/tmp/vica_width_ab}

# Never start while another sweep is running. Both edit the same config file
# and run colcon build; overlapping them produces two runs that each think they
# know what they measured.
waited=0
while pgrep -f "width_sweep.sh" | grep -qv "^$$\$"; do
    if [ "$waited" -eq 0 ]; then
        echo "기존 스윕이 끝나기를 기다립니다..."
        waited=1
    fi
    sleep 60
done
[ "$waited" -eq 1 ] && echo "기존 스윕 종료. 시작합니다."

for cell in "dwb 0.55" "mppi 0.55" "dwb 0.66" "mppi 0.66"; do
    set -- $cell
    echo
    echo "##############################################################"
    echo "#  $1  @ inflation $2"
    echo "##############################################################"
    INFLATION=$2 bash "$SWEEP" "$1" "${WIDTHS[@]}"
done

# Put the config back where the robot has it. The 0.55 in that file is the
# physical robot's value and the reason recorded next to it is that a person is
# holding the handle -- leaving 0.66 behind would silently desync the two.
python3 "$PKG/src/vica_description/scripts/select_controller.py" \
    --inflation 0.55 --config "$PKG/src/vica_description/config/vica_nav2_params.yaml"

echo
echo "=== A/B 완료. 결과 $RESULTS_DIR"
ls -d "$RESULTS_DIR"/*/ 2>/dev/null
