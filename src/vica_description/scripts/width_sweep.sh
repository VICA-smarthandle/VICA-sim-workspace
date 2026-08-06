#!/usr/bin/env bash
# Drive every lane of the width course, from a fresh spawn each time.
#
#     width_sweep.sh dwb            all six lanes, three passes each
#     width_sweep.sh mppi 1.40 1.20 just those widths
#
# The simulator is restarted for every single pass. That is slow -- about five
# minutes a pass, ninety for a controller -- and it is the only way the numbers
# mean what they say.
#
# The alternative was to drive up through a lane and back down, counting each
# traversal as a pass. It measured the wrong thing: the robot reached the exit
# corridor and then could not turn round to come back, so the second pass
# recorded a failure that was about the corridor rather than the lane. Below
# 1.60 m the robot cannot turn on the spot at all, so there is no arrangement
# where a return trip is comparable to the outbound one.
#
# No set -u: ROS's setup.bash reads unset variables and aborts with no output.
set -o pipefail

PKG=/home/sim/vica_ws
ISAAC=${ISAAC_SIM:-/home/sim/isaacsim}
ASSETS=$PKG/src/vica_description/isaac_vica_assets
STAGE=$ASSETS/vica_widthcourse.usd
PLAY=$ASSETS/play_stage.py
# Read from the course spec, not written here. The spawn moves whenever a lane
# is added -- adding three widths shifted it from -8.70 to -13.80 -- and a
# hardcoded copy would seed AMCL somewhere the robot is not, which is a whole
# evening of "Start occupied" waiting to happen.
SPEC=$ASSETS/vica_widthcourse.json
REPEATS=${REPEATS:-3}

CONTROLLER="${1:?usage: width_sweep.sh <dwb|mppi> [widths...]}"
shift
WIDTHS=("$@")
[ ${#WIDTHS[@]} -eq 0 ] && WIDTHS=(1.40 1.20 1.00 0.90 0.80 0.70)

RES=${RESULTS_DIR:-/tmp/vica_width_results}/$CONTROLLER
mkdir -p "$RES"

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
cd "$PKG" || exit 1
python3 src/vica_description/scripts/select_controller.py "$CONTROLLER" \
    --config src/vica_description/config/vica_nav2_params.yaml
colcon build --packages-select vica_description >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
MAP=$(ros2 pkg prefix vica_description)/share/vica_description/maps/vica_widthcourse.yaml

stop_everything() {
    for p in /proc/[0-9]*; do
        id=${p##*/}
        [ "$id" = "$$" ] && continue
        cmd=$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null) || continue
        if printf '%s' "$cmd" | grep -qE 'controller_server|planner_server|bt_navigator|velocity_smoother|behavior_server|collision_monitor|smoother_server|waypoint_follower|route_server|amcl|map_server|lifecycle_manager|nav2_startup_gate|opennav_docking|ros2 launch vica|kit/python'; then
            kill -9 "$id" 2>/dev/null
        fi
    done
    sleep 4
}

echo "=============================================================="
echo " $CONTROLLER   레인 ${WIDTHS[*]}   각 ${REPEATS}회   결과 $RES"
echo "=============================================================="

for W in "${WIDTHS[@]}"; do
    for R in $(seq 1 "$REPEATS"); do
        OUT="$RES/lane_${W}_r${R}.json"
        [ -f "$OUT" ] && { echo "  $W #$R  이미 있음, 건너뜀"; continue; }

        # Start the robot in front of the lane under test, not at one end
        # of the course. Otherwise the approach -- which differs by 27 m
        # between the nearest and furthest lane -- is what gets measured. A
        # 1.20 m lane went from 3/3 to 0/3 purely by being moved to the far
        # end when three widths were added.
        SPAWN=$(python3 "$ASSETS/lane_spawn.py" "$SPEC" "$W")
        SPAWN_X=${SPAWN%% *}
        SPAWN_Y=${SPAWN##* }
        if [ -z "$SPAWN_X" ] || [ -z "$SPAWN_Y" ]; then
            echo "  $W #$R  스폰 계산 실패 -- 중단"
            exit 1
        fi

        stop_everything
        LOG=/tmp/vica_sweep_sim.log
        (cd "$ISAAC" && source ./setup_ros_env.sh >/dev/null 2>&1 && \
            nohup ./python.sh -u "$PLAY" "$STAGE" 900 "$SPAWN_X" "$SPAWN_Y" > "$LOG" 2>&1 &)
        for _ in $(seq 1 40); do grep -q "=== playing" "$LOG" 2>/dev/null && break; sleep 5; done
        sleep 14

        nohup ros2 launch vica_description vica_nav2_bringup.launch.py \
            map:="$MAP" launch_rviz:=false > /tmp/vica_sweep_nav2.log 2>&1 &
        sleep 25
        timeout 20 ros2 topic pub --once /initialpose \
            geometry_msgs/msg/PoseWithCovarianceStamped \
            "{header: {frame_id: map}, pose: {pose: {position: {x: $SPAWN_X, y: $SPAWN_Y}, orientation: {w: 1.0}}, covariance: [0.05,0,0,0,0,0, 0,0.05,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.02]}}" \
            >/dev/null 2>&1

        ready=0
        for _ in $(seq 1 12); do
            sleep 15
            st=$(timeout 8 ros2 lifecycle get /bt_navigator 2>&1 | head -1)
            [ "${st%% *}" = "active" ] && { ready=1; break; }
        done
        if [ "$ready" -ne 1 ]; then
            echo "  $W #$R  nav2 가 뜨지 않음 -- 기록하고 넘어감"
            echo "{\"controller\":\"$CONTROLLER\",\"width\":$W,\"repeat\":$R,\"records\":[{\"result\":\"nav2-down\"}]}" > "$OUT"
            continue
        fi

        echo "  --- $W m  #$R"
        timeout 500 ros2 run vica_description width_trials \
            --phase lane --width "$W" --repeats 1 --spawn "$SPAWN_X,$SPAWN_Y" \
            --controller "$CONTROLLER" --out "$OUT" 2>&1 \
            | grep -E "통과|건너뜀|거부|출발 위치|검증"
    done
done

stop_everything
echo
echo "=== 완료. 결과 $RES"
