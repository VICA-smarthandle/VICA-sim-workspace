#!/usr/bin/env bash
# How late can someone step out and still be dealt with?
#
#     obstacle_sweep.sh                       every trigger distance
#     obstacle_sweep.sh 3.0 2.0 1.5           just these
#     REPEATS=3 obstacle_sweep.sh 2.0
#
# Drives the dynamic course once per trigger distance. The walker starts
# crossing when the robot is that far short of the crossing point, so a smaller
# number is a later, harder appearance. Everything else -- speed, geometry,
# where the goal is -- is identical between runs, which is the whole reason the
# trigger is a distance and not a time.
#
# The simulator is restarted for every run. Slow, and the only way the numbers
# mean what they say: a costmap that already has the walker marked from the
# previous pass is not a robot meeting someone for the first time.
#
# No set -u: ROS's setup.bash reads unset variables and aborts with no output.
set -o pipefail

PKG=${VICA_WS:-$HOME/VICA-smarthandle/vica_ws}
ISAAC=${ISAAC_SIM:-$HOME/isaacsim}
SRC_ASSETS=$PKG/src/vica_description/isaac_vica_assets
PLAY=$SRC_ASSETS/play_stage.py
COURSE=${COURSE:-vica_dynamiccourse}
REPEATS=${REPEATS:-1}
WALK_SPEED=${WALK_SPEED:-1.2}
LIMIT=${LIMIT:-180}

if [ $# -gt 0 ]; then
    TRIGGERS=("$@")
else
    # 6.0 m at 0.5 m/s is twelve seconds of warning, which nothing should fail.
    # 1.0 m is a metre and a half of stopping distance short of the walker and
    # nothing should pass. The interesting answer is between them, and the step
    # is 0.5 m because that is about one stopping distance.
    TRIGGERS=(6.0 5.0 4.0 3.0 2.5 2.0 1.5 1.0)
fi

cd "$PKG" || exit 1
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
colcon build --packages-select vica_description >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1

SHARE=$(ros2 pkg prefix vica_description)/share/vica_description
SPEC=$SHARE/isaac_vica_assets/$COURSE.json
STAGE=$SHARE/isaac_vica_assets/$COURSE.usd
MAP=$SHARE/maps/$COURSE.yaml
for f in "$SPEC" "$STAGE" "$MAP"; do
    [ -f "$f" ] || { echo "설치본이 없습니다: $f"; exit 1; }
done

RES=${RESULTS_DIR:-$PKG/results/obstacle}
mkdir -p "$RES"

SPAWN_X=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['start'][0])" "$SPEC")
SPAWN_Y=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['start'][1])" "$SPEC")

stop_everything() {
    for p in /proc/[0-9]*; do
        id=${p##*/}
        [ "$id" = "$$" ] && continue
        cmd=$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null) || continue
        if printf '%s' "$cmd" | grep -qE 'controller_server|planner_server|bt_navigator|velocity_smoother|behavior_server|collision_monitor|smoother_server|waypoint_follower|route_server|amcl|map_server|lifecycle_manager|nav2_startup_gate|opennav_docking|ros2 launch vica|kit/python|obstacle_trial'; then
            kill -9 "$id" 2>/dev/null
        fi
    done
    sleep 4
}

echo "=============================================================="
echo " 동적 장애물  코스 $COURSE  보행자 ${WALK_SPEED} m/s"
echo " 등장거리 ${TRIGGERS[*]}   각 ${REPEATS}회"
echo " 결과 $RES"
echo "=============================================================="

for T in "${TRIGGERS[@]}"; do
    for R in $(seq 1 "$REPEATS"); do
        BASE="$RES/trigger_${T}_r${R}"
        [ -f "${BASE}.json" ] && { echo "  $T m #$R  이미 있음, 건너뜀"; continue; }
        echo
        echo "  --- 등장거리 ${T} m  #$R"

        stop_everything
        LOG=/tmp/vica_obstacle_sim.log
        (cd "$ISAAC" && source ./setup_ros_env.sh >/dev/null 2>&1 && \
            VICA_WALK_TRIGGER_M="$T" VICA_WALK_SPEED="$WALK_SPEED" \
            VICA_WALK_LOG="${BASE}_walk.csv" \
            nohup ./python.sh -u "$PLAY" "$STAGE" 600 "$SPAWN_X" "$SPAWN_Y" 0 \
            > "$LOG" 2>&1 &)
        for _ in $(seq 1 40); do grep -q "=== playing" "$LOG" 2>/dev/null && break; sleep 5; done
        sleep 14

        nohup ros2 launch vica_description vica_nav2_bringup.launch.py \
            map:="$MAP" launch_rviz:=false > /tmp/vica_obstacle_nav2.log 2>&1 &
        sleep 25

        ready=0
        for _ in $(seq 1 12); do
            sleep 15
            st=$(timeout 8 ros2 lifecycle get /bt_navigator 2>&1 | head -1)
            [ "${st%% *}" = "active" ] && { ready=1; break; }
        done
        if [ "$ready" != 1 ]; then
            echo "    nav2 가 뜨지 않았습니다 -- 건너뜁니다"
            continue
        fi

        # The trigger has to reach the recorder too, not only the simulator.
        # It was set on the play_stage line alone, so obstacle_trial read its
        # own environment, found nothing, and wrote the default into every
        # trial's metadata. obstacle_report now prefers the filename, which is
        # this loop's own record, but the metadata should be right as well.
        VICA_WALK_TRIGGER_M="$T" VICA_WALK_SPEED="$WALK_SPEED" \
            timeout $((LIMIT + 60)) ros2 run vica_description obstacle_trial \
            --spec "$SPEC" --out "${BASE}.csv" --limit "$LIMIT"
    done
done

stop_everything
echo
echo "=== 완료. 표를 보려면:"
echo "    ros2 run vica_description obstacle_report $RES"
