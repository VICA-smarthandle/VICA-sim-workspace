#!/usr/bin/env bash
# Drive every lane of the width course, from a fresh spawn each time.
#
#     width_sweep.sh dwb                 all six lanes, three passes each
#     width_sweep.sh mppi 1.40 1.20      just those widths
#     INFLATION=0.66 width_sweep.sh mppi 1.20 1.15
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
# INFLATION sets inflation_radius on both costmaps for the whole sweep and
# writes the result under a directory that names it, so a 0.55 run and a 0.66
# run cannot land on top of each other. Leave it unset to use whatever the
# config already carries. The config's own value comes from the physical robot
# and this script does not put it back afterwards -- select_controller --show
# reports it, and the results directory records what was used.
#
# No set -u: ROS's setup.bash reads unset variables and aborts with no output.
set -o pipefail

PKG=/home/sim/vica_ws
ISAAC=${ISAAC_SIM:-/home/sim/isaacsim}
SRC_ASSETS=$PKG/src/vica_description/isaac_vica_assets
PLAY=$SRC_ASSETS/play_stage.py
REPEATS=${REPEATS:-3}
INFLATION=${INFLATION:-}
# Which course. The width course measures corridors; the avoid course measures
# the gap beside an obstacle, using the same lane schema so nothing else here
# changes. Both write "width" into the spec, and for the avoid course that is
# the free gap -- which is the point, because the two tables then line up
# column for column and the difference is what an obstacle costs.
COURSE=${COURSE:-vica_widthcourse}

CONTROLLER="${1:?usage: [INFLATION=x] width_sweep.sh <dwb|mppi> [widths...]}"
shift
WIDTHS=("$@")
[ ${#WIDTHS[@]} -eq 0 ] && WIDTHS=(1.40 1.20 1.00 0.90 0.80 0.70)

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
cd "$PKG" || exit 1

SEL=(python3 src/vica_description/scripts/select_controller.py "$CONTROLLER"
     --config src/vica_description/config/vica_nav2_params.yaml)
[ -n "$INFLATION" ] && SEL+=(--inflation "$INFLATION")
"${SEL[@]}" || exit 1

colcon build --packages-select vica_description >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1

SHARE=$(ros2 pkg prefix vica_description)/share/vica_description
MAP=$SHARE/maps/$COURSE.yaml
# Play the stage the harness gates on. width_trials looks for the verify stamp
# on the USD sitting next to the installed spec, so playing the source copy
# meant the gate and the simulator were reading two different files. They are
# the same bytes today; they stop being the same the first time the builder is
# run without a rebuild after it, which is exactly when the gate matters.
SPEC=$SHARE/isaac_vica_assets/$COURSE.json
STAGE=$SHARE/isaac_vica_assets/$COURSE.usd
for f in "$SPEC" "$STAGE" "$MAP"; do
    [ -f "$f" ] || { echo "설치본이 없습니다: $f"; exit 1; }
done

# The inflation actually in the built config, not the one asked for -- if the
# build silently used a stale copy this is where it shows.
INFL_USED=$(grep -m1 -oE "inflation_radius: [0-9.]+" \
    "$SHARE/config/vica_nav2_params.yaml" | grep -oE "[0-9.]+$")
RES=${RESULTS_DIR:-/tmp/vica_width_results}/${CONTROLLER}_infl${INFL_USED}
mkdir -p "$RES"

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
echo " $CONTROLLER   inflation ${INFL_USED}   레인 ${WIDTHS[*]}   각 ${REPEATS}회"
echo " 결과 $RES"
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
        SPAWN=$(python3 "$SRC_ASSETS/lane_spawn.py" "$SPEC" "$W")
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
            echo "{\"controller\":\"$CONTROLLER\",\"inflation_radius\":$INFL_USED,\"width\":$W,\"repeat\":$R,\"records\":[{\"result\":\"nav2-down\"}]}" > "$OUT"
            continue
        fi

        echo "  --- $W m  #$R"
        # Keep the exit status of width_trials, not grep's, and say so when a
        # pass writes nothing.
        #
        # The filter below used to swallow every refusal the harness prints on
        # stderr. Twelve passes of a twenty-seven pass sweep produced no result
        # file and no visible reason: the AMCL gate was rejecting them, exactly
        # as designed, into a log that showed only the lane headers. A sweep
        # that silently returns five widths out of nine reads like a sweep that
        # found five widths.
        set +e
        # --spawn=... rather than --spawn ... : the value starts with a minus
        # sign for every lane in the left half of the course, and argparse
        # reads a leading minus as an option name and dies with "expected one
        # argument". The four narrowest lanes are all at negative x, so the
        # sweep had never once measured them -- it ran them, failed to parse,
        # wrote nothing, and reported a table of the five widths that happened
        # to sit at x >= 0.
        timeout 500 ros2 run vica_description width_trials \
            --phase lane --width "$W" --repeats 1 --spawn="$SPAWN_X,$SPAWN_Y" \
            --spec "$SPEC" --controller "$CONTROLLER" --out "$OUT" 2>&1 \
            | grep -E "통과|건너뜀|거부|출발 위치|검증|AMCL|없습니다|않았습니다"
        rc=${PIPESTATUS[0]}
        # No `set -e` to match the `set +e` above: errexit is never enabled in
        # this script (line 27 turns on pipefail and nothing else), so turning
        # it on here would not restore a previous state, it would start one.
        # From the second pass onward every transient non-zero would abort the
        # sweep: `ros2 lifecycle get` while nav2 is still coming up, `topic pub`
        # hitting its timeout, lane_spawn failing. The retry loop and the
        # nav2-down record below exist to handle exactly those, and errexit
        # would kill the sweep before they ran.
        if [ ! -f "$OUT" ]; then
            echo "  $W #$R  결과 없음 (width_trials exit $rc) -- 기록하고 넘어감"
            echo "{\"controller\":\"$CONTROLLER\",\"inflation_radius\":$INFL_USED,\"width\":$W,\"repeat\":$R,\"records\":[{\"result\":\"refused\",\"exit\":$rc}]}" > "$OUT"
        fi
    done
done

stop_everything
echo
echo "=== 완료. $CONTROLLER inflation $INFL_USED -> $RES"
