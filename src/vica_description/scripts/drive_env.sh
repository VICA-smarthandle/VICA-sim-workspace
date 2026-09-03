#!/usr/bin/env bash
# Drive a mapped environment from one point to another and record it.
#
#     drive_env.sh vica_hospital 0 0 10 0
#     GOALS="10 0 10 6 0 6" drive_env.sh vica_hospital 0 0
#
# For the environments that stand in for real buildings, where the courses'
# lane machinery does not apply: there are no lanes, no widths and no spec
# written by a builder, just a map and two points.
#
# Writes results/env/<stage>/<name>.csv and .json. The JSON is in the shape
# replay_render.py reads, so the run can be re-rendered with the actual robot
# in the actual stage afterwards.
#
# No set -u: ROS's setup.bash reads unset variables and aborts with no output.
set -o pipefail

PKG=${VICA_WS:-$HOME/VICA-smarthandle/vica_ws}
ISAAC=${ISAAC_SIM:-$HOME/isaacsim}
STAGE_NAME=${1:?usage: drive_env.sh <stage> <sx> <sy> <gx> <gy>}
SX=${2:?}; SY=${3:?}; GX=${4:?}; GY=${5:?}
# NAME is set in the environment on Ubuntu (from os-release), so the run name
# has its own variable. The first hospital run wrote Ubuntu.pose.log.
RUN_NAME=${RUN_NAME:-run}
CONTROLLER=${CONTROLLER:-dwb}
PLANNER=${PLANNER:-hybrid}
LIMIT=${LIMIT:-240}

cd "$PKG" || exit 1
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
python3 src/vica_description/scripts/select_controller.py "$CONTROLLER" \
    --planner "$PLANNER" \
    --config src/vica_description/config/vica_nav2_params.yaml || exit 1
colcon build --packages-select vica_description >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1

SHARE=$(ros2 pkg prefix vica_description)/share/vica_description
STAGE=$SHARE/isaac_vica_assets/$STAGE_NAME.usd
MAP=$SHARE/maps/$STAGE_NAME.yaml
for f in "$STAGE" "$MAP"; do
    [ -f "$f" ] || { echo "없습니다: $f"; exit 1; }
done

OUT=$PKG/results/env/$STAGE_NAME
mkdir -p "$OUT"
SPEC=$OUT/$RUN_NAME.spec.json
python3 - "$SPEC" "$STAGE_NAME" "$SX" "$SY" "$GX" "$GY" <<'PY'
import json, sys
path, stage, sx, sy, gx, gy = sys.argv[1:7]
json.dump({"stage": stage + ".usd",
           "start": [float(sx), float(sy)],
           "goal": [float(gx), float(gy)]}, open(path, "w"), indent=2)
PY

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
echo " $STAGE_NAME   $CONTROLLER + $PLANNER   ($SX,$SY) -> ($GX,$GY)"
echo " 결과 $OUT/$RUN_NAME"
echo "=============================================================="

stop_everything
LOG=/tmp/vica_env_sim.log
(cd "$ISAAC" && source ./setup_ros_env.sh >/dev/null 2>&1 && \
    VICA_POSE_LOG="$OUT/$RUN_NAME.pose.log" \
    nohup ./python.sh -u "$SHARE/isaac_vica_assets/play_stage.py" \
    "$STAGE" 600 "$SX" "$SY" 0 > "$LOG" 2>&1 &)
for _ in $(seq 1 40); do grep -q "=== playing" "$LOG" 2>/dev/null && break; sleep 5; done
sleep 14

nohup ros2 launch vica_description vica_nav2_bringup.launch.py \
    map:="$MAP" launch_rviz:=false > /tmp/vica_env_nav2.log 2>&1 &
sleep 35
ready=0
for _ in $(seq 1 12); do
    sleep 15
    st=$(timeout 8 ros2 lifecycle get /bt_navigator 2>&1 | head -1)
    [ "${st%% *}" = "active" ] && { ready=1; break; }
done
[ "$ready" = 1 ] || { echo "nav2 가 뜨지 않았습니다"; stop_everything; exit 1; }

timeout $((LIMIT + 60)) ros2 run vica_description obstacle_trial \
    --spec "$SPEC" --out "$OUT/$RUN_NAME.csv" --limit "$LIMIT"

# Into the shape replay_render reads: records[].track of [t, x, y, yaw].
python3 - "$OUT/$RUN_NAME.csv" "$OUT/$RUN_NAME.json" "$STAGE_NAME" "$GX" "$GY" <<'PY'
import csv, json, sys
src, dst, stage, gx, gy = sys.argv[1:6]
track = []
with open(src) as fh:
    for r in csv.DictReader(fh):
        track.append([float(r["t"]), float(r["x"]), float(r["y"]), float(r["yaw"])])
json.dump({"stage": stage + ".usd",
           "records": [{"label": stage, "result": "recorded",
                        "goal": [float(gx), float(gy)], "track": track}]},
          open(dst, "w"))
print(f"  {len(track)} 샘플 -> {dst}")
PY

stop_everything
echo
echo "=== 렌더:"
echo "    \$ISAAC_SIM/python.sh $SHARE/isaac_vica_assets/replay_render.py \\"
echo "        $OUT/$RUN_NAME.json --stage $STAGE --view follow"
