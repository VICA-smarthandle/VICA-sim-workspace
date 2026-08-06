#!/usr/bin/env bash
# Build a course, prepare it, and verify it -- in that order, stopping on the
# first failure.
#
#     ./make_stage.sh build_vica_widthcourse_stage.py vica_widthcourse.usd
#
# The three steps have to run together and in order. Running the builder alone
# leaves a stage with no sensors, no joint drives and no ROS graphs, and nothing
# about the file says so: it opens, it renders, and the robot falls through the
# floor when you try to use it. Doing them by hand is how a width course was
# rebuilt twice and measured once in between.
# No set -u: ROS's setup_ros_env.sh reads unset variables and the script would
# abort with no output at all. That has happened here before, to a benchmark
# script, and cost an afternoon before anyone looked at the flags.
#
# No pipefail either, because every step below is piped into grep, and grep
# returns 1 when it matches nothing -- which would abort a step that succeeded.
# Each step's own exit code is checked explicitly instead.
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
ISAAC="${ISAAC_SIM:-/home/sim/isaacsim}"
# --prepare-only re-prepares and verifies a stage that already exists, for the
# stages built before this pipeline did. Rebuilding those would mean fetching
# the environment asset again, which is not in the repository.
if [ "${1:-}" = "--prepare-only" ]; then
    BUILDER=""
    STAGE="${2:?usage: make_stage.sh --prepare-only <stage.usd>}"
else
    BUILDER="${1:?usage: make_stage.sh <builder.py> <stage.usd>}"
    STAGE="${2:?usage: make_stage.sh <builder.py> <stage.usd>}"
fi

cd "$ISAAC"
# shellcheck disable=SC1091
source ./setup_ros_env.sh >/dev/null 2>&1

if [ -n "$BUILDER" ]; then
echo "=============================================================="
echo " 1/3  build   $BUILDER"
echo "=============================================================="
if ! ./python.sh -u "$HERE/$BUILDER" > /tmp/vica_build.log 2>&1; then
    tail -20 /tmp/vica_build.log
    echo "build 실패"
    exit 1
fi
grep -E "wrote|targets|robot starts|actual gap|MISMATCH" /tmp/vica_build.log || true
else
echo "=============================================================="
echo " 1/3  build   건너뜀 (--prepare-only)"
echo "=============================================================="
fi

echo
echo "=============================================================="
echo " 2/3  prepare  sensors, joints, ROS graphs"
echo "=============================================================="
if ! ./python.sh -u "$HERE/prepare_stage.py" "$HERE/$STAGE" > /tmp/vica_prepare.log 2>&1; then
    tail -20 /tmp/vica_prepare.log
    echo "prepare 실패"
    exit 1
fi
grep -E "^===|built |velocity drive|articulation solver|stamped|다음:" /tmp/vica_prepare.log || true

echo
echo "=============================================================="
echo " 3/3  verify"
echo "=============================================================="
set +e
./python.sh -u "$HERE/verify_stage.py" "$HERE/$STAGE" 8 > /tmp/vica_verify.log 2>&1
rc=$?
set -e
grep -E "^---|^  (PASS|FAIL)|실패|전부 통과" /tmp/vica_verify.log || tail -20 /tmp/vica_verify.log
if [ "$rc" -ne 0 ]; then
    echo
    echo "검증 실패 -- 이 스테이지로 측정하지 마십시오."
    exit "$rc"
fi

# Stamped by a process that never played the stage. See stamp_verified.py.
python3 "$HERE/stamp_verified.py" "$HERE/$STAGE"
