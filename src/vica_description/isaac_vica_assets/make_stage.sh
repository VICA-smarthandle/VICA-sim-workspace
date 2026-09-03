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
ISAAC="${ISAAC_SIM:-$HOME/isaacsim}"
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

# Every use below is "$HERE_STAGE", which is right for a bare filename and
# nonsense for anything else: a path pastes onto $HERE and the run fails three
# steps later with "could not open" on a doubled path. Tab completion produces
# a path, so this is easy to hit and the error says nothing about why.
case "$STAGE" in
    */*) STAGE="$(cd "$(dirname "$STAGE")" && pwd)/$(basename "$STAGE")"
         HERE_STAGE="$STAGE" ;;
    *)   HERE_STAGE="${HERE}/${STAGE}" ;;
esac
if [ ! -f "$HERE_STAGE" ] && [ -n "${BUILDER}" ]; then
    : # the builder is about to create it
elif [ ! -f "$HERE_STAGE" ]; then
    echo "스테이지가 없습니다: $HERE_STAGE"
    exit 1
fi

cd "$ISAAC"
# shellcheck disable=SC1091
source ./setup_ros_env.sh >/dev/null 2>&1

if [ -n "$BUILDER" ]; then
echo "=============================================================="
echo " 1/3  build   $BUILDER"
echo "=============================================================="
# BUILDER_ARGS passes flags to the builder. Quoting them into $BUILDER makes
# the whole string the filename, which fails with "can't open file
# 'build_vica_testroom_stage.py --arm'" -- a message that reads like a missing
# file rather than a quoting mistake.
#
#     BUILDER_ARGS=--arm ./make_stage.sh build_vica_testroom_stage.py \
#         vica_testroom_arm.usd
# shellcheck disable=SC2086
if ! ./python.sh -u "$HERE/$BUILDER" ${BUILDER_ARGS} > /tmp/vica_build.log 2>&1; then
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
if ! ./python.sh -u "$HERE/prepare_stage.py" "$HERE_STAGE" > /tmp/vica_prepare.log 2>&1; then
    tail -20 /tmp/vica_prepare.log
    echo "prepare 실패"
    exit 1
fi
# The exit code is necessary and has not always been sufficient. Isaac's
# SimulationApp.close() ends the process with status 0 of its own accord, so a
# step that raised, printed its traceback and asked to exit 1 was read here as
# a success; verify then passed the stage a *previous* run had prepared and
# said 전부 통과. prepare_stage.py no longer routes failure through close(),
# and this reads the log as well, because one of the two guards being enough
# is the assumption that produced the wrong answer.
if grep -q "RAISED" /tmp/vica_prepare.log; then
    grep -B2 -A12 "RAISED" /tmp/vica_prepare.log
    echo "prepare 단계가 예외로 죽었습니다 (종료 코드는 0 이었습니다)"
    exit 1
fi
grep -E "^===|built |velocity drive|position drive|자세:|articulation solver|stamped|다음:" /tmp/vica_prepare.log || true

echo
echo "=============================================================="
echo " 3/3  verify"
echo "=============================================================="
set +e
./python.sh -u "$HERE/verify_stage.py" "$HERE_STAGE" 8 > /tmp/vica_verify.log 2>&1
rc=$?
set -e
grep -E "^---|^  (PASS|FAIL)|실패|전부 통과" /tmp/vica_verify.log || tail -20 /tmp/vica_verify.log
# The exit code is read AND the log is read, for the same reason the prepare
# step above does it. Isaac's SimulationApp.close() ends the process with
# status 0 of its own accord, and verify_stage.py calls close() before its
# sys.exit(1) -- so a stage that failed a check returned 0 here.
#
# That is not hypothetical. On 2026-09-02 the hospital stage failed "높이가
# 변하지 않음" with the robot 66.8 m below the floor, this line read rc=0, and
# stamp_verified.py below marked it verified. The trial harness accepts a
# stamped stage, so the next run would have measured a robot in freefall and
# produced a table of numbers about it.
if [ "$rc" -ne 0 ] || grep -qE "^  FAIL|^  실패 [0-9]+건" /tmp/vica_verify.log; then
    echo
    echo "검증 실패 -- 이 스테이지로 측정하지 마십시오."
    [ "$rc" -eq 0 ] && echo "(종료 코드는 0 이었습니다. 로그로 판정했습니다.)"
    exit 1
fi

# Stamped by a process that never played the stage. See stamp_verified.py.
python3 "$HERE/stamp_verified.py" "$HERE_STAGE"
