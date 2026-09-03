#!/usr/bin/env bash
#
# Expand urdf/VICA.xacro into the .urdf files the Isaac URDF importer reads.
#
#   export_isaac_urdf.sh              -> urdf/vica.urdf        (driving robot)
#   export_isaac_urdf.sh --arm        -> urdf/vica_arm.urdf    (use_arm:=true)
#
# VICA_ARM_ARGS passes extra xacro arguments into the arm variant only. The
# arm defaults to an OpenMANIPULATOR-X at the lengths ROBOTIS sells, which
# cannot reach a lift button from this deck -- 0.915 m against 1.10. The
# lengths we intend to build are a flag rather than a new default, because the
# file should keep describing the real product:
#
#   VICA_ARM_ARGS="arm_link3_len:=0.30 arm_link4_len:=0.28" \
#       export_isaac_urdf.sh --arm
#
# VICA.xacro carries the sampled reach table that those two numbers come from.
#   export_isaac_urdf.sh --all        -> both
#   export_isaac_urdf.sh --check      -> verify both are current, write nothing
#
# Isaac's importer takes .urdf, not .xacro, so the expanded copy is a real build
# artefact rather than a convenience -- and it is the one file in the chain that
# can silently revert a fix. urdf/vica.urdf sat in the tree carrying the old
# 0.293 m wheel separation long after VICA.xacro was corrected to 0.364 m;
# importing it would have put the wrong robot back into the stage with nothing
# to show that anything had happened.
#
# That was one artefact drifting from one source. There are now two, because
# the arm variant is built from the same xacro, and the importer step between
# them is a human clicking through a GUI. So each file carries a stamp naming
# the variant and the hash of the xacro it came from, and --check compares
# those against the xacro on disk. The stamp is a comment, which means it
# survives into nothing downstream -- it is there for this script and for
# whoever is about to spend an afternoon wondering which robot they imported.
#
# Run this first, every time, before re-importing:
#
#   1. export_isaac_urdf.sh        <- you are here
#   2. Isaac URDF Importer  -> urdf/vica.urdf  or  urdf/vica_arm.urdf
#   3. fixup_vica_usd_joints.py    (caster drives off, wheel drives set)
#   4. build_vica_ros_graphs.py    (every ROS 2 interface graph)
#
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg_dir="$(dirname "${script_dir}")"
xacro_in="${pkg_dir}/urdf/VICA.xacro"
materials_in="${pkg_dir}/urdf/materials.xacro"

mode="base"
case "${1:-}" in
    --arm)   mode="arm" ;;
    --all)   mode="all" ;;
    --check) mode="check" ;;
    "")      mode="base" ;;
    *) echo "usage: export_isaac_urdf.sh [--arm|--all|--check]" >&2; exit 2 ;;
esac

if ! command -v xacro >/dev/null 2>&1; then
    echo "xacro not on PATH -- source /opt/ros/jazzy/setup.bash first." >&2
    exit 1
fi

# The source hash covers materials.xacro too: it is included, so a change there
# changes the robot while leaving VICA.xacro's own hash untouched.
source_hash="$(cat "${xacro_in}" "${materials_in}" | sha256sum | cut -c1-16)"

stamp_of() {   # $1 = urdf path
    grep -m1 -oE 'vica-source-hash [0-9a-f]+' "$1" 2>/dev/null | awk '{print $2}'
}

variant_of() {
    grep -m1 -oE 'vica-variant [a-z]+' "$1" 2>/dev/null | awk '{print $2}'
}

write_variant() {   # $1 = variant name, $2 = xacro args...
    local variant="$1"; shift
    local out="${pkg_dir}/urdf/vica.urdf"
    [ "${variant}" = "arm" ] && out="${pkg_dir}/urdf/vica_arm.urdf"

    # Relative, so the header comment xacro writes does not record whoever
    # happened to build it. The absolute form put /home/<user>/... into a file
    # that is committed to a public repository.
    ( cd "${pkg_dir}" && xacro "urdf/$(basename "${xacro_in}")" "$@" \
        -o "${out}" )
    check_urdf "${out}" >/dev/null

    # Stamp goes in after check_urdf, so a malformed expansion never gets one.
    # Inserted on line 2, before <robot>, where it cannot land inside another
    # comment and break the XML.
    sed -i "2i <!-- vica-variant ${variant}  vica-source-hash ${source_hash} -->" "${out}"
    check_urdf "${out}" >/dev/null

    echo "wrote ${out}   [${variant}, ${source_hash}]"
    # The three numbers worth eyeballing: the drive wheels sit at +/-0.182
    # (0.364 m apart), and the laser is 0.192 above base_link, absolute 0.382.
    grep -A1 -E '"(left|right)_wheel_joint"|"laser_joint"' "${out}" \
        | grep -E 'joint name|origin' \
        | sed 's/^[[:space:]]*/    /'
    echo
}

check_variant() {   # $1 = variant, returns 1 if stale
    local variant="$1"
    local out="${pkg_dir}/urdf/vica.urdf"
    [ "${variant}" = "arm" ] && out="${pkg_dir}/urdf/vica_arm.urdf"

    if [ ! -f "${out}" ]; then
        echo "  ${variant}: ${out##*/} 없음"
        return 1
    fi
    local have; have="$(stamp_of "${out}")"
    local hv;   hv="$(variant_of "${out}")"
    if [ -z "${have}" ]; then
        echo "  ${variant}: ${out##*/} 에 스탬프 없음 -- 이 스크립트 이전에 만들어진 파일입니다"
        return 1
    fi
    if [ "${hv}" != "${variant}" ]; then
        echo "  ${variant}: ${out##*/} 는 '${hv}' 변형입니다 -- 파일명과 내용이 다릅니다"
        return 1
    fi
    if [ "${have}" != "${source_hash}" ]; then
        echo "  ${variant}: ${out##*/} 가 낡았습니다 (${have} != ${source_hash})"
        echo "            VICA.xacro 가 바뀐 뒤 다시 내보내지 않았습니다."
        return 1
    fi
    echo "  ${variant}: ${out##*/} 최신 (${have})"
    return 0
}

if [ "${mode}" = "check" ]; then
    echo "VICA.xacro + materials.xacro 해시: ${source_hash}"
    rc=0
    check_variant base || rc=1
    check_variant arm  || rc=1
    if [ "${rc}" -ne 0 ]; then
        echo
        echo "낡았거나 없는 파일이 있습니다. 그대로 임포트하면 스테이지에 다른 로봇이 들어갑니다."
    fi
    exit "${rc}"
fi

case "${mode}" in
    base) write_variant base ;;
    arm)  write_variant arm use_arm:=true ${VICA_ARM_ARGS:-} ;;
    all)  write_variant base
          write_variant arm use_arm:=true ${VICA_ARM_ARGS:-} ;;
esac
