#!/usr/bin/env bash
#
# Expand urdf/VICA.xacro into urdf/vica.urdf, the file the Isaac URDF importer
# actually reads.
#
# Isaac's importer takes .urdf, not .xacro, so the expanded copy is a real build
# artefact rather than a convenience -- and it is the one file in the chain that
# can silently revert a fix. urdf/vica.urdf sat in the tree carrying the old
# 0.293 m wheel separation long after VICA.xacro was corrected to 0.364 m;
# importing it would have put the wrong robot back into the stage with nothing
# to show that anything had happened.
#
# Run this first, every time, before re-importing:
#
#   1. export_isaac_urdf.sh        <- you are here
#   2. Isaac URDF Importer  -> urdf/vica.urdf
#   3. fixup_vica_usd_joints.py    (caster drives off, wheel drives set)
#   4. build_vica_ros_graphs.py    (every ROS 2 interface graph)
#
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg_dir="$(dirname "${script_dir}")"
xacro_in="${pkg_dir}/urdf/VICA.xacro"
urdf_out="${pkg_dir}/urdf/vica.urdf"

if ! command -v xacro >/dev/null 2>&1; then
    echo "xacro not on PATH -- source /opt/ros/jazzy/setup.bash first." >&2
    exit 1
fi

xacro "${xacro_in}" -o "${urdf_out}"
check_urdf "${urdf_out}" >/dev/null

echo "wrote ${urdf_out}"
echo
# The three numbers worth eyeballing: the drive wheels sit at +/-0.182 (0.364 m
# apart), and the laser is 0.192 above base_link for an absolute 0.382 m.
grep -A1 -E '"(left|right)_wheel_joint"|"laser_joint"' "${urdf_out}" \
    | grep -E 'joint name|origin' \
    | sed 's/^[[:space:]]*/  /'
