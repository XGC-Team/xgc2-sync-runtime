#!/usr/bin/env bash
# Build the ros_io plugin against a ROS Noetic install.
#
# Usage: scripts/build-ros-io.sh OUT.so
#
#   ROS_PREFIX  the Noetic prefix (default /opt/ros/noetic). A RoboStack
#               conda environment works too; its own compiler is then used so
#               boost and libstdc++ match roscpp.
#   CXX         overrides the compiler.
#
# formation_generator/{AssumedTrajectory,FormationTick}, periodic_sync/SyncTrigger,
# rigid_state_estimator_msgs/RigidStateEstimate, hover_thrust_estimator_msgs/
# HoverThrustEstimate and the multirotor_reference_trajectory_msgs headers
# are generated with
# gencpp from the verbatim .msg copies in plugins/ros-io/msg, so the type names
# and md5 sums equal the original packages'.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
prefix="${ROS_PREFIX:-/opt/ros/noetic}"
[[ -f "$prefix/include/ros/ros.h" ]] || { echo "build-ros-io: no ROS Noetic at $prefix (set ROS_PREFIX)" >&2; exit 2; }
python="$prefix/bin/python3"; [[ -x "$python" ]] || python=python3
cxx="${CXX:-}"
if [[ -z "$cxx" ]]; then
  if [[ -x "$prefix/bin/x86_64-conda-linux-gnu-c++" ]]; then cxx="$prefix/bin/x86_64-conda-linux-gnu-c++"; else cxx=c++; fi
fi
gen="$(dirname "$1")/ros-io-gen"
rm -rf "$gen"
for msg in formation_generator/AssumedTrajectory formation_generator/FormationTick periodic_sync/SyncTrigger rigid_state_estimator_msgs/RigidStateEstimate \
  multirotor_reference_trajectory_msgs/{AnalyticReference,SampledReference,FlatReferencePoint,WaypointReferenceRequest,ActivePolynomialReference,ReferenceStatus} \
  hover_thrust_estimator_msgs/HoverThrustEstimate; do
  pkg="${msg%/*}"
  "$python" "$prefix/lib/gencpp/gen_cpp.py" "$root/plugins/ros-io/msg/$msg.msg" -p "$pkg" \
    -Istd_msgs:"$prefix/share/std_msgs/msg" -Igeometry_msgs:"$prefix/share/geometry_msgs/msg" -Iperiodic_sync:"$root/plugins/ros-io/msg/periodic_sync" -I"$pkg:$root/plugins/ros-io/msg/$pkg" \
    -o "$gen/$pkg" -e "$prefix/share/gencpp" >/dev/null
done
"$cxx" -std=c++17 -O2 -fPIC -Wall -Wextra -shared -fvisibility=hidden \
  -I "$root/abi/include" -I "$gen" -isystem "$prefix/include" \
  -o "$1" "$root/plugins/ros-io/ros_io.cpp" \
  -L "$prefix/lib" -Wl,-rpath,"$prefix/lib" -lroscpp -lroscpp_serialization -lrosconsole -lrostime -lcpp_common
