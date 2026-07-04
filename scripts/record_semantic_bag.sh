#!/usr/bin/env bash
# Record the diffbot semantic-export bundle for semantic-backend bring-up /
# evaluation (DualMap, OneMap).
#
# Bring up the robot with the export enabled:
#   ros2 launch diffbot diffbot.launch.py enable_semantic_export:=true
# then run this on the PC (must see the robot's /semantic/* topics on the ROS 2
# graph, e.g. same ROS_DOMAIN_ID over the LAN). The bundle is throttled +
# compressed and carries map-frame camera Odometry, so it replays directly into
# the semantic backend. /tf{,_static} are included for debugging / offline
# re-derivation.
#
# Usage: record_semantic_bag.sh [output_dir]
set -euo pipefail

OUT="${1:-semantic_eval_$(date +%Y%m%d_%H%M%S)}"

exec ros2 bag record -o "${OUT}" \
  /semantic/color/image_raw/compressed \
  /semantic/aligned_depth/image_raw/compressedDepth \
  /semantic/color/camera_info \
  /semantic/odom \
  /tf \
  /tf_static
