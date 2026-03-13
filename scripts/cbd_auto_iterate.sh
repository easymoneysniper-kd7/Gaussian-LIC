#!/bin/bash
set -euo pipefail

PROJECT="/home/hao7/project/Gaussian-LIC"
BASELINE_SSIM="0.867048"  # v16

echo "[auto] Starting CBD auto iterate at $(date)"

# Ensure no stale ROS processes are running from previous interrupted runs
docker exec gaussian_lic_dev bash -lc "pkill -9 -x gs_mapping 2>/dev/null || true; pkill -9 -x laserMapping 2>/dev/null || true; pkill -9 -x rosbag 2>/dev/null || true; pkill -9 -x roscore 2>/dev/null || true; pkill -9 -x rosmaster 2>/dev/null || true"

# Ensure CBD raw frontend launch exists inside container
if ! docker exec gaussian_lic_dev bash -lc "test -f /root/catkin_fastlivo2/src/FAST-LIVO2/launch/mapping_cbd_raw.launch"; then
  echo "[auto] mapping_cbd_raw.launch not found in container; creating it"
  docker exec gaussian_lic_dev bash -lc "cat > /root/catkin_fastlivo2/src/FAST-LIVO2/launch/mapping_cbd_raw.launch <<'EOF'
<launch>
  <arg name='rviz' default='false' />
  <rosparam command='load' file='\$(find fast_livo)/config/avia.yaml' />
  <node pkg='fast_livo' type='fastlivo_mapping' name='laserMapping' output='screen'>
    <rosparam file='\$(find fast_livo)/config/camera_pinhole.yaml' />
  </node>
</launch>
EOF"
fi

run_one() {
  local tag="$1"
  local pos_lr="$2"
  local subsample="$3"

  echo "[auto] ===== Run $tag ====="
  echo "[auto] Setting position_lr=$pos_lr, subsample_factor=$subsample"

  sed -i "s/^position_lr:.*/position_lr: ${pos_lr}/" "$PROJECT/config/fastlivo.yaml"
  sed -i "s#<param name=\"subsample_factor\" type=\"int\"    value=\"[0-9]\+\"     />#<param name=\"subsample_factor\" type=\"int\"    value=\"${subsample}\"     />#" "$PROJECT/launch/fastlivo.launch"

  echo "[auto] Launching e2e for $tag"
  ROSBAG_IMAGE_REMAP='/left_camera/image:=/left_camera/image' \
  bash "$PROJECT/scripts/run_fastlivo2_e2e.sh" \
    "CBD_Building_01_fastlivo2_${tag}" \
    1.0 \
    /root/catkin_gaussian/src/Gaussian-LIC/datasets/CBD_Building_01.bag \
    mapping_cbd_raw.launch \
    fastlivo.launch \
    > "/tmp/${tag}_run.log" 2>&1

  local result_dir
  result_dir=$(ls -d "$PROJECT/result/CBD_Building_01_fastlivo2_${tag}"* 2>/dev/null | sort | tail -1)
  if [[ -z "${result_dir}" || ! -f "${result_dir}/metrics.txt" ]]; then
    echo "[auto] ERROR: metrics not found for $tag"
    return 1
  fi

  local test_ssim test_lpips
  test_ssim=$(grep '^test_ssim=' "${result_dir}/metrics.txt" | cut -d= -f2)
  test_lpips=$(grep '^test_lpips=' "${result_dir}/metrics.txt" | cut -d= -f2)

  echo "[auto] Result $tag: test_ssim=${test_ssim} test_lpips=${test_lpips}"
  echo "[auto] metrics path: ${result_dir}/metrics.txt"

  printf "%s\n" "$test_ssim"
}

v18_ssim=$(run_one "v18_sub1" "0.00008" "1" | tail -1)

echo "[auto] Compare v18_ssim=${v18_ssim} vs baseline=${BASELINE_SSIM}"
if awk -v a="$v18_ssim" -v b="$BASELINE_SSIM" 'BEGIN{exit !(a < b)}'; then
  echo "[auto] v18 worse than baseline; running v19 with higher position_lr"
  run_one "v19_sub1_pos16" "0.00016" "1" >/tmp/v19_auto_result.txt
else
  echo "[auto] v18 >= baseline; skip v19"
fi

echo "[auto] CBD auto iterate done at $(date)"
