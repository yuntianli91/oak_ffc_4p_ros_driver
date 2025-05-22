#!/bin/bash
# 获取当前时间作为文件名
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
FILENAME="calib_cam_imu_$TIMESTAMP.bag"

rosbag record /mavros/imu/data_raw \
  /oak_ffc_4p/assemble_image/compressed \
  --output-name=FILENAME 