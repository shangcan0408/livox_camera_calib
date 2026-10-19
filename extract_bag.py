#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import rosbag
import cv2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

BAG_FILE = os.path.expanduser('~/calib_data.bag')  # 刚才录制的bag路径
IMAGE_TOPIC = '/camera/color/image_raw'
# 自动检测雷达话题（优先使用 PointCloud2）
LIDAR_TOPICS = ['/livox/points', '/livox/lidar']

DATA_DIR = os.path.expanduser('~/catkin_ws/src/livox_camera_calib/data')
os.makedirs(DATA_DIR, exist_ok=True)
IMG_OUT = os.path.join(DATA_DIR, '0.png')
PCD_OUT = os.path.join(DATA_DIR, '0.pcd')

print(f"[1/3] 读取 Bag: {BAG_FILE}")
bag = rosbag.Bag(BAG_FILE, 'r')
bridge = CvBridge()

# 1. 抽取中间最清晰的一张图片
saved_img = False
msg_count = bag.get_message_count(IMAGE_TOPIC)
target_idx = msg_count // 2
current_idx = 0

for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC]):
    if current_idx == target_idx:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(IMG_OUT, cv_img)
        print(f" -> 成功保存基准图像: {IMG_OUT}")
        saved_img = True
        break
    current_idx += 1

if not saved_img:
    print("❌ 未能在图像话题中提取到图像，请检查话题名称！")
    sys.exit(1)

# 2. 累积 15~20 秒的点云数据
print("[2/3] 正在累积稠密点云，请稍候...")
all_points = []
lidar_topic_found = None

for t_name in LIDAR_TOPICS:
    if bag.get_message_count(t_name) > 0:
        lidar_topic_found = t_name
        break

if not lidar_topic_found:
    print("❌ 未找到雷达话题，Bag 中的话题有:", bag.get_type_and_topic_info().topics.keys())
    sys.exit(1)

print(f" -> 找到雷达话题: {lidar_topic_found}")
frame_cnt = 0
for topic, msg, t in bag.read_messages(topics=[lidar_topic_found]):
    if msg._type == 'sensor_msgs/PointCloud2':
        pts = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        for p in pts:
            all_points.append(p)
    elif 'CustomMsg' in msg._type:
        for p in msg.points:
            all_points.append((p.x, p.y, p.z, float(p.reflectivity)))
    frame_cnt += 1

bag.close()
print(f" -> 共累积 {frame_cnt} 帧点云，总点数: {len(all_points)}")

# 3. 写入标准 PCD 格式 (PointXYZI)
print(f"[3/3] 正在写入高密度 PCD: {PCD_OUT}")
with open(PCD_OUT, 'w') as f:
    f.write("# .PCD v0.7 - Point Cloud Data file format\n")
    f.write("VERSION 0.7\n")
    f.write("FIELDS x y z intensity\n")
    f.write("SIZE 4 4 4 4\n")
    f.write("TYPE F F F F\n")
    f.write("COUNT 1 1 1 1\n")
    f.write(f"WIDTH {len(all_points)}\n")
    f.write("HEIGHT 1\n")
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write(f"POINTS {len(all_points)}\n")
    f.write("DATA ascii\n")
    for pt in all_points:
        f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f} {pt[3]:.1f}\n")

print("🎉 恭喜！图像与稠密点云已全部生成在 data/ 目录下！")
