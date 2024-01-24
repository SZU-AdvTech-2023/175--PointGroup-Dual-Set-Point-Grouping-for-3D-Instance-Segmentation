import os
import random
import numpy as np
import open3d as o3d

scannet_root = '/data/scannet' # Scannet数据集的根目录

scene_list = os.listdir(scannet_root) # 获取Scannet数据集中的场景列表

scene1 = random.choice(scene_list)
scene2 = random.choice(scene_list)  # 选择两个随机的场景进行混合
print(f"mix scene：{scene1} and {scene2}")

# 加载场景1的点云数据

scene1_path = os.path.join(scannet_root, scene1)
scene1_ply = os.path.join(scene1_path, f"{scene1}_vh_clean_2.ply")
pcd1 = o3d.io.read_point_cloud(scene1_ply)

# 加载场景2的点云数据

scene2_path = os.path.join(scannet_root, scene2)
scene2_ply = os.path.join(scene2_path, f"{scene2}_vh_clean_2.ply")
pcd2 = o3d.io.read_point_cloud(scene2_ply)

# 随机选择一些点进行混合

num_points = min(len(pcd1.points), len(pcd2.points))
selected_indices = random.sample(range(num_points), int(num_points * 0.5))

# 提取选择的点

points1 = np.asarray(pcd1.points)[selected_indices, :]
colors1 = np.asarray(pcd1.colors)[selected_indices, :]
points2 = np.asarray(pcd2.points)[selected_indices, :]
colors2 = np.asarray(pcd2.colors)[selected_indices, :]

# 创建新的点云对象并将选择的点进行混合

mixed_pcd = o3d.geometry.PointCloud()
mixed_pcd.points = o3d.utility.Vector3dVector(np.vstack((points1, points2)))
mixed_pcd.colors = o3d.utility.Vector3dVector(np.vstack((colors1, colors2)))

# 保存混合后的点云数据

output_ply = '/data/mixed/output1.ply'
o3d.io.write_point_cloud(output_ply, mixed_pcd)