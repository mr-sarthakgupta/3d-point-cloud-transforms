import os
import numpy as np
import open3d as o3d
from reorient_floor import align_point_cloud_to_floor

# Load and process a PLY file
input_file = "reoriented_plyfiles/chair_pc.ply"
pcd, floor_plane, floor_inliers = align_point_cloud_to_floor(input_file)

# Get all points as numpy array
points = np.asarray(pcd.points)

# Create boolean mask for non-floor points 
non_floor_mask = np.zeros(len(points), dtype=bool)
non_floor_mask[floor_inliers] = True

# Get non-floor points
non_floor_points = points[non_floor_mask]

# Calculate mean absolute height of non-floor points
mean_abs_height = np.mean(np.abs(non_floor_points[:, 2]))
print(f"Mean absolute height of non-floor points: {mean_abs_height}")
