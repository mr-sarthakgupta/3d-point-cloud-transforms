import os
import random
import numpy as np
import open3d as o3d
import pytest
from reorient_floor import align_point_cloud_to_floor
import tempfile

# np.random.seed(42)

def test_floor_alignment():
    filename = random.choice(os.listdir("plyfiles"))
    original_pcd = o3d.io.read_point_cloud(f"plyfiles/{filename}")
    
    # Apply random transformation
    random_angle = np.random.rand() * (np.pi * 0.33)  # Random rotation up to ~30 degrees
    random_axis = np.random.rand(3)
    random_axis = random_axis / np.linalg.norm(random_axis)
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(random_axis * random_angle)
    
    random_translation = np.random.rand(3) - 0.5
    
    transformed_pcd = o3d.geometry.PointCloud(original_pcd)
    transformed_pcd.rotate(R)
    transformed_pcd.translate(random_translation)
    
    # Save transformed point cloud to temporary file
    temp_dir = tempfile.mkdtemp()
    input_path = f"{temp_dir}/input.ply"
    output_path = f"{temp_dir}/output.ply"
    o3d.io.write_point_cloud(input_path, transformed_pcd)
    
    # Run alignment
    aligned_pcd, floor_plane, floor_inliers = align_point_cloud_to_floor(input_path, output_path, name="test.ply")
    
    assert len(floor_inliers) > 100, "Not enough floor points detected" 
    
    aligned_points = np.asarray(aligned_pcd.points)
    assert np.mean(np.abs(aligned_points[floor_inliers][:, 1])) < 0.05, "Floor not properly aligned to y=0"

if __name__ == '__main__':
    pytest.main([__file__])
    # test_floor_alignment()