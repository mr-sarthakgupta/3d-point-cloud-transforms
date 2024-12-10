import numpy as np
import open3d as o3d
import os

def detect_and_reorient_floor(ply_file_path):
    pcd = o3d.io.read_point_cloud(ply_file_path)
    points = np.asarray(pcd.points)
    print(f'Debug - points shape: {points.shape}')

    target_normal = np.array([0, -1, 0])

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                           ransac_n=3,
                                           num_iterations=1000)

    floor_normal = plane_model[:3] / np.linalg.norm(np.expand_dims(plane_model[:3], axis=0), ord='fro')
    
    rotation_axis = np.cross(floor_normal, target_normal)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    
    angle = np.arccos(np.dot(floor_normal, target_normal))
    
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * angle)

    pcd.rotate(R)    
    o3d.io.write_point_cloud(f'ex_shoe_pc.ply', pcd)

    print('Debug - reoriented_pcd written successfully')


def compute_rotation_matrix(source_normal, target_normal):
    # Normalize vectors
    source_normal = source_normal / np.linalg.norm(source_normal)
    target_normal = target_normal / np.linalg.norm(target_normal)
    
    # Calculate rotation axis and angle
    rotation_axis = np.cross(source_normal, target_normal)
    if np.all(rotation_axis == 0):
        return np.eye(3)  # Return identity if vectors are parallel
    
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    cos_angle = np.dot(source_normal, target_normal)
    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
    
    # Convert axis-angle to rotation matrix using Rodrigues' formula
    K = np.array([
        [0, -rotation_axis[2], rotation_axis[1]],
        [rotation_axis[2], 0, -rotation_axis[0]],
        [-rotation_axis[1], rotation_axis[0], 0]
    ], dtype=np.float64)
    rotation_matrix = (np.eye(3, dtype=np.float64) + np.sin(angle) * K + 
                      (1 - cos_angle) * np.matmul(K, K))
    
    print('Debug - rotation_matrix:', rotation_matrix)

    rotation_matrix = rotation_matrix.astype(np.float64)

    return rotation_matrix


if __name__ == "__main__":
    # for filename in os.listdir('plyfiles')[:1]:
    detect_and_reorient_floor(f'plyfiles/shoe_pc.ply')