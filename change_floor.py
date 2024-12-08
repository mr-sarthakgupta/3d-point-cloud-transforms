import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA

def detect_and_reorient_floor(ply_path):
    """
    Detect the floor plane in a point cloud and reorient it to the YZ plane centered at origin.
    
    Args:
        ply_path (str): Path to the input .ply file
    
    Returns:
        o3d.geometry.PointCloud: Reoriented point cloud
    """
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(ply_path)
    points = np.asarray(pcd.points)
    print('Debug - points shape:', points.shape)
    # Use PCA to detect the floor plane
    pca = PCA(n_components=3)
    pca.fit(points)
    
    # The plane with the smallest variance is likely the floor
    floor_normal = pca.components_[np.argmin(pca.explained_variance_)]

    # Ensure the floor normal points upwards (positive y direction)
    if floor_normal[1] < 0:
        floor_normal = -floor_normal
    
    # Create rotation matrix to align floor with YZ plane
    # We want to rotate so that floor_normal aligns with [0, 1, 0]
    current_up = floor_normal
    target_up = np.array([0, 1, 0])
    
    # Compute rotation using cross product and angle
    rotation_axis = np.cross(current_up, target_up)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    
    angle = np.arccos(np.dot(current_up, target_up))
    
    print('Debug - rotation axis:', rotation_axis)
    print('Debug - angle:', angle)

    # Create rotation matrix
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * angle)
    print('Debug - rotation matrix:', R)    
    # Apply rotation
    pcd.rotate(R)
    
    # Center the point cloud by subtracting the mean
    points_rotated = np.asarray(pcd.points)
    centroid = np.mean(points_rotated, axis=0)
    pcd.translate(-centroid)
    
    return pcd

def main():
    # Example usage
    ply_file_path = 'plyfiles/pillow_pc.ply'  # Replace with your .ply file path
    reoriented_pcd = detect_and_reorient_floor(ply_file_path)
    
    # Optional: Visualize the result
    o3d.visualization.draw_geometries([reoriented_pcd])
    
    # Optional: Save the reoriented point cloud
    o3d.io.write_point_cloud('reoriented_shoe_0.ply', reoriented_pcd)

if __name__ == '__main__':
    main()