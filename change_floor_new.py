import os
import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA

def detect_and_align_floor(input_ply_path, visualization=False):
    """
    Detect the floor plane in a point cloud and re-orient it to the YZ plane.
    
    Parameters:
    -----------
    input_ply_path : str
        Path to the input .ply file
    visualization : bool, optional
        Whether to visualize the original and transformed point clouds
    
    Returns:
    --------
    transformed_pcd : open3d.geometry.PointCloud
        Point cloud with floor aligned to YZ plane
    floor_plane : np.ndarray
        Detected floor plane coefficients [a, b, c, d]
    """
    # Read the point cloud
    pcd = o3d.io.read_point_cloud(input_ply_path)
    points = np.asarray(pcd.points)
    
    # Robust floor detection using RANSAC plane segmentation
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, 
                                             ransac_n=3, 
                                             num_iterations=1000)
    
    # Extract floor points
    floor_points = points[inliers]
    
    # Perform PCA to get principal axes
    pca = PCA(n_components=3)
    pca.fit(floor_points)
    
    # The normal vector of the plane
    normal = pca.components_[2]
    
    # Ensure normal points upward (positive y direction)
    if normal[1] < 0:
        normal = -normal
    
    # Create rotation matrix to align floor to YZ plane
    z_axis = np.array([0, 0, 1])
    rotation_axis = np.cross(normal, z_axis)
    rotation_angle = np.arccos(np.dot(normal, z_axis))
    
    # Rodrigues' rotation formula
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(
        rotation_axis * rotation_angle
    )
    
    # Transform point cloud
    transformed_pcd = pcd.rotate(rotation_matrix)
    
    # Center the point cloud by subtracting mean
    points_transformed = np.asarray(transformed_pcd.points)
    mean_point = np.mean(points_transformed, axis=0)
    transformed_pcd.translate(-mean_point)
    
    # Visualization (optional)
    if visualization:
        # Original point cloud
        orig_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
        
        # Transformed point cloud
        transformed_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
        transformed_coord_frame.rotate(rotation_matrix)
        
        o3d.visualization.draw_geometries([pcd, orig_coord_frame, 
                                           transformed_pcd, transformed_coord_frame])
    
    return transformed_pcd, plane_model

# Example usage
def main(filename):
    input_path = f'plyfiles/{filename}'
    transformed_pcd, floor_plane = detect_and_align_floor(input_path, visualization=True)
    
    # Save transformed point cloud
    o3d.io.write_point_cloud(f'experiment_floor_2/{filename}', transformed_pcd)
    
    # Print floor plane equation
    print("Floor Plane Equation (ax + by + cz + d = 0):")
    print(f"{floor_plane[0]:.4f}x + {floor_plane[1]:.4f}y + {floor_plane[2]:.4f}z + {floor_plane[3]:.4f} = 0")

if __name__ == "__main__":
    for filename in os.listdir('plyfiles')[:2]:
        main(filename)