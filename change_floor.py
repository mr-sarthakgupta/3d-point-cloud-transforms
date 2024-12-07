import numpy as np
import open3d as o3d
from scipy.optimize import minimize

def fit_plane_ransac(points, max_iterations=1000, threshold=0.01):
    """
    Use RANSAC to robustly fit a plane to the point cloud
    
    Args:
        points (np.ndarray): Input point cloud points
        max_iterations (int): Maximum RANSAC iterations
        threshold (float): Distance threshold for inliers
    
    Returns:
        tuple: Plane normal and point on the plane
    """
    best_plane = None
    max_inliers = 0
    
    for _ in range(max_iterations):
        # Randomly select 3 points
        sample_indices = np.random.choice(len(points), 3, replace=False)
        sample_points = points[sample_indices]
        
        # Compute plane normal
        v1 = sample_points[1] - sample_points[0]
        v2 = sample_points[2] - sample_points[0]
        normal = np.cross(v1, v2)
        normal /= np.linalg.norm(normal)
        
        # Compute distance of each point to the plane
        distances = np.abs(np.dot(points - sample_points[0], normal))
        
        # Count inliers
        inliers = np.sum(distances < threshold)
        
        if inliers > max_inliers:
            max_inliers = inliers
            best_plane = (normal, sample_points[0])
    
    return best_plane

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
    
    # Detect floor plane using RANSAC
    floor_normal, plane_point = fit_plane_ransac(points)
    
    # Ensure floor normal points upwards 
    if floor_normal[1] < 0:
        floor_normal = -floor_normal
    
    # Create rotation matrix to align floor normal with [0, 1, 0]
    target_up = np.array([0, 1, 0])
    
    # Compute rotation axis and angle
    rotation_axis = np.cross(floor_normal, target_up)
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    
    angle = np.arccos(np.dot(floor_normal, target_up))
    
    # Create rotation matrix
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * angle)
    
    # Apply rotation
    pcd.rotate(R, center=(0,0,0))
    
    # Recompute points after rotation
    points_rotated = np.asarray(pcd.points)
    
    # Find the lowest point on the Y-axis to shift
    min_y = np.min(points_rotated[:, 1])
    
    # Translate to ensure floor is exactly at y=0
    translation = np.array([0, -min_y, 0])
    pcd.translate(translation)
    
    return pcd

def main():
    # Example usage
    ply_file_path = 'plyfiles/shoe_pc.ply'  # Replace with your .ply file path
    reoriented_pcd = detect_and_reorient_floor(ply_file_path)
    
    # Visualize the result
    o3d.visualization.draw_geometries([reoriented_pcd])
    
    # Save the reoriented point cloud
    o3d.io.write_point_cloud('reoriented_shoe.ply', reoriented_pcd)

if __name__ == '__main__':
    main()