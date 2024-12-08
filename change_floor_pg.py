import numpy as np
import open3d as o3d
import os

def detect_and_reorient_floor(input_ply_path):
    """
    Detect the floor plane in a point cloud and precisely reorient it to the YZ plane 
    with the floor exactly at y=0.
    
    Parameters:
    -----------
    input_ply_path : str
        Path to the input .ply file
    
    Returns:
    --------
    o3d.geometry.PointCloud
        Reoriented point cloud with floor exactly at y=0
    """
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(input_ply_path)
    
    # Convert to numpy array for easier manipulation
    points = np.asarray(pcd.points)
    
    # Check if there are enough points
    if len(points) < 3:
        raise ValueError(f"Point cloud must have at least 3 points. Current point count: {len(points)}")
    
    def estimate_plane_pca(points):
        """
        Estimate plane using PCA
        """
        # Center the points
        centroid = points.mean(axis=0)
        centered_points = points - centroid
        
        # Compute covariance matrix
        cov_matrix = np.cov(centered_points.T)
        
        # Eigenvalue decomposition
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        
        # The eigenvector corresponding to the smallest eigenvalue 
        # is the normal to the plane
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        
        # Plane equation: ax + by + cz + d = 0
        d = -np.dot(normal, centroid)
        
        return list(normal) + [d]
    
    # Estimate plane model
    plane_model = estimate_plane_pca(points)
    
    # Extract plane equation coefficients (ax + by + cz + d = 0)
    a, b, c, d = plane_model
    
    # Normalize the plane coefficients
    magnitude = np.sqrt(a**2 + b**2 + c**2)
    a, b, c, d = a/magnitude, b/magnitude, c/magnitude, d/magnitude
    
    # Compute rotation to align floor with YZ plane
    current_normal = np.array([a, b, c])
    
    # Compute rotation matrix using Rodrigues' rotation formula
    def rotation_matrix(axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        axis = np.asarray(axis)
        axis = axis / np.sqrt(np.dot(axis, axis))
        a = np.cos(theta / 2.0)
        b, c, d = -axis * np.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                         [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                         [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])
    
    # Compute rotation angles
    z_axis = np.array([0, 0, 1])
    rotation_axis = np.cross(current_normal, z_axis)
    rotation_angle = np.arccos(np.dot(current_normal, z_axis))
    
    # Create rotation matrix
    R = rotation_matrix(rotation_axis, rotation_angle)
    
    # Apply rotation to point cloud
    rotated_points = np.dot(points, R.T)
    
    # Create new point cloud with rotated points
    rotated_pcd = o3d.geometry.PointCloud()
    rotated_pcd.points = o3d.utility.Vector3dVector(rotated_points)
    
    # Copy colors and normals if they exist
    if pcd.has_colors():
        rotated_pcd.colors = pcd.colors
    if pcd.has_normals():
        rotated_normals = np.dot(np.asarray(pcd.normals), R.T)
        rotated_pcd.normals = o3d.utility.Vector3dVector(rotated_normals)
    
    # Precise floor point identification and centering
    # Use 0.1st percentile to identify floor points
    floor_y_shift = np.percentile(rotated_points[:, 1], 0.1)
    
    # Translate points to ensure floor is exactly at y=0
    final_points = rotated_points.copy()
    final_points[:, 1] -= floor_y_shift
    
    # Create final point cloud
    final_pcd = o3d.geometry.PointCloud()
    final_pcd.points = o3d.utility.Vector3dVector(final_points)
    
    # Copy colors and normals to final point cloud
    if pcd.has_colors():
        final_pcd.colors = rotated_pcd.colors
    if pcd.has_normals():
        final_normals = np.dot(np.asarray(rotated_pcd.normals), np.eye(3))
        final_pcd.normals = o3d.utility.Vector3dVector(final_normals)
    
    return final_pcd

def save_reoriented_pointcloud(pcd, output_ply_path):
    """
    Save the reoriented point cloud to a .ply file
    
    Parameters:
    -----------
    pcd : o3d.geometry.PointCloud
        Reoriented point cloud
    output_ply_path : str
        Path to save the output .ply file
    """
    o3d.io.write_point_cloud(output_ply_path, pcd)

# Example usage
for filename in os.listdir('plyfiles'):
    input_path = f'plyfiles/{filename}'
    output_path = f'output_plyfiles/{filename}'

    # Detect floor and reorient point cloud
    reoriented_pcd = detect_and_reorient_floor(input_path)

    # Save the reoriented point cloud
    save_reoriented_pointcloud(reoriented_pcd, output_path)

    # Optional: Visualization (uncomment if needed)
    # o3d.visualization.draw_geometries([reoriented_pcd])