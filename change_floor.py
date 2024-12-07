import torch
import numpy as np
import pytorch3d
from pytorch3d.structures import Pointclouds
from pytorch3d.ops import estimate_pointcloud_normals
import open3d as o3d

def load_ply_to_pytorch3d(file_path):
    """
    Load a .ply file and convert it to a PyTorch3D Pointclouds object.
    
    Args:
        file_path (str): Path to the .ply file
    
    Returns:
        Pointclouds: PyTorch3D point cloud object
        torch.Tensor: Original point cloud as a tensor
    """
    # Use Open3D to read the PLY file
    pcd = o3d.io.read_point_cloud(file_path)
    points = np.asarray(pcd.points)
    
    # Convert to PyTorch tensor
    points_tensor = torch.from_numpy(points).float()
    
    # Create PyTorch3D Pointclouds object
    pointcloud = Pointclouds(points=[points_tensor])
    
    return pointcloud, points_tensor

def detect_floor_plane(points):
    """
    Detect the floor plane using RANSAC plane estimation.
    
    Args:
        points (torch.Tensor): Point cloud points
    
    Returns:
        torch.Tensor: Plane normal vector
        torch.Tensor: A point on the plane
    """
    # Estimate point cloud normals
    normals = estimate_pointcloud_normals(points.unsqueeze(0), k=10)
    normals = normals.squeeze()
    
    # Find the most common normal direction (likely floor normal)
    unique_normals, counts = torch.unique(torch.round(normals * 10) / 10, dim=0, return_counts=True)
    floor_normal = unique_normals[torch.argmax(counts)]
    
    # Find a point on the plane (average of points with similar normal)
    normal_dot_products = torch.abs(torch.mm(normals, floor_normal.unsqueeze(1)).squeeze())
    plane_points_mask = normal_dot_products > 0.9
    plane_point = points[plane_points_mask].mean(dim=0)
    
    return floor_normal, plane_point

def reorient_point_cloud(points, floor_normal, plane_point):
    """
    Reorient point cloud so that floor is on YZ plane at y=0.
    
    Args:
        points (torch.Tensor): Original point cloud
        floor_normal (torch.Tensor): Normal vector of the floor plane
        plane_point (torch.Tensor): A point on the floor plane
    
    Returns:
        torch.Tensor: Reoriented point cloud
    """
    # Normalize the floor normal
    floor_normal = floor_normal / torch.norm(floor_normal)
    
    # Calculate rotation to align floor normal with [0, 1, 0]
    target_normal = torch.tensor([0., 1., 0.], device=points.device)
    
    # Cross product to get rotation axis
    rot_axis = torch.cross(floor_normal, target_normal)
    rot_axis = rot_axis / torch.norm(rot_axis)
    
    # Calculate rotation angle
    cos_theta = torch.dot(floor_normal, target_normal)
    theta = torch.arccos(cos_theta)
    
    # Create rotation matrix using Rodriguez rotation formula
    K = torch.tensor([
        [0, -rot_axis[2], rot_axis[1]],
        [rot_axis[2], 0, -rot_axis[0]],
        [-rot_axis[1], rot_axis[0], 0]
    ], device=points.device)
    
    I = torch.eye(3, device=points.device)
    R = I + torch.sin(theta) * K + (1 - cos_theta) * torch.mm(K, K)
    
    # Rotate points
    rotated_points = torch.mm(points, R.T)
    
    # Translate to center on y=0 plane
    z_mean = rotated_points[:, 2].mean()
    x_mean = rotated_points[:, 0].mean()
    
    translated_points = rotated_points.clone()
    translated_points[:, 2] -= z_mean
    translated_points[:, 0] -= x_mean
    
    return translated_points

def process_point_cloud(file_path):
    """
    Main processing function to load, detect floor, and reorient point cloud.
    
    Args:
        file_path (str): Path to the input .ply file
    
    Returns:
        torch.Tensor: Reoriented point cloud
        torch.Tensor: Original point cloud
    """
    # Load point cloud
    pointcloud, points = load_ply_to_pytorch3d(file_path)
    
    # Detect floor plane
    floor_normal, plane_point = detect_floor_plane(points)
    
    # Reorient point cloud
    reoriented_points = reorient_point_cloud(points, floor_normal, plane_point)
    
    return reoriented_points, points

def save_reoriented_point_cloud(reoriented_points, output_path):
    """
    Save reoriented point cloud to a .ply file.
    
    Args:
        reoriented_points (torch.Tensor): Reoriented point cloud
        output_path (str): Path to save the output .ply file
    """
    # Convert to numpy for Open3D
    points_np = reoriented_points.numpy()
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_np)
    
    # Save to file
    o3d.io.write_point_cloud(output_path, pcd)

# Example usage
if __name__ == "__main__":
    input_file = "plyfiles/shoe_pc.ply"
    output_file = "result_plyfiles/shoe_pc.ply"
    
    # Process point cloud
    reoriented_points, original_points = process_point_cloud(input_file)
    
    # Save reoriented point cloud
    save_reoriented_point_cloud(reoriented_points, output_file)
    
    print(f"Reoriented point cloud saved to {output_file}")