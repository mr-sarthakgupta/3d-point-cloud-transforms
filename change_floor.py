import torch
import numpy as np

def load_ply(file_path):
    """
    Load a .ply file manually without using Open3D.
    
    Args:
        file_path (str): Path to the .ply file
    
    Returns:
        torch.Tensor: Point cloud points
    """
    def parse_ply_header(file):
        # Read header
        header_lines = []
        while True:
            line = file.readline().decode('utf-8').strip()
            header_lines.append(line)
            if line == 'end_header':
                break
        
        # Find vertex count and data start
        vertex_count = None
        for line in header_lines:
            if line.startswith('element vertex'):
                vertex_count = int(line.split()[-1])
        
        return vertex_count
    
    # Open file in binary mode
    with open(file_path, 'rb') as f:
        # Parse header
        vertex_count = parse_ply_header(f)
        
        # Read point cloud data
        dtype = np.dtype([
            ('x', 'float32'),
            ('y', 'float32'),
            ('z', 'float32'),
            ('nx', 'float32', (3,)),  # Optional: normals
            ('color', 'uint8', (3,))  # Optional: colors
        ])
        
        # Read point data
        point_data = np.fromfile(f, dtype=dtype, count=vertex_count)
        
        # Extract xyz coordinates
        points = np.column_stack((
            point_data['x'], 
            point_data['y'], 
            point_data['z']
        ))
    
    # Convert to PyTorch tensor
    return torch.from_numpy(points).float()

def estimate_plane_ransac(points, num_iterations=100, threshold=0.01):
    """
    Estimate floor plane using RANSAC.
    
    Args:
        points (torch.Tensor): Input point cloud
        num_iterations (int): Number of RANSAC iterations
        threshold (float): Inlier threshold
    
    Returns:
        torch.Tensor: Plane normal
        torch.Tensor: Point on the plane
    """
    best_normal = None
    max_inliers = 0
    
    for _ in range(num_iterations):
        # Randomly sample 3 points to define a plane
        indices = torch.randperm(len(points))[:3]
        sample_points = points[indices]
        
        # Compute plane normal using cross product
        v1 = sample_points[1] - sample_points[0]
        v2 = sample_points[2] - sample_points[0]
        normal = torch.cross(v1, v2)
        normal = normal / torch.norm(normal)
        
        # Compute distance of each point to the plane
        # Plane equation: dot(normal, x - point_on_plane) = 0
        distances = torch.abs(torch.mm(points - sample_points[0], normal.unsqueeze(1)).squeeze())
        
        # Count inliers
        inliers = (distances < threshold)
        num_inliers = inliers.sum()
        
        # Update best plane if more inliers found
        if num_inliers > max_inliers:
            max_inliers = num_inliers
            best_normal = normal
            best_point = sample_points[0]
    
    return best_normal, best_point

def compute_rotation_matrix(current_normal, target_normal):
    """
    Compute rotation matrix to align current normal with target normal.
    
    Args:
        current_normal (torch.Tensor): Current plane normal
        target_normal (torch.Tensor): Target normal (typically [0, 1, 0])
    
    Returns:
        torch.Tensor: 3x3 rotation matrix
    """
    # Normalize input vectors
    current_normal = current_normal / torch.norm(current_normal)
    target_normal = target_normal / torch.norm(target_normal)
    
    # Compute rotation axis (cross product)
    rot_axis = torch.cross(current_normal, target_normal)
    
    # If normals are parallel, return identity matrix
    if torch.norm(rot_axis) < 1e-6:
        return torch.eye(3, dtype=current_normal.dtype, device=current_normal.device)
    
    rot_axis = rot_axis / torch.norm(rot_axis)
    
    # Compute rotation angle
    cos_theta = torch.dot(current_normal, target_normal)
    theta = torch.arccos(cos_theta)
    
    # Rodrigues' rotation formula
    K = torch.tensor([
        [0, -rot_axis[2], rot_axis[1]],
        [rot_axis[2], 0, -rot_axis[0]],
        [-rot_axis[1], rot_axis[0], 0]
    ], device=current_normal.device, dtype=current_normal.dtype)
    
    I = torch.eye(3, device=current_normal.device, dtype=current_normal.dtype)
    R = I + torch.sin(theta) * K + (1 - cos_theta) * torch.mm(K, K)
    
    return R

def reorient_point_cloud(points, floor_normal, floor_point):
    """
    Reorient point cloud to place floor on YZ plane.
    
    Args:
        points (torch.Tensor): Input point cloud
        floor_normal (torch.Tensor): Estimated floor normal
        floor_point (torch.Tensor): A point on the floor plane
    
    Returns:
        torch.Tensor: Reoriented point cloud
    """
    # Target normal (floor will be aligned with Y-axis)
    target_normal = torch.tensor([0., 1., 0.], device=points.device)
    
    # Compute rotation matrix
    R = compute_rotation_matrix(floor_normal, target_normal)
    
    # Rotate points
    rotated_points = torch.mm(points, R.T)
    
    # Center the point cloud on YZ plane
    x_mean = rotated_points[:, 0].mean()
    z_mean = rotated_points[:, 2].mean()
    
    # Translate points
    centered_points = rotated_points.clone()
    centered_points[:, 0] -= x_mean
    centered_points[:, 2] -= z_mean
    
    return centered_points

def save_ply(points, output_path):
    """
    Save point cloud to a .ply file manually.
    
    Args:
        points (torch.Tensor): Point cloud to save
        output_path (str): Output file path
    """
    # Convert to NumPy
    points_np = points.numpy()
    
    # Open file for writing
    with open(output_path, 'wb') as f:
        # Write PLY header
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {len(points_np)}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "end_header\n"
        )
        f.write(header.encode('utf-8'))
        
        # Write point data
        points_np.astype('<f4').tofile(f)

def process_point_cloud(input_path, output_path):
    """
    Main processing function.
    
    Args:
        input_path (str): Input .ply file path
        output_path (str): Output .ply file path
    """
    # Load point cloud
    points = load_ply(input_path)
    
    # Estimate floor plane
    floor_normal, floor_point = estimate_plane_ransac(points)
    
    # Reorient point cloud
    reoriented_points = reorient_point_cloud(points, floor_normal, floor_point)
    
    # Save reoriented point cloud
    save_ply(reoriented_points, output_path)
    
    print(f"Processed point cloud saved to {output_path}")
    print(f"Estimated floor normal: {floor_normal}")

# Example usage
if __name__ == "__main__":
    input_file = "plyfiles/shoe_pc.ply"
    output_file = "output_plyfiles/shoe_pc.ply"
    
    process_point_cloud(input_file, output_file)