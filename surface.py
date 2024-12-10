# import numpy as np
# import open3d as o3d

# def reconstruct_surface_poisson(point_cloud, depth=9, linear_fit=False):
#     """
#     Reconstruct surface using Poisson Surface Reconstruction
    
#     Args:
#         point_cloud (o3d.geometry.PointCloud): Input point cloud
#         depth (int): Octree depth for reconstruction (higher = more detailed)
#         linear_fit (bool): Whether to use linear interpolation
    
#     Returns:
#         o3d.geometry.TriangleMesh: Reconstructed surface mesh
#     """
#     # Estimate normals if not already computed
#     if not point_cloud.has_normals():
#         point_cloud.estimate_normals()
#         point_cloud.orient_normals_consistent_tangent_plane(100)
    
#     # Poisson surface reconstruction
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         point_cloud, depth=depth, linear_fit=linear_fit
#     )
    
#     return mesh

# def reconstruct_surface_ball_pivoting(point_cloud, radii=[0.01, 0.02, 0.04]):
#     """
#     Reconstruct surface using Ball Pivoting Algorithm
    
#     Args:
#         point_cloud (o3d.geometry.PointCloud): Input point cloud
#         radii (list): List of ball radii for reconstruction
    
#     Returns:
#         o3d.geometry.TriangleMesh: Reconstructed surface mesh
#     """
#     # Estimate normals if not already computed
#     if not point_cloud.has_normals():
#         point_cloud.estimate_normals()
#         point_cloud.orient_normals_consistent_tangent_plane(100)
    
#     # Ball Pivoting Algorithm
#     mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#         point_cloud, o3d.utility.DoubleVector(radii)
#     )
    
#     return mesh

# def post_process_mesh(mesh, remove_outliers=True, smooth_iterations=10):
#     """
#     Post-process the reconstructed mesh
    
#     Args:
#         mesh (o3d.geometry.TriangleMesh): Input mesh
#         remove_outliers (bool): Whether to remove small clusters
#         smooth_iterations (int): Number of smoothing iterations
    
#     Returns:
#         o3d.geometry.TriangleMesh: Processed mesh
#     """
#     # Remove duplicate vertices and degenerate triangles
#     mesh.remove_duplicated_vertices()
#     mesh.remove_degenerate_triangles()
    
#     # Remove small clusters of triangles
#     if remove_outliers:
#         mesh = mesh.cluster_connected_triangles()[0]
    
#     print(mesh)
#     print(smooth_iterations)
#     exit()

#     # Optional mesh smoothing
#     if smooth_iterations > 0:
#         mesh.filter_smooth_laplacian(number_of_iterations=smooth_iterations)
    
#     # Compute vertex normals for better rendering
#     mesh.compute_vertex_normals()
    
#     return mesh

# def main():
#     # Load the reoriented point cloud
#     ply_file_path = 'plyfiles/shoe_pc.ply'
#     point_cloud = o3d.io.read_point_cloud(ply_file_path)
    
#     # Reconstruct surface using Poisson method
#     poisson_mesh = reconstruct_surface_poisson(point_cloud)
#     processed_poisson_mesh = post_process_mesh(poisson_mesh)
    
#     # Reconstruct surface using Ball Pivoting method
#     ball_pivoting_mesh = reconstruct_surface_ball_pivoting(point_cloud)
#     processed_ball_pivoting_mesh = post_process_mesh(ball_pivoting_mesh)
    
#     # Visualize and save both meshes
#     o3d.visualization.draw_geometries([processed_poisson_mesh])
#     o3d.visualization.draw_geometries([processed_ball_pivoting_mesh])
    
#     # Save reconstructed meshes
#     o3d.io.write_triangle_mesh('poisson_mesh.ply', processed_poisson_mesh)
#     o3d.io.write_triangle_mesh('ball_pivoting_mesh.ply', processed_ball_pivoting_mesh)

# if __name__ == '__main__':
#     main()

#########################################################################################################################################################

# import numpy as np
# import open3d as o3d

# def reconstruct_surface_poisson(point_cloud, depth=9, linear_fit=False):
#     """
#     Reconstruct surface using Poisson Surface Reconstruction
    
#     Args:
#         point_cloud (o3d.geometry.PointCloud): Input point cloud
#         depth (int): Octree depth for reconstruction (higher = more detailed)
#         linear_fit (bool): Whether to use linear interpolation
    
#     Returns:
#         o3d.geometry.TriangleMesh: Reconstructed surface mesh
#     """
#     # Estimate normals if not already computed
#     if not point_cloud.has_normals():
#         point_cloud.estimate_normals()
#         point_cloud.orient_normals_consistent_tangent_plane(100)
    
#     # Poisson surface reconstruction with error handling
#     try:
#         # Try the newer API first
#         mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#             point_cloud, depth=depth, linear_fit=linear_fit
#         )
#     except Exception as e:
#         # Fallback to alternative method if the first approach fails
#         print(f"Primary Poisson reconstruction failed: {e}")
#         print("Attempting alternative reconstruction method...")
        
#         # Alternative method using explicit conversion
#         mesh = o3d.geometry.TriangleMesh()
        
#         # Convert point cloud to numpy array
#         points = np.asarray(point_cloud.points)
#         normals = np.asarray(point_cloud.normals)
        
#         # Add vertices and normals to mesh
#         mesh.vertices = o3d.utility.Vector3dVector(points)
#         mesh.vertex_normals = o3d.utility.Vector3dVector(normals)
    
#     return mesh

# def reconstruct_surface_ball_pivoting(point_cloud, radii=None):
#     """
#     Reconstruct surface using Ball Pivoting Algorithm
    
#     Args:
#         point_cloud (o3d.geometry.PointCloud): Input point cloud
#         radii (list): List of ball radii for reconstruction
    
#     Returns:
#         o3d.geometry.TriangleMesh: Reconstructed surface mesh
#     """
#     # Estimate normals if not already computed
#     if not point_cloud.has_normals():
#         point_cloud.estimate_normals()
#         point_cloud.orient_normals_consistent_tangent_plane(100)
    
#     # If no radii provided, compute automatically
#     if radii is None:
#         # Compute radii based on point cloud characteristics
#         points = np.asarray(point_cloud.points)
#         bbox = np.max(points, axis=0) - np.min(points, axis=0)
#         avg_point_distance = np.mean(np.linalg.norm(np.diff(points, axis=0), axis=1))
#         radii = [
#             avg_point_distance * 0.5, 
#             avg_point_distance, 
#             avg_point_distance * 2
#         ]
    
#     # Ball Pivoting Algorithm with error handling
#     try:
#         # Try standard method first
#         mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#             point_cloud, o3d.utility.DoubleVector(radii)
#         )
#     except Exception as e:
#         print(f"Ball pivoting reconstruction failed: {e}")
        
#         # Fallback mesh creation
#         mesh = o3d.geometry.TriangleMesh()
#         points = np.asarray(point_cloud.points)
#         mesh.vertices = o3d.utility.Vector3dVector(points)
    
#     return mesh

# def post_process_mesh(mesh, remove_outliers=True, smooth_iterations=10):
#     """
#     Post-process the reconstructed mesh
    
#     Args:
#         mesh (o3d.geometry.TriangleMesh): Input mesh
#         remove_outliers (bool): Whether to remove small clusters
#         smooth_iterations (int): Number of smoothing iterations
    
#     Returns:
#         o3d.geometry.TriangleMesh: Processed mesh
#     """
#     # Remove duplicate vertices and degenerate triangles
#     mesh.remove_duplicated_vertices()
    
#     # Remove small clusters of triangles
#     if remove_outliers:
#         # Use clustering method with error handling
#         try:
#             # Try to get connected triangles
#             mesh, _, _ = mesh.cluster_connected_triangles()
#         except Exception as e:
#             print(f"Cluster connection failed: {e}")
    
#     # Optional mesh smoothing
#     if smooth_iterations > 0:
#         try:
#             mesh.filter_smooth_laplacian(number_of_iterations=smooth_iterations)
#         except Exception as e:
#             print(f"Mesh smoothing failed: {e}")
    
#     # Compute vertex normals for better rendering
#     mesh.compute_vertex_normals()
    
#     return mesh

# def main():
#     # Load the reoriented point cloud
#     ply_file_path = 'reoriented_shoe.ply'
#     point_cloud = o3d.io.read_point_cloud(ply_file_path)
    
#     # Reconstruct surface using Poisson method
#     poisson_mesh = reconstruct_surface_poisson(point_cloud)
#     processed_poisson_mesh = post_process_mesh(poisson_mesh)
    
#     # Reconstruct surface using Ball Pivoting method
#     ball_pivoting_mesh = reconstruct_surface_ball_pivoting(point_cloud)
#     processed_ball_pivoting_mesh = post_process_mesh(ball_pivoting_mesh)
    
#     # Visualize and save both meshes
#     o3d.visualization.draw_geometries([processed_poisson_mesh])
#     o3d.visualization.draw_geometries([processed_ball_pivoting_mesh])
    
#     # Save reconstructed meshes
#     o3d.io.write_triangle_mesh('poisson_mesh.ply', processed_poisson_mesh)
#     o3d.io.write_triangle_mesh('ball_pivoting_mesh.ply', processed_ball_pivoting_mesh)

# if __name__ == '__main__':
#     main()

#########################################################################################################################################################

import os
import numpy as np
import open3d as o3d
import scipy.spatial

def downsample_point_cloud(point_cloud, voxel_size=0.01):
    """
    Downsample point cloud to reduce memory usage and computational complexity
    
    Args:
        point_cloud (o3d.geometry.PointCloud): Input point cloud
        voxel_size (float): Size of voxel for downsampling
    
    Returns:
        o3d.geometry.PointCloud: Downsampled point cloud
    """
    return point_cloud.voxel_down_sample(voxel_size)

def compute_adaptive_radii(points, percentiles=[10, 50, 90]):
    """
    Compute adaptive radii for surface reconstruction
    
    Args:
        points (np.ndarray): Point cloud coordinates
        percentiles (list): Percentiles for radius computation
    
    Returns:
        list: Adaptive radii
    """
    # Use KD-tree for memory-efficient nearest neighbor distances
    kdtree = scipy.spatial.cKDTree(points)
    
    # Compute k-nearest neighbor distances
    k = min(20, len(points))  # Limit k to prevent memory issues
    distances, _ = kdtree.query(points, k=k)
    
    # Compute percentile distances
    if k > 1:
        # Average distances for each point
        point_distances = np.mean(distances[:, 1:], axis=1)
    else:
        point_distances = distances
    
    # Compute radii based on percentiles
    radii = [np.percentile(point_distances, p) for p in percentiles]
    
    return radii

def reconstruct_surface_memory_efficient(point_cloud, method='poisson', depth=8):
    """
    Memory-efficient surface reconstruction
    
    Args:
        point_cloud (o3d.geometry.PointCloud): Input point cloud
        method (str): Reconstruction method
        depth (int): Reconstruction depth
    
    Returns:
        o3d.geometry.TriangleMesh: Reconstructed mesh
    """
    # Downsample point cloud
    downsampled_cloud = downsample_point_cloud(point_cloud)
    
    # Estimate normals
    downsampled_cloud.estimate_normals()
    downsampled_cloud.orient_normals_consistent_tangent_plane(100)
    
    # Choose reconstruction method
    if method == 'poisson':
        try:
            # Poisson reconstruction with lower depth
            mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                downsampled_cloud, depth=depth
            )
        except Exception as e:
            print(f"Poisson reconstruction failed: {e}")
            # Fallback to alternative method
            points = np.asarray(downsampled_cloud.points)
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(points)
    
    elif method == 'ball_pivoting':
        # Compute memory-efficient radii
        points = np.asarray(downsampled_cloud.points)
        radii = compute_adaptive_radii(points)
        
        try:
            # Ball pivoting reconstruction
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                downsampled_cloud, o3d.utility.DoubleVector(radii)
            )
        except Exception as e:
            print(f"Ball pivoting reconstruction failed: {e}")
            # Fallback to alternative method
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(points)
    
    else:
        raise ValueError("Invalid reconstruction method. Choose 'poisson' or 'ball_pivoting'.")
    
    return mesh

def main(filename):
    # Load the point cloud
    ply_file_path = f'plyfiles/{filename}'
    point_cloud = o3d.io.read_point_cloud(ply_file_path)
    
    # Print point cloud information
    print(f"Original point cloud size: {len(point_cloud.points)} points")
    
    # Reconstruct mesh with memory-efficient methods
    poisson_mesh = reconstruct_surface_memory_efficient(point_cloud, method='poisson')
    ball_pivoting_mesh = reconstruct_surface_memory_efficient(point_cloud, method='ball_pivoting')
    
    # Visualize and save meshes
    o3d.visualization.draw_geometries([poisson_mesh])
    o3d.visualization.draw_geometries([ball_pivoting_mesh])
    
    # Save reconstructed meshes
    o3d.io.write_triangle_mesh(f'poisson_plyfiles/{filename}', poisson_mesh)
    o3d.io.write_triangle_mesh(f'ball_pivot_plyfiles/{filename}', ball_pivoting_mesh)

if __name__ == '__main__':
    for filename in os.listdir('plyfiles'):
        main(filename)