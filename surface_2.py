import open3d as o3d
import numpy as np

def load_ply_point_cloud(file_path):
    """
    Load a .ply point cloud file using Open3D
    
    Args:
        file_path (str): Path to the .ply file
    
    Returns:
        open3d.geometry.PointCloud: Loaded point cloud
    """
    point_cloud = o3d.io.read_point_cloud(file_path)
    return point_cloud

def surface_reconstruction_methods(point_cloud):
    """
    Demonstrate multiple surface reconstruction techniques
    
    Args:
        point_cloud (open3d.geometry.PointCloud): Input point cloud
    
    Returns:
        dict: Reconstructed surfaces using different methods
    """
    # Method 1: Poisson Surface Reconstruction
    # This method creates a watertight surface with smooth interpolation
    poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        point_cloud, 
        depth=9,  # Controls mesh resolution
        width=0,  # Adaptive depth
        scale=1.1,  # Scales the reconstruction
        linear_fit=False
    )
    poisson_mesh.compute_vertex_normals()
    
    # Method 2: Ball Pivoting Algorithm (BPA)
    # Good for preserving local geometry details
    distances = point_cloud.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radii = [avg_dist, avg_dist * 2]
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        point_cloud, 
        o3d.utility.DoubleVector(radii)
    )
    bpa_mesh.compute_vertex_normals()
    
    # Method 3: Alpha Shape Reconstruction
    # Provides a more conservative surface reconstruction
    tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(point_cloud)
    alpha = 0.1  # Adjust this for different surface granularity
    alpha_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
        point_cloud, 
        alpha, 
        tetra_mesh, 
        pt_map
    )
    alpha_mesh.compute_vertex_normals()
    
    return {
        'poisson_mesh': poisson_mesh, 
        'ball_pivoting_mesh': bpa_mesh, 
        'alpha_shape_mesh': alpha_mesh
    }

def visualize_reconstructed_surface(mesh):
    """
    Visualize the reconstructed surface
    
    Args:
        mesh (open3d.geometry.TriangleMesh): Mesh to visualize
    """
    o3d.visualization.draw_geometries([mesh])

def main(ply_file_path):
    """
    Main workflow for point cloud surface reconstruction
    
    Args:
        ply_file_path (str): Path to input .ply file
    """
    # Load point cloud
    point_cloud = load_ply_point_cloud(ply_file_path)
    
    # Preprocess: Estimate normals (helpful for surface reconstruction)
    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1,  # Adjust based on your point cloud scale
            max_nn=30
        )
    )
    
    # Perform surface reconstruction
    reconstructed_surfaces = surface_reconstruction_methods(point_cloud)
    
    # Optional: Visualize each method
    for method_name, mesh in reconstructed_surfaces.items():
        print(f"Visualizing {method_name}")
        visualize_reconstructed_surface(mesh)

# Example usage
main('plyfiles/shoe_pc.ply')