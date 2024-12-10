import os
import open3d as o3d
import numpy as np

def load_ply_point_cloud(file_path):
    point_cloud = o3d.io.read_point_cloud(file_path)
    return point_cloud

def surface_reconstruction_methods(point_cloud):
    
    poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        point_cloud, 
        depth=7,  # Controls mesh resolution
        width=0,  # Adaptive depth
        scale=1.0,  # Scales the reconstruction
        linear_fit=False
    )
    poisson_mesh.compute_vertex_normals()
    
    
    distances = point_cloud.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    # radii = [0.1 * avg_dist, 0.25 * avg_dist, 0.5 * avg_dist, avg_dist, avg_dist * 2, avg_dist * 4, avg_dist * 8, avg_dist * 16, avg_dist * 32, avg_dist *64]
    radii = [avg_dist, avg_dist * 2]
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        point_cloud, 
        o3d.utility.DoubleVector(radii)
    )
    bpa_mesh.compute_vertex_normals()
    
    
    tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(point_cloud)
    alpha = 0.3  # Start with a reasonable alpha value
    try:
        alpha_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            point_cloud, alpha, compute_normals=True)
    except (RuntimeError, IndexError):
        # If alpha shape fails, return an empty mesh
        alpha_mesh = o3d.geometry.TriangleMesh()
        alpha_mesh.compute_vertex_normals()
    
    return {
        'poisson_mesh': poisson_mesh, 
        'ball_pivoting_mesh': bpa_mesh, 
        'alpha_shape_mesh': alpha_mesh
    }

def visualize_reconstructed_surface(mesh):
    o3d.visualization.draw_geometries([mesh])

def main(ply_file_path):
    
    point_cloud = load_ply_point_cloud(f"plyfiles/{ply_file_path}")
    
    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1,  # Adjust based on your point cloud scale
            max_nn=30
        )
    )
    
    reconstructed_surfaces = surface_reconstruction_methods(point_cloud)
    
    for method_name, mesh in reconstructed_surfaces.items():
        # if method_name == 'alpha_shape_mesh':
        #     print(f"Visualizing {method_name}")
        #     visualize_reconstructed_surface(mesh)
        if method_name == 'poisson_mesh':
            o3d.io.write_triangle_mesh(f"smooth_plyfiles/poisson/{ply_file_path}", mesh)
        if method_name == 'ball_pivoting_mesh':
            o3d.io.write_triangle_mesh(f"smooth_plyfiles/bpa/{ply_file_path}", mesh)
        if method_name == 'alpha_shape_mesh':
            o3d.io.write_triangle_mesh(f"smooth_plyfiles/alpha/{ply_file_path}", mesh)

if __name__ == "__main__":
    for filename in os.listdir('plyfiles'):
        main(filename)