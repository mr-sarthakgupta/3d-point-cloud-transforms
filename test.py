import open3d as o3d
import numpy as np
import unittest
import copy

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

def generate_random_point_cloud(num_points=1000, noise_scale=0.1):
    """
    Generate a synthetic point cloud for testing
    
    Args:
        num_points (int): Number of points to generate
        noise_scale (float): Scale of random noise to add
    
    Returns:
        open3d.geometry.PointCloud: Synthetic point cloud
    """
    # Create a basic point cloud (e.g., points on a sphere)
    points = np.random.randn(num_points, 3)
    points /= np.linalg.norm(points, axis=1)[:, np.newaxis]
    
    # Add some noise
    points += np.random.normal(0, noise_scale, points.shape)
    
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    
    return point_cloud

def apply_random_transformations(point_cloud):
    """
    Apply random transformations to a point cloud
    
    Args:
        point_cloud (open3d.geometry.PointCloud): Input point cloud
    
    Returns:
        open3d.geometry.PointCloud: Transformed point cloud
    """
    # Create a deep copy to avoid modifying the original
    transformed_cloud = copy.deepcopy(point_cloud)
    
    # Random translation
    translation = np.random.uniform(-1, 1, 3)
    transformed_cloud.translate(translation)
    
    # Random rotation
    rotation_angle = np.random.uniform(0, 2 * np.pi)
    rotation_axis = np.random.rand(3)
    rotation_axis /= np.linalg.norm(rotation_axis)
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(
        rotation_axis * rotation_angle
    )
    transformed_cloud.rotate(rotation_matrix)
    
    # Random scaling
    scale_factor = np.random.uniform(0.5, 2.0)
    transformed_cloud.scale(scale_factor, center=(0, 0, 0))
    
    return transformed_cloud

def surface_reconstruction_methods(point_cloud):
    """
    Demonstrate multiple surface reconstruction techniques
    
    Args:
        point_cloud (open3d.geometry.PointCloud): Input point cloud
    
    Returns:
        dict: Reconstructed surfaces using different methods
    """
    # Estimate normals before reconstruction
    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1,  # Adjust based on your point cloud scale
            max_nn=30
        )
    )
    
    # Method 1: Poisson Surface Reconstruction
    poisson_mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        point_cloud, 
        depth=9,
        width=0,
        scale=1.1,
        linear_fit=False
    )
    poisson_mesh.compute_vertex_normals()
    
    # Method 2: Ball Pivoting Algorithm (BPA)
    distances = point_cloud.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radii = [avg_dist, avg_dist * 2]
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        point_cloud, 
        o3d.utility.DoubleVector(radii)
    )
    bpa_mesh.compute_vertex_normals()
    
    # Method 3: Alpha Shape Reconstruction
    tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(point_cloud)
    alpha = 0.1
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

class TestPointCloudReconstruction(unittest.TestCase):
    def setUp(self):
        """
        Generate a synthetic point cloud for testing
        """
        self.original_point_cloud = generate_random_point_cloud()
    
    def test_point_cloud_transformations(self):
        """
        Test that point cloud transformations don't break reconstruction
        """
        # Apply random transformations
        transformed_cloud = apply_random_transformations(self.original_point_cloud)
        
        # Verify basic properties are maintained
        self.assertGreater(len(transformed_cloud.points), 0, 
            "Transformed point cloud should not be empty")
        
        # Ensure transformation didn't remove points
        self.assertEqual(len(self.original_point_cloud.points), 
                         len(transformed_cloud.points), 
                         "Point count should remain constant")
    
    def test_surface_reconstruction_methods(self):
        """
        Test surface reconstruction methods with transformed point cloud
        """
        # Apply transformations
        transformed_cloud = apply_random_transformations(self.original_point_cloud)
        
        # Perform reconstruction
        reconstructed_surfaces = surface_reconstruction_methods(transformed_cloud)
        
        # Verify each reconstruction method produces a valid mesh
        for method_name, mesh in reconstructed_surfaces.items():
            # Check mesh is not empty
            self.assertTrue(len(mesh.vertices) > 0, 
                f"{method_name} should produce non-empty mesh")
            
            # Check mesh has triangles
            self.assertTrue(len(mesh.triangles) > 0, 
                f"{method_name} should produce triangulated mesh")
            
            # Verify mesh has vertex normals
            normals = np.asarray(mesh.vertex_normals)
            self.assertEqual(len(normals), len(mesh.vertices), 
                f"{method_name} should have vertex normals for each vertex")
    
    def test_reconstruction_consistency(self):
        """
        Test that reconstruction methods produce somewhat similar meshes
        """
        # Perform reconstruction
        reconstructed_surfaces = surface_reconstruction_methods(self.original_point_cloud)
        
        # Compare number of vertices and triangles between methods
        methods = list(reconstructed_surfaces.keys())
        for i in range(len(methods)):
            for j in range(i+1, len(methods)):
                mesh1 = reconstructed_surfaces[methods[i]]
                mesh2 = reconstructed_surfaces[methods[j]]
                
                # Check vertex count is in a reasonable range
                vertices_ratio = len(mesh1.vertices) / len(mesh2.vertices)
                self.assertTrue(0.5 <= vertices_ratio <= 2.0, 
                    f"Vertex count between {methods[i]} and {methods[j]} should be similar")
                
                # Check triangle count is in a reasonable range
                triangles_ratio = len(mesh1.triangles) / len(mesh2.triangles)
                self.assertTrue(0.5 <= triangles_ratio <= 2.0, 
                    f"Triangle count between {methods[i]} and {methods[j]} should be similar")

def main():
    """
    Run unit tests
    """
    unittest.main(argv=[''], exit=False)

if __name__ == '__main__':
    main()