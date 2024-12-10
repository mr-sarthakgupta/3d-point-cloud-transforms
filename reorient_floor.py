import os
import numpy as np
import open3d as o3d

def align_point_cloud_to_floor(input_ply_path, output_ply_path=None, name=None):
    # Read the point cloud
    pcd = o3d.io.read_point_cloud(input_ply_path)
    points = np.asarray(pcd.points)
    
    # Store plane candidates
    plane_candidates = []
    
    # Remaining points to segment
    remaining_points = points.copy()
    
    # Try multiple plane segmentations to find all potential planes
    max_planes = 10  # Limit to prevent infinite loop
    for _ in range(max_planes):
        if len(remaining_points) < 0.05 * len(points):
            break
        
        # Create a temporary point cloud from remaining points
        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(remaining_points)
        
        # Detect plane using RANSAC
        plane_model, inliers = temp_pcd.segment_plane(
            distance_threshold=0.006,  # Adjust based on point cloud density
            ransac_n=3, 
            num_iterations=1000
        )
        
        # If no significant plane found, break
        if len(inliers) < 0.05 * len(points):
            break
        
        # Store the plane candidate
        plane_candidates.append((plane_model, inliers, len(inliers)))
        
        # Remove inliers from remaining points
        remaining_points = np.delete(remaining_points, inliers, axis=0)
    
    # If no planes found
    if not plane_candidates:
        print(f"Could not identify any plane surfaces for {name}")
        return (0, 0)
        # raise ValueError("Could not identify any plane surfaces")
    
    # Sort plane candidates by number of points (descending)
    plane_candidates.sort(key=lambda x: x[2], reverse=True)
    
    # Select the plane with the most points
    floor_plane, floor_inliers, _ = plane_candidates[0]

    inlier_cloud = pcd.select_by_index(floor_inliers)
    inlier_cloud.paint_uniform_color([1, 0, 0])
    o3d.io.write_point_cloud(f"floors/{name}", inlier_cloud)

    a, b, c, d = floor_plane
    # Compute rotation matrix to align floor to y=0
    # Normalize the plane equation
    magnitude = np.sqrt(a**2 + b**2 + c**2)
    a, b, c = a/magnitude, b/magnitude, c/magnitude
    
    # Create rotation matrix
    normal = np.array([a, b, c])
    standard_normal = np.array([0, -1, 0])
    
    # Compute rotation vector
    rotation_vector = np.cross(normal, standard_normal)
    rotation_angle = np.arccos(np.dot(normal, standard_normal))
    
    # Convert to rotation matrix
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_vector * rotation_angle)
    
    # Rotate point cloud
    pcd.rotate(R, center=(0, 0, 0))
    
    # Translate to center floor at origin
    floor_points = np.asarray(pcd.points)[floor_inliers]
    floor_center = np.mean(floor_points, axis=0)
    pcd.translate(-floor_center)
    
    # Optional: save aligned point cloud
    if output_ply_path:
        o3d.io.write_point_cloud(output_ply_path, pcd)
    
    return pcd, floor_plane, floor_inliers

# Example usage
def main():
    for file in os.listdir('plyfiles'):
        input_file = f'plyfiles/{file}'
        output_file = f'reoriented_plyfiles/{file}'
        print(input_file)
        aligned_pcd, floor_plane, floor_inliers = align_point_cloud_to_floor(input_file, output_file, name=file)

        # except Exception as e:
        #     print(f"Error processing point cloud: {e}")

if __name__ == '__main__':
    main()