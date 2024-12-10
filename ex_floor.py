import numpy as np
import open3d as o3d
from sklearn.linear_model import RANSACRegressor
import traceback
import sys

def detect_and_reorient_floor(ply_file_path):
    """
    Detect the floor plane in a point cloud and reorient the point cloud.
    """
    pcd = o3d.io.read_point_cloud(ply_file_path)
    points = np.asarray(pcd.points)
    
    print(f"Total points: {len(points)}")
    print(f"Points shape: {points.shape}")
    
    
    print(f"NaN values: {np.isnan(points).any()}")
    print(f"Infinite values: {np.isinf(points).any()}")
    
    print(f"X range: {points[:,0].min()} to {points[:,0].max()}")
    print(f"Y range: {points[:,1].min()} to {points[:,1].max()}")
    print(f"Z range: {points[:,2].min()} to {points[:,2].max()}")
    
    reoriented_pcd = o3d.geometry.PointCloud()
    reoriented_pcd.points = o3d.utility.Vector3dVector(points)
    
    print("Point cloud created successfully!")

    return reoriented_pcd
    
# Example usage
if __name__ == '__main__':
    # Check if file path is provided
    if len(sys.argv) < 2:
        print("Please provide the path to the .ply file")
        sys.exit(1)
    
    input_ply_path = sys.argv[1]
    
    try:
        # Detect and process point cloud
        pcd = detect_and_reorient_floor(input_ply_path)
        
        # Visualize the result
        o3d.visualization.draw_geometries([pcd])

        o3d.io.write_point_cloud('reoriented_fig.ply', pcd)
    
    except Exception as e:
        print("Error processing point cloud:")
        print(traceback.format_exc())