# reorient_floor.py

## Overview

This script provides functionality to automatically detect and align floor planes in 3D point cloud data. It uses the RANSAC algorithm for plane detection and implements geometric transformations to reorient the point cloud so that the floor plane becomes parallel to the the plane y = 0.

We first find the plane with the maximum amount of points and identify it as the floor, then we find the angle between the normal to the plane and the desired normal(the y-axis) and then use the cross product of the two to identify the rotation vector and the angle to be rotated and create the rotation matrix accordingly and transform the point cloud. Finally we also centre the floor to the origin. While the algorithm implemented is general and should work for all cases, we have observed the method to faulter when the floor is large (such as in the case of shoe2_pc.ply). Also, in some cases where the angle of perturbation is very large ( > pi) the algorithm is unable to reorient the floor to the correct position even though it finds the floor correctly.


## Main Function: `align_point_cloud_to_floor`


### Parameters
- `input_ply_path` (str): Path to input PLY file
- `output_ply_path` (str, optional): Path to save the aligned point cloud
- `name` (str, optional): Identifier for the point cloud


### Returns
- `pcd`: Aligned point cloud
- `floor_plane`: Parameters of the detected floor plane
- `floor_inliers`: Indices of points belonging to the floor


## Algorithm Steps & Intuition


### 1. Plane Detection
```python
plane_model, inliers = temp_pcd.segment_plane(
    distance_threshold=0.006,
    ransac_n=3,
    num_iterations=1000
)
```

#### Reasoning

- Uses RANSAC (Random Sample Consensus) to robustly detect planes
- Points within `distance_threshold` of the plane are considered inliers
- Multiple plane candidates are stored to ensure the floor is correctly identified


### 2. Floor Identification
```python
plane_candidates.sort(key=lambda x: x[2], reverse=True)
floor_plane, floor_inliers, _ = plane_candidates[0]
```

#### Reasoning
- Assumes the floor is typically the largest planar surface
- Sorts detected planes by number of inlier points
- Selects the plane with most inliers as the floor


### 3. Rotation Alignment
```python
normal = np.array([a, b, c])
standard_normal = np.array([0, -1, 0])
rotation_vector = np.cross(normal, standard_normal)
rotation_angle = np.arccos(np.dot(normal, standard_normal))
```

#### Reasoning
- Computes rotation to align floor normal with Y-axis
- Uses cross product to find rotation axis
- Calculates rotation angle using dot product
- Ensures floor becomes parallel to XZ plane


### 4. Translation
```python
floor_points = np.asarray(pcd.points)[floor_inliers]
floor_center = np.mean(floor_points, axis=0)
pcd.translate(-floor_center)
```

#### Reasoning
- Centers the point cloud at origin
- Uses mean of floor points as reference
- Ensures consistent positioning across different point clouds


## Implementation Details

### Plane Segmentation Loop
- Iteratively segments up to 10 planes
- Stops if remaining points < 5% of original
- Ensures robust floor detection in complex scenes


### Parameters
- `distance_threshold=0.006`: Tolerance for plane fitting
- `ransac_n=3`: Minimum points for plane fitting
- `num_iterations=1000`: RANSAC iterations for reliable results


## Usage Example
```python
input_file = 'path/to/input.ply'
output_file = 'path/to/output.ply'
aligned_pcd, floor_plane, floor_inliers = align_point_cloud_to_floor(input_file, output_file)
```

## Limitations

1. Assumes floor is the largest planar surface
2. May struggle with:
   - Heavily cluttered scenes
   - Multiple large horizontal surfaces
   - Incomplete or noisy floor data

## Directory Structure

- `plyfiles/`: Input point clouds
- `reoriented_plyfiles/`: Aligned output point clouds
- `floors/`: Visualizations of detected floor planes


# surface_smoothing.py

## Overview
This script implements various surface reconstruction methods for 3D point clouds using Open3D. It processes PLY files containing point cloud data and generates smoothed mesh surfaces using three different algorithms: Poisson reconstruction, Ball-Pivoting, and Alpha Shape.

### Surface Reconstruction Methods

python
def surface_reconstruction_methods(point_cloud):

Implements three different surface reconstruction approaches:

#### a) Poisson Surface Reconstruction
- **Algorithm**: Uses an implicit function technique to reconstruct a closed surface
- **Parameters**:
  - `depth=7`: Controls mesh resolution (higher values create finer details)
  - `scale=1.0`: Scales the reconstruction
  - `linear_fit=False`: Determines whether to use linear interpolation

**Advantages**:
- Creates closed surfaces
- Handles noise well
- Good for complete, clean point clouds

**Limitations**:
- May create artifacts in areas with sparse points
- Can over-smooth fine details

#### b) Ball-Pivoting Algorithm (BPA)
- **Algorithm**: Simulates rolling a ball on the point cloud to create triangular faces
- Uses adaptive radii based on point cloud density:
  ```python
  distances = point_cloud.compute_nearest_neighbor_distance()
  avg_dist = np.mean(distances)
  radii = [avg_dist, avg_dist * 2]
  ```

**Advantages**:
- Preserves sharp features
- Works well with evenly distributed points

**Limitations**:
- Creates holes in sparse areas
- Sensitive to noise

#### c) Alpha Shape
- **Algorithm**: Creates a shape by "carving" out the space around points using a sphere of radius alpha
- Uses fixed alpha value of 0.3
- Includes error handling for failed reconstructions

**Advantages**:
- Good for detecting shape boundaries
- Works well with dense point clouds

**Limitations**:
- Sensitive to alpha parameter
- Not suitable for noisy data

### 3. Normal Estimation

python
point_cloud.estimate_normals(
search_param=o3d.geometry.KDTreeSearchParamHybrid(
radius=0.1,
max_nn=30
)
)

- Essential pre-processing step for surface reconstruction
- Uses hybrid search (combines radius and k-nearest neighbors)
- Parameters:
  - `radius=0.1`: Search radius for finding neighboring points
  - `max_nn=30`: Maximum number of neighbors to consider

### 4. File Organization
- Input files stored in `plyfiles/` directory
- Output meshes saved in:
  - `smooth_plyfiles/poisson/`
  - `smooth_plyfiles/bpa/`
  - `smooth_plyfiles/alpha/`

## Usage

1. Place PLY files in the `plyfiles` directory
2. Run the script:

python surface_smoothing.py

3. Reconstructed meshes will be saved in respective directories under `smooth_plyfiles/`

# test_reorient_floor.py
## Overview
This test file validates the functionality of the floor alignment algorithm implemented in `reorient_floor.py`. The primary purpose is to ensure that point clouds are correctly reoriented so that their floor planes align with the y=0 plane.

## Test Strategy
The test employs a property-based testing approach where:
1. A random point cloud is selected from the `plyfiles` directory
2. A random transformation is applied to deliberately misalign it
3. The alignment algorithm is run
4. The results are validated against expected geometric properties

### Validation Criteria

1. Floor Point Detection:
python
assert len(floor_inliers) > 100, "Not enough floor points detected"

- Ensures the algorithm identifies a sufficient number of floor points
- The threshold of 100 points is a reasonable minimum for a valid floor plane

2. Floor Alignment:

python
assert np.mean(np.abs(aligned_points[floor_inliers][:, 1])) < 0.05

- Verifies that the identified floor points are close to y=0 after alignment
- Tolerance of 0.05 units allows for minor numerical imprecisions while ensuring practical alignment

## Design Rationale

### Why 30-Degree Limit?
- Realistic scenario modeling
- Helps avoid edge cases where floor detection might be ambiguous

### Why 0.05 Alignment Tolerance?
- Balances precision with practical requirements
- Accounts for numerical precision limitations
- Sufficient for most real-world applications

## Usage

The test can be run in two ways:
1. Using pytest:

pytest test_reorient_floor.py

- The random seed is commented out (`# np.random.seed(42)`) to ensure different random transformations on each run

- The test assumes the existence of a `plyfiles` directory containing valid point cloud files
