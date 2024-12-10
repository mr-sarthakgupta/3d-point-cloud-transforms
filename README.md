## 3D Point Cloud Transforms

This repository contains code for transforming 3D point clouds. There are two primary transformations implemented. `reorient_floor.py` contains code to identify the floor of the point cloud and rotate and recentre the point cloud to make the floor lie on `y = 0` and centering the floor on origin. `surface_smoothing.py` contains code for converting the point cloud to other representations namely *poisson mesh*, *ball pivoting mesh* and *alpha shape mesh*.

The point cloud `.ply` files are organized as follows:

- `plyfiles` contains all the original point clouds
- `floors` contains the point clouds of the floors extracted using `align_point_cloud_to_floor` function from `reorient_floor.py`
- `reoriented_plyfiles` contains all the reoriented point clouds with their floors coincinding with `y = 0` and centered at the origin
- `smooth_plyfiles` contains three subdirectories, each corresponding to a different representation generated using `surface_smoothing.py`

Also, the file `test_reoriemt_floor.py` tests the algorithm for finding and rotating the floor to the desired orientation. We first find the plane with the maximum amount of points and identify it as the floor, then we find the angle between the normal to the plane and the desired normal(the y-axis) and then use the cross product of the two to identify the rotation vector and the angle to be rotated and create the matrix and transform the point cloud. Finally we also centre the floor to the origin. While the algorithm implemented is general and should work for all cases, I have observed the method to faulter when the floor is large (such as in the case of `shoe2_pc.ply`). Also, in some cases where the angle of perturbation is very large (` > pi`) the algorithm is unable to reorient the floor to the correct position even though it finds the floor correctly.

Poisson mesh requires passing the depth parameter, which alters the granularity of of the 3D reconstruction, finding this depth parameter is important to build a suitable representation but it still cannot create a very appealing reconstruction. Ball pivoting mesh hits any 3 points (and it does not fall through those 3 points) it creates a triangles. This creates a visually sparse representation and it depends on the selection of the radii of the spheres. The alpha shape mesh creates a triangular mesh. This last representation looks visually the best.

Ideally to make better surface reconstruction we could probably use a combination of different reconstruction methods to use the best part from each of them, like the cleanliness of the reconstruction from ball-pivoting while having the continuity of surface from the other two.

For improving upon the reorientation algorithm, I find it very strange that the algorithm is able to find the floor in all cases but sometimes can't find the correct transformation as it should be a straightforwrd task given the surface.

I have attached all the .ply files resulting from the transformations with this repository for visualization.
