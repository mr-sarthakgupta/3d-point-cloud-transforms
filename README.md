## 3D Point Cloud Transforms

This repository contains code for transforming 3D point clouds. There are two primary transformations implemented. `reorient_floor.py` contains code to identify the floor of the point cloud and rotate and recentre the point cloud to make the floor lie on `y = 0` and centering the floor on origin. `surface_smoothing.py` contains code for converting the point cloud to other representations namely *poisson mesh*, *ball pivoting mesh* and *alpha shape mesh*.

The point cloud `.ply` files are organized as follows:

- `plyfiles` contains all the original point clouds
- `floors` contains the point clouds of the floors extracted using `align_point_cloud_to_floor` function from `reorient_floor.py`
- `reoriented_plyfiles` contains all the reoriented point clouds with their floors coincinding with `y = 0` and centered at the origin
- `smooth_plyfiles` contains three subdirectories, each corresponding to a different representation generated using `surface_smoothing.py`
