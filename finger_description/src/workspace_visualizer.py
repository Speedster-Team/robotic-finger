from itertools import product
import time

import modern_robotics as mr
import numpy as np
import open3d as o3d


def get_dip_angle(pip_angle):

    pip_angle_deg = np.rad2deg(pip_angle)

    # Polynomial coefficients for dip_angle (highest degree first)
    coeffs = [1.70488603e-12,
              -4.32223057e-10,
              4.06376685e-08,
              -1.09825341e-06,
              -7.29328873e-05,
              4.98961830e-03,
              9.22375906e-01,
              1.04087208e-02]
    # Evaluate dip_angle polynomial via Horner's method
    result = 0.0
    for coeff in coeffs:
        result = result * pip_angle_deg + coeff
    return np.deg2rad(result)


def screw_axis(w, q):
    v = -np.cross(w, q)
    return np.concatenate([w, v])


# define home configuration
M = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0.156],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

# S_list = np.array([
#     [0.0,  0.0,  1.0,  0.0,    0.0,    0.0,],
#     [-1.0,  0.0,  0.0,  0.0,    0.0,    0.0178,],
#     [-1.0,  0.0,  0.0,  0.0,    0.0,    0.079,],
#     [-1.0,  0.0,  0.0,  0.0,    0.0,    0.1195],
#     ])

# Joint axes (omega)
w_splay = np.array([0.0,  0.0,  1.0])
w_mcp = np.array([-1.0, 0.0,  0.0])
w_pip = np.array([-1.0, 0.0,  0.0])
w_dip = np.array([-1.0, 0.0,  0.0])

# Joint positions in space frame (accumulated from URDF)
q_splay = np.array([0.0,     0.0, 0.0])
q_mcp = np.array([0.0,     0.0178, 0.0])
q_pip = np.array([-0.0001, 0.079, -0.0017])
q_dip = np.array([0.0035, 0.1195, -0.004])

# v = -omega x q

S1 = screw_axis(w_splay, q_splay)
S2 = screw_axis(w_mcp,   q_mcp)
S3 = screw_axis(w_pip,   q_pip)
S4 = screw_axis(w_dip,   q_dip)

S_list = np.array([S1, S2, S3, S4])
print(S_list)

num_points = 25
splay_points = 25
splay_rom = np.linspace(-.55, .55, splay_points)
mcp_rom = np.linspace(0, np.pi/2, num_points)
# pip_rom = np.linspace(0, np.pi/2, num_points)
pip_rom = np.array([0.0, np.pi/2])
points = []
total = num_points * splay_points * 2
start = time.time()

for count, (splay, mcp, pip) in enumerate(product(splay_rom, mcp_rom, pip_rom)):
    angles = np.array([splay, mcp, pip, get_dip_angle(pip)])
    T = mr.FKinSpace(M, S_list.T, angles)
    points.append(T[:3, 3])

    if count % 10000 == 0 and count > 0:
        elapsed = time.time() - start
        percent_done = count / total
        time_remaining = (elapsed / percent_done) - elapsed
        print(f'{percent_done * 100:.2f}% done, {time_remaining:.1f}s remaining')

points = np.array(points)
print(f'Shape: {points.shape}')
print(f'Min:   {points.min(axis=0)}')
print(f'Max:   {points.max(axis=0)}')
print(f'Std:   {points.std(axis=0)}')
print(f'Sample points:\n{points[:5]}')

# alpha=0 is convex hull, higher alpha = tighter fit
points_mm = points * 1000.0
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_mm)  # load your Nx3 numpy array in
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=15, max_nn=50))

# Orient normals to point away from the centroid
pcd.orient_normals_towards_camera_location(pcd.get_center())

mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)

# More aggressive density trim
vertices_to_remove = densities < np.quantile(densities, 0.1)
mesh.remove_vertices_by_mask(vertices_to_remove)

# Clean up degenerate geometry
mesh.remove_degenerate_triangles()
mesh.remove_duplicated_triangles()
mesh.remove_duplicated_vertices()
mesh.remove_non_manifold_edges()

# Recompute and orient normals outward
mesh.compute_vertex_normals()
mesh.orient_triangles()

output_path = 'src/robotic-finger/finger_description/src/workspace.stl'
o3d.io.write_triangle_mesh(output_path, mesh)
# # Build trimesh mesh from hull (will fill in voids)
# mesh = trimesh.Trimesh(vertices=hull.points, faces=hull.simplices)
# trimesh.repair.fix_normals(mesh)
# mesh.export("/home/michael-jenz/rds_ws/src/robotic-finger/finger_description/src/workspace.stl")
