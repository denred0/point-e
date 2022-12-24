from PIL import Image
import torch
import trimesh
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from tqdm.auto import tqdm

from point_e.models.download import load_checkpoint
from point_e.models.configs import MODEL_CONFIGS, model_from_config
from point_e.util.pc_to_mesh import marching_cubes_mesh
from point_e.util.plotting import plot_point_cloud
from point_e.util.point_cloud import PointCloud

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

print('creating SDF model...')
name = 'sdf'
model = model_from_config(MODEL_CONFIGS[name], device)
model.eval()

print('loading SDF model...')
model.load_state_dict(load_checkpoint(name, device))

name = 'pointcloud_300M'

# Load a point cloud we want to convert into a mesh.
pc = PointCloud.load(f'{name}.npz')

# Plot the point cloud as a sanity check.
fig = plot_point_cloud(pc, grid_size=2)
# fig.show()

# Produce a mesh (with vertex colors)
mesh = marching_cubes_mesh(
    pc=pc,
    model=model,
    batch_size=4096,
    grid_size=128,  # increase to 128 for resolution used in evals
    progress=True,
)

# Write the mesh to a PLY file to import into some other program.
with open(f'{name}.ply', 'wb') as f:
    mesh.write_ply(f)

pcd_scene = o3d.geometry.PointCloud()
pcd_scene.points = o3d.utility.Vector3dVector(pc.coords)
pcd_scene.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
ball_radii = [0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.3]
result_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd_scene,
                                                                              o3d.utility.DoubleVector(ball_radii))
# o3d.visualization.draw_geometries([result_mesh])

mesh = trimesh.load(f'{name}.ply', force='mesh')
mesh.show()

