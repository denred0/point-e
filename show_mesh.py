import trimesh

mesh = trimesh.load('mesh.ply', force='mesh')
mesh.show()