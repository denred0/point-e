import time
import threading
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering


def main():
    gui.Application.instance.initialize()

    window = gui.Application.instance.create_window('img', width=1200, height=800)
    widget = gui.SceneWidget()
    widget.scene = rendering.Open3DScene(window.renderer)
    window.add_child(widget)

    # frame = o3d.geometry.TriangleMesh.create_coordinate_frame(1.5)
    name = 'pointcloud_1B'
    mesh = o3d.io.read_triangle_mesh(f"{name}.ply")
    mesh.compute_vertex_normals()

    mat = rendering.MaterialRecord()
    mat.shader = 'defaultLit'

    widget.scene.camera.look_at([0, 0, 0], [1, 1, 1], [0, 0, 1])
    # widget.scene.add_geometry('frame', frame, mat)
    widget.scene.add_geometry('mesh', mesh, mat)

    def update_geometry():
        widget.scene.clear_geometry()
        # widget.scene.add_geometry('frame', frame, mat)
        widget.scene.add_geometry('mesh', mesh, mat)

    def thread_main():
        while True:
            # Rotation
            R = mesh.get_rotation_matrix_from_xyz((0, 0, np.pi / 32))
            mesh.rotate(R, center=(0, 0, 0))

            # Update geometry
            gui.Application.instance.post_to_main_thread(window, update_geometry)

            time.sleep(0.1)

    threading.Thread(target=thread_main).start()

    gui.Application.instance.run()


if __name__ == "__main__":
    main()
