from common import setup_scene
import sapien.core as sapien

sapien.VulkanRenderer.set_viewer_shader_dir("./shader/ibl")

engine, renderer, scene, viewer = setup_scene()

b = scene.create_actor_builder()
m = renderer.create_material()
m.set_base_color([1, 1, 1, 0.7])
m.metallic = 1
m.roughness = 0.2
b.add_sphere_visual(radius=0.1, material=m)
sphere = b.build_kinematic()
sphere.set_pose(sapien.Pose([0.15, 0, 0.1]))

viewer.toggle_axes(False)
viewer.toggle_camera_lines(False)

# code to insepct the camera point cloud

# cam = scene.add_camera("", 128, 128, 1, 0.1, 10)
# cam.set_local_pose(sapien.Pose([0.74096619, 0.92117075, 0.45152833], [-0.3826834, 0, 0, 0.9238795]))
# scene.step()
# scene.update_render()
# cam.take_picture()

# pos_depth = cam.get_position_rgba()
# mask = pos_depth[..., -1] < 1

# rgb = cam.get_color_rgba()[..., :3][mask]
# pos = pos_depth[..., :3][mask]

# import trimesh
# points = trimesh.points.PointCloud(pos, rgb)
# points.show()


while True:
    scene.step()
    viewer.render()
    print(viewer.fps_camera_controller.xyz)
