import sapien.core as sapien
from sapien.utils import Viewer


engine = sapien.Engine()
renderer = sapien.VulkanRenderer()
engine.set_renderer(renderer)

scene = engine.create_scene()
scene.set_environment_map("../assets/env.ktx")

b = scene.create_actor_builder()
b.add_visual_from_file("../assets/scene.glb")
b.add_box_collision(sapien.Pose([0, 0, -0.375]), half_size=[0.375, 0.75, 0.375])
b.build_static()
scene.add_ground(-0.9, render=False)

viewer = Viewer(renderer, resolutions=((1024, 1024),))
viewer.set_scene(scene)

# scene.add_directional_light([0.3, 0.3, -1], [1, 1, 1], True, scale=3, near=-5, far=5)
# scene.add_point_light([0, 0, 0.3], [0.2, 0.2, 0.2], True, shadow_map_size=8192)
# scene.add_spot_light([0, 0, 0.3], [0, 0, -1], 1.0, 1.5, [0.2, 0.2, 0.2], True, shadow_map_size=8192)

b = scene.create_actor_builder()
b.add_box_collision(half_size=[0.03] * 3)
b.add_box_visual(half_size=[0.03] * 3, color=[1, 0, 0, 1])
b.build().set_pose(sapien.Pose([0, 0, 0.03]))

viewer.set_camera_xyz(0.404, -0.194, 0.205)
viewer.set_camera_rpy(0, -0.25, -2.5)
viewer.toggle_axes(False)

while not viewer.closed:
    scene.update_render()
    viewer.render()
