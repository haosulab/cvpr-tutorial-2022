from common import setup_scene
import sapien.core as sapien

timestep = 1 / 120
engine, renderer, scene, viewer = setup_scene(robot=False)
scene.set_timestep(timestep)

loader = scene.create_urdf_loader()
loader.fix_root_link = True
loader.scale = 0.3
oven = loader.load("../assets/101773/mobility.urdf")
# loader.load_multiple_collisions_from_file = True
# oven = loader.load("../assets/101773/mobility_cvx.urdf")
oven.set_pose(sapien.Pose([0, 0, 0.2], [0, 0, 0, 1]))  # This is not unit quaternion!
qpos = oven.get_qpos()
qpos[0] = 1.5
oven.set_qpos(qpos)

b = scene.create_actor_builder()
b.add_box_collision(half_size=[0.02] * 3, density=1e4)
b.add_box_visual(half_size=[0.02] * 3, color=[1, 0, 0, 1])
box1 = b.build()
box1.set_pose(sapien.Pose([0.06, 0, 0.164]))
box1.set_name("box")

viewer.set_camera_xyz(0.73653925, 0.04725184, 0.38002751)
viewer.set_camera_rpy(0.0, -0.21, 3.12)
viewer.toggle_pause(True)

while not viewer.closed:
    scene.step()
    viewer.render()
