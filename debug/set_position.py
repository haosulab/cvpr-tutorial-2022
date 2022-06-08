from common import setup_scene
import sapien.core as sapien

engine, renderer, scene, viewer = setup_scene()

robot = scene.get_all_articulations()[0]
qpos = [
    0,
    0.14642349,
    -0.00051129825,
    -1.9572794,
    -0.0013204297,
    2.1474252,
    0.7797619,
    0.039998677,
    0.04,
]
robot.set_qpos(qpos)
robot.set_drive_target(qpos)

loader = scene.create_urdf_loader()
loader.fix_root_link = True
loader.scale = 0.3
oven = loader.load("../assets/101773/mobility.urdf")
oven.set_pose(sapien.Pose([0.3, 0, 0.2]))  # This is not unit quaternion!

qpos = oven.get_qpos()
qpos[0] = 0.4
oven.set_qpos(qpos)

oven.get_active_joints()[0].set_drive_property(50, 10)
oven.get_active_joints()[0].set_drive_target(0.4)

viewer.set_camera_xyz(0.21, 0.96, 0.32)
viewer.set_camera_rpy(0, 0, 2.2)
viewer.toggle_pause(True)

goal = [
    0,
    0.6265889,
    -0.00045825623,
    -1.756508,
    -0.0011921047,
    1.8158535,
    0.77940315,
    0.039998643,
    0.04,
]

count = 0

robot.set_drive_target(goal)

while not viewer.closed:
    robot.set_qf(robot.compute_passive_force(external=False))

    scene.step()
    viewer.render()
    count += 1
