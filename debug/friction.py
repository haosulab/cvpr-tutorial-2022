from common import setup_scene
import sapien.core as sapien

timestep = 1 / 120
config = sapien.SceneConfig()

config.default_dynamic_friction = 0.3
config.default_static_friction = 0.3
config.default_restitution = 0.0
config.solver_iterations = 5

engine, renderer, scene, viewer = setup_scene(config)
scene.set_timestep(timestep)

robot: sapien.Articulation = scene.get_all_articulations()[0]

size = 0.03
z = size * 1.01

b = scene.create_actor_builder()
b.add_box_collision(half_size=[size] * 3, density=1e4)
b.add_box_visual(half_size=[size] * 3, color=[1, 0, 0, 1])
box1 = b.build()
box1.set_pose(sapien.Pose([0, 0, z]))
box1.set_name("box1")

viewer.set_camera_xyz(0.95667515, -0.7565938, 0.95767801)
viewer.set_camera_rpy(0, -0.67, -2.53)

hand = next(l for l in robot.get_links() if "hand" in l.name)

target_pose = sapien.Pose([0.55, 0, 0.13], [0, 1, 0, 0])
model = robot.create_pinocchio_model()
result, success, error = model.compute_inverse_kinematics(
    hand.get_index(), target_pose, robot.get_qpos(), [1] * 7 + [0, 0]
)

robot.set_drive_target(result)
for _ in range(30):
    force = robot.compute_passive_force(external=False)
    robot.set_qf(force)
    scene.step()
    viewer.render()

for j in robot.get_active_joints()[-2:]:
    j.set_drive_target(0)
for _ in range(30):
    force = robot.compute_passive_force(external=False)
    robot.set_qf(force)
    scene.step()
    viewer.render()


target_pose = sapien.Pose([0.55, 0, 0.2], [0, 1, 0, 0])
result, success, error = model.compute_inverse_kinematics(
    hand.get_index(), target_pose, robot.get_qpos(), [1] * 7 + [0, 0]
)
result[-2:] = 0

robot.set_drive_target(result)
for _ in range(30):
    force = robot.compute_passive_force(external=False)
    robot.set_qf(force)
    scene.step()
    viewer.render()


while not viewer.closed:
    force = robot.compute_passive_force(external=False)
    robot.set_qf(force)
    scene.step()
    viewer.render()
