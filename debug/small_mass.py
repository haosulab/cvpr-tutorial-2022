from common import setup_scene
from sapien.utils import Viewer
import sapien.core as sapien


def setup_scene(config=None):
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    if config is None:
        config = sapien.SceneConfig()
    scene = engine.create_scene(config)
    scene.set_environment_map("../assets/env.ktx")

    b = scene.create_actor_builder()
    b.add_visual_from_file("../assets/scene.glb")
    b.add_box_collision(sapien.Pose([0, 0, -0.375]), half_size=[0.375, 0.75, 0.375])
    b.build_static()
    scene.add_ground(-0.9, render=False)

    loader = scene.create_urdf_loader()
    robot = loader.load("../assets/panda.urdf", {"density": 1})
    robot.set_pose(sapien.Pose([-0.55, 0, 0]))
    for j in robot.get_active_joints():
        j.set_drive_property(1000, 100)
    for j in robot.get_active_joints()[-2:]:
        j.set_drive_property(1000, 5, 10)

    for l in robot.get_links():
        print(l.mass, l.inertia)


    qpos = [0, 0.345, 0, -2.25, 0, 2.75, 0.78, 0.04, 0.04]
    robot.set_qpos(qpos)
    robot.set_drive_target(qpos)

    viewer = Viewer(renderer, resolutions=((1024, 1024),))
    viewer.set_scene(scene)

    scene.add_directional_light(
        [0.2, 0.2, -1], [1, 1, 1], True, scale=3, near=-5, far=5
    )

    return engine, renderer, scene, viewer


timestep = 1 / 120
config = sapien.SceneConfig()

engine, renderer, scene, viewer = setup_scene(config)

scene.set_timestep(timestep)

robot: sapien.Articulation = scene.get_all_articulations()[0]

size = 0.03
z = size * 1.01

b = scene.create_actor_builder()
b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[1, 0, 0, 1])
box1 = b.build()
box1.set_pose(sapien.Pose([0, 0, z]))
box1.set_name("box1")

viewer.set_camera_xyz(0.95667515, -0.7565938, 0.95767801)
viewer.set_camera_rpy(0, -0.67, -2.53)

for j in robot.get_active_joints()[-2:]:
    j.set_drive_target(0.04)


while not viewer.closed:
    force = robot.compute_passive_force(external=False)
    robot.set_qf(force)
    scene.step()
    viewer.render()
