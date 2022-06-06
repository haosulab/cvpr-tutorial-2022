import sapien.core as sapien
from sapien.utils import Viewer


def setup_scene():
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_environment_map("../assets/env.ktx")

    b = scene.create_actor_builder()
    b.add_visual_from_file("../assets/scene.glb")
    b.build_static()

    loader = scene.create_urdf_loader()
    robot = loader.load("../assets/panda.urdf")
    robot.set_pose(sapien.Pose([-0.55, 0, 0]))
    for j in robot.get_active_joints():
        j.set_drive_property(1000, 10)
    qpos = [0, 0.345, 0, -2.25, 0, 2.75, 0.78, 0.04, 0.04]
    robot.set_qpos(qpos)
    robot.set_drive_target(qpos)

    viewer = Viewer(renderer, resolutions=((1024, 1024),))
    viewer.set_scene(scene)

    scene.add_directional_light(
        [0.2, 0.2, -1], [1, 1, 1], True, scale=3, near=-5, far=5
    )

    return engine, renderer, scene, viewer
