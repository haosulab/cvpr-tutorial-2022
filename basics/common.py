import sapien.core as sapien
from sapien.utils import Viewer

sapien.VulkanRenderer.set_viewer_shader_dir("shader/ibl")

def setup_scene(config=None, robot=True):
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    if config is None:
        config = sapien.SceneConfig()
    scene = engine.create_scene(config)
    scene.set_environment_map("../assets/env.ktx")

    b = scene.create_actor_builder()
    b.add_visual_from_file("../assets/scene2.glb")
    b.add_box_collision(sapien.Pose([0, 0, -0.375]), half_size=[0.375, 0.75, 0.375])
    b.build_static()
    scene.add_ground(-0.9, render=False)

    if robot:
        loader = scene.create_urdf_loader()
        robot = loader.load("../assets/panda.urdf")
        robot.set_pose(sapien.Pose([-0.55, 0, 0]))
        for j in robot.get_active_joints():
            j.set_drive_property(1000, 100)
        for j in robot.get_active_joints()[-2:]:
            j.set_drive_property(1000, 5, 10)

        qpos = [0, 0.345, 0, -2.25, 0, 2.75, 0.78, 0.0, 0.0]
        robot.set_qpos(qpos)
        robot.set_drive_target(qpos)

    viewer = Viewer(renderer, resolutions=((1024, 1024),))
    viewer.set_scene(scene)

    scene.add_directional_light(
        [0.2, 0.2, -1], [1, 1, 1], True, scale=3, near=-5, far=5
    )
    scene.add_point_light(
        [-0.2, 0.5, 1], [1, 1, 1],
    )
    scene.add_point_light(
        [-0.2, -0.5, 1], [1, 1, 1],
    )

    return engine, renderer, scene, viewer
