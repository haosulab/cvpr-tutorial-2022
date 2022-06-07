import argparse
from typing import List

import numpy as np
import sapien.core as sapien
from sapien.utils import Viewer


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-u', '--urdf', action='store', type=str, required=True, help="Path to the URDF file.")
    parser.add_argument('-s', '--simulate', action='store_true', default=True,
                        help="Whether to physically simulate the robot and check collision.")
    return parser.parse_args()


def visualize_articulation(urdf_file, simulate):
    # Setup
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer(offscreen_only=False)
    engine.set_renderer(renderer)
    config = sapien.SceneConfig()
    config.gravity = np.array([0, 0, 0])
    scene = engine.create_scene(config=config)
    scene.set_timestep(1 / 125)

    # Ground
    visual_material = renderer.create_material()
    visual_material.set_base_color(np.array([0.5, 0.5, 0.5, 1]))
    visual_material.set_roughness(0.7)
    visual_material.set_metallic(1)
    visual_material.set_specular(0.04)
    scene.add_ground(-1, render_material=visual_material)

    # Lighting
    scene.set_ambient_light(np.array([0.3, 0.3, 0.3]))
    scene.add_directional_light(np.array([-1, -1, -1]), np.array([0.5, 0.5, 0.5]), shadow=True)
    scene.add_directional_light(np.array([0, 0, -1]), np.array([0.9, 0.8, 0.8]), shadow=False)
    scene.add_spot_light(np.array([0, 0, 1.5]), direction=np.array([0, 0, -1]), inner_fov=0.3, outer_fov=1.0,
                         color=np.array([0.5, 0.5, 0.5]), shadow=False)

    # Viewer
    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(1, 0, 1)
    viewer.set_camera_rpy(0, -0.6, 3.14)
    viewer.toggle_axes(0)

    # Articulation
    loader = scene.create_urdf_loader()
    robot = loader.load(urdf_file)
    robot.set_qpos(np.zeros([robot.dof]))
    scene.step()

    painted_actor_set = set()
    while not viewer.closed:
        scene.update_render()
        viewer.render()
        if simulate:
            while len(painted_actor_set) > 0:
                actor = painted_actor_set.pop()
                unpaint_actor([actor])
            scene.step()
            for contact in scene.get_contacts():
                has_contact_force = False
                for point in contact.points:
                    if np.linalg.norm(point.impulse) > 1e-3:
                        has_contact_force = True
                if has_contact_force:
                    actor0 = contact.actor0
                    actor1 = contact.actor1
                    paint_actor([actor0, actor1])
                    painted_actor_set.add(actor0)
                    painted_actor_set.add(actor1)
                    print(f"Link {actor0.get_name()} and {actor1.get_name()} collide!")


def paint_actor(actors: List[sapien.ActorBase], color=(1, 0, 0, 1)):
    for actor in actors:
        actor.render_collision(True)
        for geom in actor.get_collision_visual_bodies():
            for shape in geom.get_render_shapes():
                mat = shape.material
                mat.set_base_color(color)
                shape.set_material(mat)


def unpaint_actor(actors: List[sapien.ActorBase]):
    for actor in actors:
        actor.render_collision(False)
        for geom in actor.get_collision_visual_bodies():
            for shape in geom.get_render_shapes():
                mat = shape.material
                mat.set_base_color(np.array([0, 1, 0, 1]))
                shape.set_material(mat)


if __name__ == '__main__':
    args = parse_args()
    visualize_articulation(args.urdf, args.simulate)
