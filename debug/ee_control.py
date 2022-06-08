from common import setup_scene
import sapien.core as sapien
import numpy as np
import argparse

from imageio import mimsave


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--damping",
        type=float,
        help="Damping factor in the IK solver"
    )
    return parser.parse_args()


def compute_inverse_kinematics(world_velocity, jacobian, damping):
    lmbda = np.eye(6) * (damping ** 2)
    joint_velocity = jacobian.T @ np.linalg.lstsq(jacobian.dot(jacobian.T) + lmbda, world_velocity, rcond=None)[0]

    return joint_velocity


BALANCE_PASSIVE_FORCE = True
engine, renderer, scene, viewer = setup_scene()
viewer.set_camera_xyz(0.3, 0.6, 0.8)
viewer.set_camera_rpy(0, -0.7, 2.2)
args = parse_args()
damping = args.damping
action_repeat = 5
robot: sapien.Articulation = scene.get_all_articulations()[0]

cartesian_velocity = np.array([1, 0, 0, 0, 0, 0]) * 0.5
ee_link = next(link for link in robot.get_links() if link.get_name() == "panda_hand")
ee_link_order = robot.get_links().index(ee_link) - 1  # Do not consider root link
robot_arm_drive = robot.get_drive_target()

step = 0

img_list = []
viewer.toggle_axes(False)
while step < 600:
    if step % action_repeat == 0:
        # Jacobian of ee link with respect to robot arm joint position
        ee_jacobian = robot.compute_world_cartesian_jacobian()[ee_link_order * 6: ee_link_order * 6 + 6, :7]
        qvel = compute_inverse_kinematics(cartesian_velocity, ee_jacobian, damping=damping)
        robot_arm_drive[:7] += qvel * scene.get_timestep() * action_repeat
        robot.set_drive_target(robot_arm_drive)

    scene.step()
    viewer.render()
    img = (viewer.window.get_float_texture("Color") * 255).astype(np.uint8)
    img_list.append(img)
    step += 1

mimsave("large_damping.gif", img_list[::5])
