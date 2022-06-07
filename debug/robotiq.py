import argparse

import numpy as np
import sapien.core as sapien
from sapien.utils import Viewer


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c",
        "--model_constraint",
        action="store_true",
        default=False,
        help="Whether to model the joint constraint for the gripper.",
    )
    return parser.parse_args()


def add_gripper_constraint(robot, scene):
    # base = [l for l in robot.get_links() if l.name == "robotiq_arg2f_base_link"][0]
    # lp = [l for l in robot.get_links() if l.name == "left_inner_finger_pad"][0]
    # rp = [l for l in robot.get_links() if l.name == "right_inner_finger_pad"][0]

    # lif = [l for l in robot.get_links() if l.name == "left_inner_knuckle"][0]
    # lok = [l for l in robot.get_links() if l.name == "left_outer_knuckle"][0]
    # rif = [l for l in robot.get_links() if l.name == "right_inner_knuckle"][0]
    # rok = [l for l in robot.get_links() if l.name == "right_outer_knuckle"][0]

    # rd = scene.create_drive(base, sapien.Pose(), rp, sapien.Pose())
    # rd.lock_motion(0, 0, 0, 1, 1, 1)
    # ld = scene.create_drive(base, sapien.Pose(), lp, sapien.Pose(q=np.array([0, 0, 0, 1])))
    # ld.lock_motion(0, 0, 0, 1, 1, 1)
    # ld2 = scene.create_drive(lif, sapien.Pose(), lok, sapien.Pose())
    # ld2.lock_motion(0, 0, 0, 1, 1, 1)
    # rd2 = scene.create_drive(rif, sapien.Pose(), rok, sapien.Pose())
    # rd2.lock_motion(0, 0, 0, 1, 1, 1)

    outer_knuckle = next(
        j for j in robot.get_active_joints() if j.name == "right_outer_knuckle_joint"
    )
    outer_finger = next(
        j for j in robot.get_active_joints() if j.name == "right_inner_finger_joint"
    )
    inner_knuckle = next(
        j for j in robot.get_active_joints() if j.name == "right_inner_knuckle_joint"
    )

    pad = outer_finger.get_child_link()
    lif = inner_knuckle.get_child_link()

    T_pw = pad.pose.inv().to_transformation_matrix()
    p_w = (
        outer_finger.get_global_pose().p
        + inner_knuckle.get_global_pose().p
        - outer_knuckle.get_global_pose().p
    )
    T_fw = lif.pose.inv().to_transformation_matrix()
    p_f = T_fw[:3, :3] @ p_w + T_fw[:3, 3]
    p_p = T_pw[:3, :3] @ p_w + T_pw[:3, 3]

    right_drive = scene.create_drive(lif, sapien.Pose(p_f), pad, sapien.Pose(p_p))
    right_drive.lock_motion(1, 1, 1, 0, 0, 0)

    outer_knuckle = next(
        j for j in robot.get_active_joints() if j.name == "left_outer_knuckle_joint"
    )
    outer_finger = next(
        j for j in robot.get_active_joints() if j.name == "left_inner_finger_joint"
    )
    inner_knuckle = next(
        j for j in robot.get_active_joints() if j.name == "left_inner_knuckle_joint"
    )

    pad = outer_finger.get_child_link()
    lif = inner_knuckle.get_child_link()

    T_pw = pad.pose.inv().to_transformation_matrix()
    p_w = (
        outer_finger.get_global_pose().p
        + inner_knuckle.get_global_pose().p
        - outer_knuckle.get_global_pose().p
    )
    T_fw = lif.pose.inv().to_transformation_matrix()
    p_f = T_fw[:3, :3] @ p_w + T_fw[:3, 3]
    p_p = T_pw[:3, :3] @ p_w + T_pw[:3, 3]

    left_drive = scene.create_drive(lif, sapien.Pose(p_f), pad, sapien.Pose(p_p))
    left_drive.lock_motion(1, 1, 1, 0, 0, 0)


args = parse_args()
engine = sapien.Engine()
renderer = sapien.VulkanRenderer(offscreen_only=False)
engine.set_renderer(renderer)
scene = engine.create_scene()
scene.set_environment_map("../assets/env.ktx")
scene.add_directional_light([0.2, 0.2, -1], [1, 1, 1], True, scale=3, near=-5, far=5)
scene.add_ground(-0.9, render=True)

# Viewer
viewer = Viewer(renderer)
viewer.set_scene(scene)
viewer.set_camera_xyz(0, 0, 0.3)
viewer.set_camera_rpy(0, -1.57, 0)
viewer.toggle_axes(0)

# Articulation
urdf_file = "../assets/robotiq_2f_85_gripper_visualization/robotiq_85_original.urdf"
loader = scene.create_urdf_loader()
builder = loader.load_file_as_articulation_builder(urdf_file)

# Disable self collision for simplification
for link_builder in builder.get_link_builders():
    link_builder.set_collision_groups(1, 1, 2, 0)
robot = builder.build(fix_root_link=True)
robot.set_pose(sapien.Pose(p=np.array([0.0, 0, 0]), q=np.array([0.7071, 0, 0.7071, 0])))
robot.set_qpos(np.zeros([robot.dof]))

# MAdd constraints
if args.model_constraint:
    add_gripper_constraint(robot, scene)


right_joint = next(
    j for j in robot.get_active_joints() if j.name == "right_outer_knuckle_joint"
)
right_joint.set_drive_property(1e5, 2000, 0.1)
right_joint.set_drive_target(0.4)

left_joint = next(
    j for j in robot.get_active_joints() if j.name == "left_outer_knuckle_joint"
)
left_joint.set_drive_property(1e5, 2000, 0.1)
left_joint.set_drive_target(0.4)

viewer.toggle_pause(True)
while not viewer.closed:
    qpos = robot.get_qpos()
    qf = np.zeros_like(qpos)
    correction = min((qpos[0] - qpos[3]) * 10, 1)
    qf[3] += correction
    qf[0] -= correction
    robot.set_qf(qf)

    scene.update_render()
    scene.step()
    viewer.render()
