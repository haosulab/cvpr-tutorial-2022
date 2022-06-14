from common import setup_scene
from sapien.core import Pose

engine, renderer, scene, viewer = setup_scene()
scene.set_timestep(1 / 120)

robot = scene.get_all_articulations()[0]

link_idx = 10
target_pose = Pose()
joint_mask=[]

def x():

    robot_model = robot.create_pinocchio_model()
    joint_positions, success, error = robot_model.compute_inverse_kinematics(
        link_idx,
        target_pose,
        active_qmask=joint_mask,  # joints with mask value 1 are allowed to move
        max_iterations=100,
       )
