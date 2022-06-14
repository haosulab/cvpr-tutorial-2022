from common import setup_scene
from sapien.core import Pose, Articulation

engine, renderer, scene, viewer = setup_scene()
scene.set_timestep(1 / 120)

robot: Articulation = scene.get_all_articulations()[0]

joint_velocity_target = []

    for joint in robot.get_active_joints():
        # stiffness: diagonal of K_p
        # damping: diagonal of K_v
        joint.set_drive_property(stiffness=0, damping=10.0)

    robot.set_drive_velocity_target(joint_velocity_target)  # set PD control velocity
    passive_force = robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True)  # ID(0;q,q̇)
    robot.set_qf(passive_force)  # augment PD control with ID
