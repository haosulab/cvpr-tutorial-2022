from gym import Env
import gym
import numpy as np

import sapien.core as sapien


class OpenCabinetEnv(Env):

    action_space = gym.spaces.Box(np.ones(9) * -0.2, np.ones(9) * 0.2)

    def __init__(self, simulation_frequency=500, control_frequency=20):
        self.substeps = simulation_frequency // control_frequency

        self.engine = sapien.Engine()
        self.renderer = sapien.VulkanRenderer()
        self.engine.set_renderer(self.renderer)
        self.scene = self.engine.create_scene()
        self.scene.set_timestep(1 / simulation_frequency)

        self.scene.set_ambient_light(color=[0.3, 0.3, 0.3])
        self.scene.add_directional_light(
            direction=[-0.3, -0.3, -1], color=[1, 1, 1], shadow=True
        )

        self.scene.add_ground(altitude=0, render=True)
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.robot = loader.load("../assets/panda.urdf")
        self.robot.set_pose(sapien.Pose([-1, 0, 0]))

        loader.scale = 0.6
        self.cabinet = loader.load("../assets/45146/mobility.urdf")
        self.cabinet.set_pose(sapien.Pose([0, 0, 0.5]))

        self.camera = self.scene.add_camera(
            name="camera", width=128, height=128, fovy=np.pi / 2, near=0.01, far=2.0
        )
        self.camera.set_local_pose(
            sapien.Pose(
                [-1, 0.75, 0.75], [0.86988501, 0.05311156, 0.09607517, -0.48088336]
            )
        )

    def _get_observation(self):
        rgbd = self.render("rgbd")
        qpos = self.robot.get_qpos()
        qvel = self.robot.get_qvel()

        return np.concatenate([qpos, qvel]), rgbd

    def _get_reward(self):
        # sparse reward
        if self.cabinet.get_qpos()[1] > np.pi / 3:
            return 1
        else:
            return 0

        # dense reward
        # return self.cabinet.get_qpos()[1]

    def _get_done(self):
        return self.cabinet.get_qpos()[1] > np.pi / 3

        # it is okay to never have a done
        # return False

    def _get_info(self):
        return None

    def reset(self):
        # initial cabinet joint positions
        self.cabinet.set_qpos([0, 0])

        # intiail robot base pose, may be randomized
        self.robot.set_pose(sapien.Pose([-1, 0, 0]))

        # initial robot joint positions, may be randomized
        qpos = [0, 0.345, 0, -2.25, 0, 2.75, 0.78, 0.04, 0.04]
        self.robot.set_qpos(qpos)

        # damping may require further fine tuning later
        joints = self.robot.get_active_joints()
        for joint in joints[:-2]:  # arm joints
            joint.set_drive_property(stiffness=0, damping=300)

        for joint in joints[-2:]:  # finger joints
            joint.set_drive_property(stiffness=0, damping=10)

        self.robot.set_drive_velocity_target(np.zeros(len(joints)))

        return self._get_observation()

    def step(self, action):
        self.robot.set_drive_velocity_target(action)
        for substep in range(self.substeps):
            passive_force = self.robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            self.robot.set_qf(passive_force)
            self.scene.step()

        return (
            self._get_observation(),
            self._get_reward(),
            self._get_done(),
            self._get_info(),
        )

    def render(self, mode="human"):
        if mode == "human":
            if not hasattr(self, "viewer") or self.viewer is None:
                from sapien.utils import Viewer

                self.viewer = Viewer(self.renderer)
                self.viewer.set_scene(self.scene)
                self.viewer.set_camera_xyz(-2, 0, 0.5)

            self.viewer.render()

        if mode == "rgbd":
            self.scene.update_render()
            self.camera.take_picture()
            color = self.camera.get_float_texture("Color")[..., :3]

            # channel 2 is z depth, channel 3 is 01 depth
            depth = self.camera.get_float_texture("Position")[..., [3]]

            return np.concatenate([color, depth], 2)


def main():
    env = OpenCabinetEnv()
    env.reset()

    while True:
        random_action = env.action_space.sample()
        obs, reward, done, info = env.step(random_action)
        env.render()
        if env.viewer.closed:
            break


if __name__ == "__main__":
    main()
