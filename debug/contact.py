from common import setup_scene
import sapien.core as sapien

timestep = 1 / 120
config = sapien.SceneConfig()
config.solver_iterations = 5
print(config.solver_iterations)
engine, renderer, scene, viewer = setup_scene(config)
scene.set_timestep(timestep)


size = 0.05
z = size * 1.01

b = scene.create_actor_builder()
b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[1, 0, 0, 1])
box1 = b.build()
box1.set_pose(sapien.Pose([0.15, 0, z]))
box1.set_name("box1")

b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[0, 0, 1, 1])
box2 = b.build()
box2.set_pose(sapien.Pose([0.15, 0.105, z]))
box2.set_name("box2")

viewer.set_camera_xyz(0.95667515, -0.7565938, 0.95767801)
viewer.set_camera_rpy(0, -0.67, -2.53)
viewer.toggle_pause(True)


while True:
    scene.step()
    viewer.render()
