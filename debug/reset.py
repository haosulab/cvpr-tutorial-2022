from common import setup_scene
import sapien.core as sapien

timestep = 1 / 120
config = sapien.SceneConfig()
config.solver_iterations = 5
print(config.solver_iterations)
engine, renderer, scene, viewer = setup_scene()
scene.set_timestep(timestep)


size = 0.05
z = size * 1.01

boxes = []

b = scene.create_actor_builder()
b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[1, 0, 0, 1])
box = b.build()
box.set_pose(sapien.Pose([0.15, 0, z]))
boxes.append(box)

b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[0, 1, 0, 1])
box = b.build()
box.set_pose(sapien.Pose([0.15, 0, z * 3]))
boxes.append(box)

b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[0, 0, 1, 1])
box = b.build()
box.set_pose(sapien.Pose([0.15, 0, z * 5]))
boxes.append(box)

b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[1, 1, 0, 1])
box = b.build()
box.set_pose(sapien.Pose([0.15, 0, z * 7]))
boxes.append(box)

b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[0, 1, 1, 1])
box = b.build()
box.set_pose(sapien.Pose([0.15, 0, z * 9]))
boxes.append(box)

b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[1, 0, 1, 1])
box = b.build()
box.set_pose(sapien.Pose([0.15, 0, z * 11]))
boxes.append(box)

b.add_box_collision(half_size=[size] * 3)
b.add_box_visual(half_size=[size] * 3, color=[1, 1, 1, 1])
box = b.build()
box.set_pose(sapien.Pose([0.15, -0.3, z * 11]))
box.set_velocity([0, 2, 0])
boxes.append(box)

poses = [box.get_pose() for box in boxes]
vels = [(box.velocity, box.angular_velocity) for box in boxes]


def reset1():
    for pose, box in zip(poses, boxes):
        box.set_pose(pose)


def reset2():
    for pose, (v, w), box in zip(poses, vels, boxes):
        box.set_pose(pose)
        box.set_velocity(v)
        box.set_angular_velocity(w)


viewer.set_camera_xyz(0.95667515, -0.7565938, 0.95767801)
viewer.set_camera_rpy(0, -0.67, -2.53)

count = 0
while True:

    for _ in range(2):
        scene.step()
    viewer.render()
    count += 1

    if count % 30 == 0:
        viewer.toggle_pause(True)

    if count % 31 == 0:
        reset1()
