from common import setup_scene

engine, renderer, scene, viewer = setup_scene()
scene.set_timestep(1 / 120)

mat = engine.create_physical_material(1.0, 1.0, 0.0)

b = scene.create_actor_builder()
b.add_box_collision(half_size=[0.03] * 3, material=mat)
b.add_box_visual(half_size=[0.03] * 3, color=[1, 0, 0, 1])
b.build()

viewer.toggle_axes(False)
viewer.toggle_camera_lines(False)

viewer.set_camera_xyz(-0.11578903, 0.65288437, 0.37519158)
viewer.set_camera_rpy(0, -0.1635, 1.81318531)


while not viewer.closed:
    scene.step()
    viewer.render()
