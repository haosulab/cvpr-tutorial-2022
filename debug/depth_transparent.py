from common import setup_scene
import sapien.core as sapien

sapien.VulkanRenderer.set_viewer_shader_dir("./shader/ibl")

engine, renderer, scene, viewer = setup_scene()

b = scene.create_actor_builder()
m = renderer.create_material()
m.set_base_color([1, 1, 1, 0.7])
m.metallic = 1
m.roughness = 0.2
b.add_sphere_visual(radius=0.1, material=m)
sphere = b.build_kinematic()
sphere.set_pose(sapien.Pose([0.15, 0, 0.1]))


while True:
    scene.step()
    viewer.render()
