from common import setup_scene

engine, renderer, scene, viewer = setup_scene()

while True:
    scene.step()
    viewer.render()
