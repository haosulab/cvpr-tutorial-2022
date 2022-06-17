import json
import os
from pathlib import Path

import pybullet as p
from bs4 import BeautifulSoup


def do_vhacd(input_path, output_path, log_path=None, **kwargs):
    p.connect(p.DIRECT)
    if log_path is None:
        root, _ = os.path.splitext(output_path)
        log_path = root + ".log.txt"
        print(log_path)
    p.vhacd(input_path, output_path, log_path, **kwargs)
    p.disconnect()


def load_json(json_path):
    with open(json_path, "r") as f:
        return json.load(f)


def load_model_ids(json_path):
    infos = load_json(json_path)
    model_ids = [x["id"] for x in infos]
    return model_ids


def load_semantics(txt_path):
    with open(txt_path, "r") as f:
        annos = [x.strip().split() for x in f.readlines()]
    return annos


# def main():
#     model_dir = Path("../assets/101773")

#     vhacd_dir = model_dir / "vhacd_objs"
#     vhacd_dir.mkdir(exist_ok=True)

#     urdf_path = model_dir / "mobility.urdf"
#     new_urdf_path = model_dir / "mobility_vhacd.urdf"

#     with open(urdf_path, "r") as f:
#         urdf = BeautifulSoup(f, features="xml")

#     links = list(urdf.find_all("link"))
#     for link in links:
#         collisions = list(link.find_all("collision"))
#         # print(link.attrs["name"], collisions)
#         for collision in collisions:
#             mesh = collision.find("mesh")
#             if mesh is None:
#                 continue

#             col_filename = mesh.attrs["filename"]
#             basename: str = os.path.basename(col_filename)
#             vhacd_path = vhacd_dir / basename
#             if not vhacd_path.exists():
#                 do_vhacd(str(model_dir / col_filename), str(vhacd_path))
#             mesh.attrs["filename"] = col_filename.replace(
#                 "textured_objs", "vhacd_objs"
#             )

#     with open(new_urdf_path, "w") as f:
#         f.write(urdf.prettify())




import json
import os
import shutil
import subprocess
import tempfile
from collections import OrderedDict
from pathlib import Path

import numpy as np
import trimesh
from bs4 import BeautifulSoup
from transforms3d.euler import euler2mat, mat2euler


do_cvx = do_vhacd

# def do_cvx(input_path, output_path, threshold=0.05, seed=0, timeout=600):
#     # specify your own path
#     with tempfile.TemporaryDirectory() as temp_dir:
#         basename = os.path.basename(output_path)
#         root, ext = os.path.splitext(basename)
#         log_path = os.path.join(temp_dir, root + ".log.txt")
#         command = (
#             "{} -i {} -o {} -of {} -l {} -t {threshold} --seed {seed} -pr 8".format(
#                 str(BINARY_FILE),
#                 input_path,
#                 root,
#                 temp_dir,
#                 log_path,
#                 threshold=threshold,
#                 seed=seed,
#             )
#         )
#         try:
#             result = subprocess.run(command, shell=True, timeout=timeout, check=True)
#         except subprocess.TimeoutExpired as err:
#             print(str(err))
#             return None
#         except subprocess.CalledProcessError as err:
#             print(str(err))
#             return None
#         else:
#             shutil.copy2(os.path.join(temp_dir, root + "_convex.obj"), output_path)
#             shutil.copy2(log_path, os.path.dirname(output_path))
#             with open(log_path) as f:
#                 lines = list(filter(None, [x.strip() for x in f.readlines()]))
#             return int(lines[-1].split("#Convex After Merge: ")[-1])


def load_json(json_path):
    with open(json_path, "r") as f:
        return json.load(f)


def load_model_ids(json_path):
    infos = load_json(json_path)
    model_ids = [x["id"] for x in infos]
    return model_ids


def load_meshes(filename):
    """Loads triangular meshes from a file.
    Parameters
    ----------
    filename : str
        Path to the mesh file.
    Returns
    -------
    meshes : list of :class:`~trimesh.base.Trimesh`
        The meshes loaded from the file.
    """
    meshes = trimesh.load(filename)

    # If we got a scene, dump the meshes
    if isinstance(meshes, trimesh.Scene):
        meshes = list(meshes.dump())
        meshes = [g for g in meshes if isinstance(g, trimesh.Trimesh)]

    if isinstance(meshes, (list, tuple, set)):
        meshes = list(meshes)
        if len(meshes) == 0:
            raise ValueError("At least one mesh must be pmeshesent in file")
        for r in meshes:
            if not isinstance(r, trimesh.Trimesh):
                raise TypeError("Could not load meshes from file")
    elif isinstance(meshes, trimesh.Trimesh):
        meshes = [meshes]
    else:
        raise ValueError("Unable to load mesh from file")

    return meshes


class URDFParser:
    def __init__(self, urdf_path) -> None:
        with open(urdf_path, "r") as f:
            self.urdf = BeautifulSoup(f, features="xml")
        self.root_dir = Path(urdf_path).parent

    def get_links(self):
        return list(self.urdf.find_all("link"))

    @staticmethod
    def get_collisions(link, geom_type=None):
        collisions = list(link.find_all("collision"))
        if geom_type is not None:
            collisions = [col for col in collisions if col.find(geom_type) is not None]
        return collisions

    @staticmethod
    def parse_origin(tag):
        T = np.eye(4)
        origin = tag.find("origin")
        if origin is not None:
            if "xyz" in origin.attrs:
                T[:3, 3] = np.fromstring(origin.attrs["xyz"], sep=" ")
            if "rpy" in origin.attrs:
                rpy = np.fromstring(origin.attrs["rpy"], sep=" ")
                T[:3, :3] = euler2mat(rpy, axes="sxyz")
        return T

    def load_mesh(self, mesh_tag, is_collision=True):
        filename = self.root_dir / mesh_tag.attrs["filename"]
        meshes = load_meshes(str(filename))

        scale = mesh_tag.attrs.get("scale")
        if scale is not None:
            scale = np.fromstring(scale, sep=" ")
            print("scale:", scale)
            for m in meshes:
                m.apply_scale(scale)

        if is_collision:
            # Delete visuals for simplicity
            for m in meshes:
                m.visual = trimesh.visual.ColorVisuals(mesh=m)

        return meshes

    @staticmethod
    def create_origin(T):
        R, t = T[:3, :3], T[:3, 3]
        xyz = "{} {} {}".format(*t)
        rpy = "{} {} {}".format(*mat2euler(R))
        origin = BeautifulSoup("", features="xml").new_tag("origin", xyz=xyz, rpy=rpy)
        return origin

    @staticmethod
    def create_collision(T, filename):
        soup = BeautifulSoup("", features="xml")
        collision = soup.new_tag("collision")
        if T is not None:
            origin = URDFParser.create_origin(T)
            collision.append(origin)
        geom = soup.new_tag("geometry")
        geom.append(soup.new_tag("mesh", filename=filename))
        collision.append(geom)
        return collision


def main():
    # root_dir = ASSET_DIR / "partnet_mobility/dataset"
    # model_ids = load_model_ids(THIS_DIR / "meta/faucet.json")
    # print(len(model_ids))

    # error_model_ids = []

    # for model_id in model_ids:
        # if model_id != 152:  # uncomment for specific sample
            # continue
        # if model_id in [1528, 152]:
        #     continue

    model_dir = Path("../assets/101773")
    flag = False

    cvx_dir = model_dir / "cvx_objs"
    cvx_dir.mkdir(exist_ok=True)

    urdf_path = model_dir / "mobility.urdf"
    new_urdf_path = model_dir / "mobility_cvx.urdf"

    urdf_parser = URDFParser(urdf_path)
    for link in urdf_parser.get_links():
        link_name = link.attrs["name"]
        collisions = urdf_parser.get_collisions(link, geom_type="mesh")
        if len(collisions) == 0:
            continue

        link_meshes = []
        for collision in collisions:
            meshes = urdf_parser.load_mesh(collision.find("mesh"))
            T = urdf_parser.parse_origin(collision)
            for mesh in meshes:
                mesh.apply_transform(T)
            link_meshes.extend(meshes)

            # Remove the original collision
            collision.extract()
        # print(len(link_meshes))

        output_path = cvx_dir / (link_name + ".obj")
        # print(output_path)
        # A trick to merge meshes
        link_mesh = link_meshes[0] + link_meshes[1:]
        # link_mesh.show()
        trimesh.exchange.export.export_mesh(link_mesh, output_path)
        print(output_path)

        # convex decomposition
        n_comp = do_cvx(str(output_path), str(output_path))
        # if n_comp is None:
        #     flag = True
        #     break
        # if n_comp > 20:
        #     do_cvx(str(output_path), str(output_path))

        # Add merged collision
        new_collision = urdf_parser.create_collision(
            None, str(output_path.relative_to(model_dir))
        )
        # print(new_collision)
        link.append(new_collision)

    if flag:
        print("Clean up", cvx_dir)
        # clean-up
        shutil.rmtree(cvx_dir)
        new_urdf_path.unlink(True)
    else:
        with open(new_urdf_path, "w") as f:
            f.write(urdf_parser.urdf.prettify())



if __name__ == "__main__":
    main()
