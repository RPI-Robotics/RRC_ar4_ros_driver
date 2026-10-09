#!/usr/bin/env python3
"""Export a self-contained AR4 URDF + meshes folder for Isaac Sim's URDF importer.

Isaac Sim runs on Windows and cannot resolve package paths inside the
container, so mesh paths are rewritten to be relative to the URDF file.
"""
import argparse
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ar-model",
                        default="mk5",
                        choices=["mk1", "mk2", "mk3", "mk4", "mk5"])
    parser.add_argument("--output",
                        default="isaac_export",
                        help="Output directory (default: ./isaac_export)")
    args = parser.parse_args()

    description_share = get_package_share_directory("annin_ar4_description")
    urdf = subprocess.run(
        [
            "xacro",
            os.path.join(description_share, "urdf", "ar.urdf.xacro"),
            f"ar_model:={args.ar_model}",
            "include_gripper:=true",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    robot = ET.fromstring(urdf)

    meshes_dir = os.path.join(description_share, "meshes")
    mesh_prefix = "file://" + meshes_dir + "/"
    copied = set()
    for link in robot.iter("link"):
        for element in link.findall("visual") + link.findall("collision"):
            mesh = element.find("geometry/mesh")
            if mesh is None or not mesh.get("filename", "").startswith(mesh_prefix):
                continue
            relative = mesh.get("filename")[len(mesh_prefix):]
            source = os.path.join(meshes_dir, relative)
            if not os.path.exists(source):
                print(f"Warning: dropping {element.tag} of {link.get('name')}, "
                      f"{source} is missing")
                link.remove(element)
                continue
            mesh.set("filename", "meshes/" + relative)
            if relative not in copied:
                destination = os.path.join(args.output, "meshes", relative)
                os.makedirs(os.path.dirname(destination), exist_ok=True)
                shutil.copyfile(source, destination)
                copied.add(relative)

    os.makedirs(args.output, exist_ok=True)
    urdf_path = os.path.join(args.output, f"{args.ar_model}.urdf")
    ET.ElementTree(robot).write(urdf_path, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {urdf_path} with {len(copied)} meshes")


if __name__ == "__main__":
    main()
