#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Pose
from rrc_ar4_pick_place.pick_place import PickPlace

# blue_cylinder in pick_place.world (feature/pick-and-place-world): center at (0.0, -0.38, 0.025),
# 2 cm diameter x 5 cm tall. The robot base sits at the world origin, so world == planning frame.
CYLINDER_XY = (0.0, -0.38)
# Grasp the upper part of the cylinder (center is 0.025, top is 0.05)
GRASP_Z = 0.035
# ee_link is the link_6 flange; the jaw pads are ~5 cm further out along its +z
EE_TO_GRASP = 0.05


def blue_cylinder_grasp_pose() -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y = CYLINDER_XY
    pose.position.z = GRASP_Z + EE_TO_GRASP
    # Top-down grasp: rotate 180 deg about x so ee_link +z points down at the table
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    return pose


def main():
    rclpy.init()
    safe = [0.0] * 6
    pp = PickPlace(safe_joints=safe)
    pp.pick(pick_joints=[0.0, 1.0647, 0.1047, 0.0, 0.4014, 0.0])
    pp.place(place_joints=[0.5, 1.0647, 0.1047, 0.0, 0.4014, 0.0])

    pp.pick_pose(blue_cylinder_grasp_pose())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
