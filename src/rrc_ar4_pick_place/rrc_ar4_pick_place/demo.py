import rclpy
from ar4_pick_place.pick_place import PickPlace

def main():
    rclpy.init()
    pp = PickPlace()
    safe = [0.0] * 6
    pp.pick(pick_joints=[0.0, 1.0647, 0.1047, 0.0, 0.4014, 0.0], safe_joints=safe)
    pp.place(place_joints=[0.5, 1.0647, 0.1047, 0.0, 0.4014, 0.0], safe_joints=safe)
    rclpy.shutdown()