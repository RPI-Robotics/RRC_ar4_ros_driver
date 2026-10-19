from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Pose

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class PickPlace:
    def __init__(self, safe_joints: list[float], robot: MoveItPy | None = None, node_name="ar4_pick_place"):
        self.robot = robot or MoveItPy(node_name=node_name)
        self.arm = self.robot.get_planning_component("ar_manipulator")
        self.gripper = self.robot.get_planning_component("ar_gripper")
        self.model = self.robot.get_robot_model()
        self.safe_joints = safe_joints

    def _plan_and_execute(self, component, label):
        component.set_start_state_to_current_state()
        result = component.plan()
        if not result:
            raise RuntimeError(f"Planning failed: {label}")
        self.robot.execute(result.trajectory, controllers=[])

    def open_gripper(self):
        self.gripper.set_goal_state(configuration_name="open")
        self._plan_and_execute(self.gripper, "gripper open")

    def close_gripper(self):
        self.gripper.set_goal_state(configuration_name="closed")
        self._plan_and_execute(self.gripper, "gripper closed")

    def move_joints(self, values, label="arm"):
        state = RobotState(self.model)
        state.joint_positions = dict(zip(ARM_JOINTS, values))
        self.arm.set_goal_state(robot_state=state)
        self._plan_and_execute(self.arm, label)

    def pick(self, pick_joints):
        self.open_gripper()
        self.move_joints(self.safe_joints, "safe")
        self.move_joints(pick_joints, "pick")
        self.close_gripper()
        self.move_joints(self.safe_joints, "lift")

    def place(self, place_joints):
        self.move_joints(place_joints, "place")
        self.open_gripper()
        self.move_joints(self.safe_joints, "retreat")

    def joints_from_pose(self, pose: Pose, tip_link="ee_link", timeout=1.0):
        """Solve IK for `pose` (geometry_msgs/Pose in the planning frame) and return ARM_JOINTS values."""
        state = RobotState(self.model)
        # Seed IK from the current state so the solution stays close to where the arm is now
        with self.robot.get_planning_scene_monitor().read_only() as scene:
            state.joint_positions = scene.current_state.joint_positions
        if not state.set_from_ik("ar_manipulator", pose, tip_link, timeout):
            raise RuntimeError(f"IK failed for pose: {pose}")
        positions = state.joint_positions
        return [positions[j] for j in ARM_JOINTS]

    def pick_pose(self, pick_pose: Pose):
        self.pick(self.joints_from_pose(pick_pose))
        
    def place_pose(self, place_pose: Pose):
        self.place(self.joints_from_pose(place_pose))
