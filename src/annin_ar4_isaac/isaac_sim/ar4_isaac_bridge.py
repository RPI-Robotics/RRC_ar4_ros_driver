"""AR4 setup + ros2_control bridge for Isaac Sim 5.1.

Run from Isaac Sim's Window > Script Editor with ar4_start.usd open.

First run: imports the AR4 URDF, deactivates the Franka, places the AR4 where
the Franka was, sets joint drives and moves the hand camera to the gripper.
Save the stage afterwards (File > Save) to keep these changes.

Every run: starts a TCP server that the annin_ar4_isaac/IsaacTcpSystem
ros2_control plugin (in the ROS container) connects to. Re-run this script
after restarting Isaac Sim. Re-running it in the same session is safe.
"""
import builtins
import math
import os
import socket

import numpy as np
import omni.kit.commands
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

# ----------------------------------------------------------------- settings --
EXPORT_DIR = os.path.join(os.path.expanduser("~"), "Desktop", "ar4_isaac")
URDF_PATH = os.path.join(EXPORT_DIR, "mk5.urdf")
ROBOT_USD_PATH = os.path.join(EXPORT_DIR, "usd", "mk5.usd")

ROBOT_PRIM_PATH = "/AR4"
OLD_ROBOT_PRIM_PATH = "/Franka"
# Same spot as the Franka. At zero joint angles the AR4 reaches along its -Y
# axis, so a +90 deg yaw makes it face +X toward the props like the Franka.
ROBOT_TRANSLATE = (0.0, 0.0, -0.0042729)
ROBOT_YAW_DEG = 90.0

# Angular drive gains are per degree (USD convention), linear ones per meter.
ARM_DRIVE = {"stiffness": 400.0, "damping": 40.0, "max_force": 1000.0}
GRIPPER_DRIVE = {"stiffness": 1000.0, "damping": 50.0, "max_force": 10.0}

BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 50555
# ------------------------------------------------------------------------------


def _set_import_option(config, name, value):
    setter = getattr(config, "set_" + name, None)
    if setter is not None:
        setter(value)
    else:
        setattr(config, name, value)


def import_urdf(stage):
    if not os.path.exists(URDF_PATH):
        raise FileNotFoundError(
            f"{URDF_PATH} not found. Run export_isaac_urdf.py in the ROS "
            "container and copy its output folder to the Desktop.")
    os.makedirs(os.path.dirname(ROBOT_USD_PATH), exist_ok=True)

    _, config = omni.kit.commands.execute("URDFCreateImportConfig")
    for name, value in {
            "merge_fixed_joints": False,
            "fix_base": True,
            "make_default_prim": True,
            "create_physics_scene": False,
            "import_inertia_tensor": True,
            "self_collision": False,
            "convex_decomp": False,
            "distance_scale": 1.0,
    }.items():
        _set_import_option(config, name, value)

    root_prims_before = {p.GetPath() for p in stage.GetPseudoRoot().GetChildren()}
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=URDF_PATH,
        import_config=config,
        dest_path=ROBOT_USD_PATH,
    )
    if not os.path.exists(ROBOT_USD_PATH):
        raise RuntimeError(
            "URDF import did not write " + ROBOT_USD_PATH + ". Import it "
            "manually with File > Import (Fix Base Link on, Merge Fixed "
            "Joints off), save it to that path, and re-run this script.")
    # The importer may also add the robot to the open stage; we reference it
    # ourselves below so the AR4 lives at ROBOT_PRIM_PATH.
    for prim in list(stage.GetPseudoRoot().GetChildren()):
        if prim.GetPath() not in root_prims_before:
            stage.RemovePrim(prim.GetPath())


def find_prim(stage, root_path, predicate):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if predicate(prim):
            return prim
    return None


def set_drive(prim, drive_type, gains):
    drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(gains["stiffness"])
    drive.CreateDampingAttr().Set(gains["damping"])
    drive.CreateMaxForceAttr().Set(gains["max_force"])


def setup_stage(stage):
    if stage.GetPrimAtPath(ROBOT_PRIM_PATH).IsValid():
        print(f"[AR4] {ROBOT_PRIM_PATH} already exists, skipping stage setup")
        return

    if not os.path.exists(ROBOT_USD_PATH):
        print(f"[AR4] Importing {URDF_PATH}")
        import_urdf(stage)

    old_robot = stage.GetPrimAtPath(OLD_ROBOT_PRIM_PATH)
    if old_robot.IsValid():
        old_robot.SetActive(False)
        print(f"[AR4] Deactivated {OLD_ROBOT_PRIM_PATH}")

    # The parent Xform holds the placement so it never conflicts with
    # transforms authored inside the imported robot asset.
    holder = UsdGeom.Xform.Define(stage, ROBOT_PRIM_PATH)
    holder.AddTranslateOp().Set(Gf.Vec3d(*ROBOT_TRANSLATE))
    holder.AddRotateZOp().Set(ROBOT_YAW_DEG)
    robot = stage.DefinePrim(ROBOT_PRIM_PATH + "/ar4")
    robot.GetReferences().AddReference(ROBOT_USD_PATH.replace("\\", "/"))

    for prim in Usd.PrimRange(robot):
        if prim.IsInstanceProxy():
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            # Visual-only sub-links (motor, cover, logo) have no <inertial> in
            # the URDF; keep PhysX from giving them a default 1 kg mass.
            mass = UsdPhysics.MassAPI(prim).GetMassAttr()
            if not mass or not mass.HasAuthoredValue() or mass.Get() <= 0:
                mass_api = UsdPhysics.MassAPI.Apply(prim)
                mass_api.CreateMassAttr().Set(0.001)
                mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1e-6, 1e-6, 1e-6))
        if prim.IsA(UsdPhysics.RevoluteJoint):
            set_drive(prim, "angular", ARM_DRIVE)
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            set_drive(prim, "linear", GRIPPER_DRIVE)

    gripper = find_prim(stage, ROBOT_PRIM_PATH,
                        lambda p: p.GetName() == "gripper_base_link")
    if gripper is not None:
        camera = UsdGeom.Camera.Define(stage,
                                       gripper.GetPath().AppendChild("Hand_Camera"))
        camera.CreateFocalLengthAttr().Set(18.147562)
        camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 1.0e7))
        # Offset to the side of the jaws, looking along the gripper's
        # approach axis (-Y of gripper_base_link). Adjust to taste.
        camera.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.04))
        camera.AddOrientOp().Set(Gf.Quatf(math.sqrt(0.5), -math.sqrt(0.5), 0, 0))
        _rebind_viewports(OLD_ROBOT_PRIM_PATH, camera.GetPath())

    print(f"[AR4] AR4 added at {ROBOT_PRIM_PATH}. Save the stage to keep it.")


def _rebind_viewports(old_prefix, new_camera_path):
    try:
        from omni.kit.viewport.window import get_viewport_window_instances
        for window in get_viewport_window_instances():
            api = window.viewport_api
            if str(api.camera_path).startswith(old_prefix):
                api.camera_path = new_camera_path
    except Exception as e:  # Viewport API differences are not fatal.
        print(f"[AR4] Could not rebind viewport camera: {e}")


class Ar4Bridge:
    """TCP server that connects Isaac Sim's AR4 articulation to ros2_control."""

    def __init__(self, stage):
        root = find_prim(stage, ROBOT_PRIM_PATH,
                         lambda p: p.HasAPI(UsdPhysics.ArticulationRootAPI))
        if root is None:
            raise RuntimeError(f"No articulation root found under {ROBOT_PRIM_PATH}")
        self._articulation_path = str(root.GetPath())

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((BRIDGE_HOST, BRIDGE_PORT))
        self._server.listen()
        self._server.setblocking(False)
        self._clients = {}  # socket -> receive buffer

        self._sim_view = None
        self._articulation = None
        self._dof_index = {}
        self._position_targets = None
        self._velocity_targets = None
        # Kept monotonic across Stop/Play so ROS nodes on sim time never see
        # the clock jump backwards.
        self._sim_time = 0.0

        self._step_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step)
        self._stop_sub = (omni.timeline.get_timeline_interface()
                          .get_timeline_event_stream()
                          .create_subscription_to_pop_by_type(
                              int(omni.timeline.TimelineEventType.STOP),
                              self._on_stop))
        print(f"[AR4] Bridge listening on {BRIDGE_HOST}:{BRIDGE_PORT} for "
              f"{self._articulation_path}. Press Play.")

    def shutdown(self):
        self._step_sub = None
        self._stop_sub = None
        for client in list(self._clients):
            client.close()
        self._clients.clear()
        self._server.close()

    def _on_stop(self, _event):
        self._articulation = None
        self._sim_view = None

    def _get_articulation(self):
        if self._articulation is None:
            import omni.physics.tensors as tensors
            self._sim_view = tensors.create_simulation_view("numpy")
            self._sim_view.set_subspace_roots("/")
            articulation = self._sim_view.create_articulation_view(
                self._articulation_path)
            if articulation is None or articulation.count == 0:
                return None
            self._dof_index = {
                name: i for i, name in enumerate(articulation.shared_metatype.dof_names)
            }
            # Hold the current pose until ROS sends commands.
            self._position_targets = articulation.get_dof_positions().astype(np.float32)
            self._velocity_targets = np.zeros_like(self._position_targets)
            self._articulation = articulation
            print(f"[AR4] Articulation DOFs: {list(self._dof_index)}")
        return self._articulation

    def _on_physics_step(self, dt):
        try:
            self._accept_clients()
            self._receive_commands()
            articulation = self._get_articulation()
            if articulation is None:
                return
            self._sim_time += dt
            indices = np.arange(1, dtype=np.int32)
            articulation.set_dof_position_targets(self._position_targets, indices)
            articulation.set_dof_velocity_targets(self._velocity_targets, indices)
            self._send_state(articulation)
        except Exception as e:
            print(f"[AR4] Bridge error: {e}")

    def _accept_clients(self):
        while True:
            try:
                client, address = self._server.accept()
            except BlockingIOError:
                return
            client.setblocking(False)
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._clients[client] = b""
            print(f"[AR4] ros2_control connected from {address}")

    def _drop_client(self, client):
        client.close()
        self._clients.pop(client, None)
        print("[AR4] ros2_control disconnected")

    def _receive_commands(self):
        for client in list(self._clients):
            try:
                data = client.recv(65536)
            except BlockingIOError:
                continue
            except OSError:
                self._drop_client(client)
                continue
            if not data:
                self._drop_client(client)
                continue
            buffer = self._clients[client] + data
            *lines, self._clients[client] = buffer.split(b"\n")
            for line in lines:
                self._apply_command(line.decode().split())

    def _apply_command(self, tokens):
        # C <n> <name> <position> <velocity> ...
        if len(tokens) < 2 or tokens[0] != "C" or self._position_targets is None:
            return
        for i in range(int(tokens[1])):
            name, position, velocity = tokens[2 + 3 * i:5 + 3 * i]
            dof = self._dof_index.get(name)
            if dof is None:
                continue
            position, velocity = float(position), float(velocity)
            if math.isfinite(position):
                self._position_targets[0, dof] = position
            self._velocity_targets[0, dof] = velocity if math.isfinite(velocity) else 0.0

    def _send_state(self, articulation):
        if not self._clients:
            return
        positions = articulation.get_dof_positions()[0]
        velocities = articulation.get_dof_velocities()[0]
        joints = " ".join(f"{name} {positions[i]:.6f} {velocities[i]:.6f}"
                          for name, i in self._dof_index.items())
        message = f"S {self._sim_time:.6f} {len(self._dof_index)} {joints}\n".encode()
        for client in list(self._clients):
            try:
                client.sendall(message)
            except (BlockingIOError, OSError):
                self._drop_client(client)


def main():
    previous = getattr(builtins, "_ar4_isaac_bridge", None)
    if previous is not None:
        previous.shutdown()
        builtins._ar4_isaac_bridge = None

    stage = omni.usd.get_context().get_stage()
    setup_stage(stage)
    builtins._ar4_isaac_bridge = Ar4Bridge(stage)


main()
