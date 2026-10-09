# annin_ar4_isaac

Runs the AR4 in Isaac Sim (on Windows) in place of Gazebo, controlled by
ros2_control and MoveIt in the devcontainer.

Isaac Sim's ROS 2 bridge can't reach the Docker Desktop devcontainer over
DDS, so this package uses a plain TCP link instead:

```
MoveIt -> joint_trajectory_controller / gripper_controller
       -> annin_ar4_isaac/IsaacTcpSystem (ros2_control plugin, container)
       -> TCP host.docker.internal:50555
       -> ar4_isaac_bridge.py (Isaac Sim Script Editor, Windows)
       -> AR4 articulation
```

The bridge sends joint states and simulation time back. The plugin publishes
`/clock`, so all ROS nodes run with `use_sim_time`.

## One-time setup

1. In the devcontainer, build and export the URDF:

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 run annin_ar4_isaac export_isaac_urdf.py --output isaac_export
   ```

2. Copy `isaac_export/` (contents) and `isaac_sim/ar4_isaac_bridge.py` to
   `C:\Users\<you>\Desktop\ar4_isaac\`. Edit the settings at the top of the
   script if you use another location.

3. In Isaac Sim, open `ar4_start.usd`, open **Window > Script Editor**, load
   `ar4_isaac_bridge.py` and run it. The first run imports the URDF to
   `ar4_isaac\usd\mk5.usd`, deactivates `/Franka`, adds the AR4 at `/AR4`
   with joint drives, and moves the hand camera to the gripper.
   Save the stage to keep the changes.

## Every session

1. In Isaac Sim, open the saved stage, run `ar4_isaac_bridge.py` from the
   Script Editor, and press **Play**.
2. In the devcontainer, run:

   ```bash
   ros2 launch annin_ar4_isaac isaac.launch.py
   ros2 launch annin_ar4_moveit_config moveit.launch.py use_sim_time:=true
   ```

The launch order does not matter; the plugin reconnects automatically.
Time only advances while the simulation is playing.

## Tuning

Joint drive gains, robot placement, and the bridge port are at the top of
`ar4_isaac_bridge.py`. To change the port on the ROS side, pass
`isaac_port:=<port>` to `isaac.launch.py`.
