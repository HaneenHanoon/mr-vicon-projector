# Steps to Mixed Reality (Vicon → Gazebo → Robot) — Order of Operations

**Updated:** 2025-10-22  
**Environment:** Ubuntu 22.04, ROS 2 **Humble**, Gazebo **Classic 11**  
**Package:** `mr_vicon_projector`

This section gives the exact order to bring the system up, what each step does, and the commands to run.

---

## 0) Prerequisites (install once)

```bash
# ROS 2 (desktop is convenient for RViz); source in every new shell
sudo apt update
sudo apt install -y ros-humble-desktop python3-pip

# Gazebo Classic + ROS interfaces
sudo apt install -y gazebo11 libgazebo11-dev
sudo apt install -y ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs

# Vicon ↔ ROS 2 bridge (VRPN)
sudo apt install -y ros-humble-vrpn-mocap

# iRobot Create 3 messages (actions/services used below)
sudo apt install -y ros-humble-irobot-create-msgs

# Navigation2 (needed by TB3 actor demos and general nav tooling)
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# Optional: TurtleBot3 if you use TB3 actors in Gazebo
# sudo apt install -y ros-humble-turtlebot3-msgs ros-humble-turtlebot3-gazebo

# Python deps (if your nodes use them)
pip3 install --user transforms3d
```

> After installation, clone this repo into `~/ros2_ws/src`, run `rosdep install --from-paths src -y --ignore-src`, then `colcon build` and `source install/setup.bash`.

---

## 1) Start Vicon → ROS 2 stream (VRPN client)

**Why:** Subscribes to the Vicon robot object and publishes its pose into ROS 2 (TF + topics).  
**Command:** replace `192.168.1.5` with your Vicon server IP.

```bash
ros2 launch vrpn_mocap client.launch.yaml   server:=192.168.1.5 port:=3883   sensor_data_qos:=false update_freq:=10.0 frame_id:=world
```

You should now see Vicon poses on something like `/vrpn_mocap/<subject>/pose`.

---

## 2) Launch Gazebo world

**Why:** Spawns the simulated world/obstacles and the robot model/frames in `world`.  
**Command (new package name):**
```bash
ros2 launch mr_vicon_projector my_robot_gazebo.launch.xml
```

Wait until Gazebo is stable (no new model spawns), then continue.

---

## 3) Mirror Vicon pose into Gazebo (bridge)

**Why:** Transforms Vicon poses to the Gazebo/world frame and updates the model in sim. Also republishes a clean pose topic for planners.

```bash
# Default params usually work; specify your Vicon topic and model if needed
ros2 run mr_vicon_projector mr_vicon_to_gazebo   --ros-args -p vicon_topic:=/vrpn_mocap/iRobot_create3_te/pose -p model_name:=my_robot
```

Outputs:
- Sets Gazebo entity state to match the tracked robot.
- Publishes `/gazebo_robot_pose` for downstream planners.

---

## 4) Load calibration (recommended)

**Why:** Uses `config/calibration.yaml` so you *do not* have to do interactive calibration.  
**Check:**

```bash
# Should exist in the installed share dir
ros2 pkg prefix mr_vicon_projector
# Then verify config/calibration.yaml exists in share/mr_vicon_projector/config/
```

If missing, some planners fall back to interactive calibration.

---

## 5) Start ONE planner/demonstrator

Choose **exactly one** of the following in a new sourced terminal:

```bash
# RRT with external/auto calibration support
ros2 run mr_vicon_projector mr_rrt_auto_cal

# RRT* with external/auto calibration support
ros2 run mr_vicon_projector mr_rrtstar_auto_cal

# Dynamic replanner (TTC + soft‑brake + replan)
ros2 run mr_vicon_projector mr_dynamic_replan
```

What they do:
- Convert planned waypoints into the robot frame using the Vicon↔Robot transform (from YAML if present).
- Publish markers (planned path, traveled path) for RViz.
- In the dynamic case, predict moving obstacles, apply soft‑brake when TTC < threshold, then replan.

---

## 6) Robot‑side calibration move (straight‑line heading)

**Why:** To apply/confirm the transform matrix, send a short straight move so the robot’s heading is well‑defined relative to Vicon.

```bash
# Send a short forward move (e.g., 0.20 m)
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: false, goal_pose: {pose: {position: {x: 0.2, y: 0.0, z: 0.0},
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

> Tip: Use small distances (0.2–0.5 m) in an open area so Vicon tracks reliably.

---

## 7) New start point? Reset odometry **before** the next calibration move

**Why:** If you pick a new physical start location, zero the robot odom so the transform stays consistent.

```bash
ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Then send a new short `navigate_to_position` action (Step 6) to re‑establish heading for the new start.

---

## 8) If the robot does not respond to NavigateToPosition

On Create 3, the safety override may block motion. You can set it via parameters:

```bash
ros2 service call /motion_control/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'safety_override', value: {type: 4, string_value: 'full'}}]}"
```

> Use with care; restore defaults when done.

---

## 9) (Optional) RViz for visualization

```bash
rviz2
# Add /planned_marker, /traveled_marker, TF, LaserScan (if available), and the robot model.
```

---

## 10) Shutdown order

1. Stop the planner node.  
2. Stop the Vicon→Gazebo bridge.  
3. Close Gazebo.  
4. Stop the VRPN client.

This avoids lingering TF publishers and ensures a clean next run.
