# webots_spot_manipulation

Action and service servers (plus Python clients) to move the **Kinova Gen3** arm and open/close the
**Robotiq 2F gripper** on Spot in the [webots_ros2_spot](https://github.com/MASKOR/webots_ros2_spot)
simulation (`webots_spot` package), using MoveIt 2 on ROS 2 Humble.

| Interface | Type | Description |
|---|---|---|
| `/move_delta` | `man_interfaces/action/MoveDelta` | Move the end effector relative to its current pose (dx, dy, dz, droll, dpitch, dyaw) |
| `/move_to_frame` | `man_interfaces/action/MoveToFrame` | Move the end effector to a TF frame (e.g. a detected object) with an optional offset |
| `/gripper` | `man_interfaces/srv/Gripper` | Open, close or move the gripper to a position |

This repository contains two ROS 2 packages:

| Package | Description |
|---|---|
| [`webots_spot_manipulation`](webots_spot_manipulation) | Servers, clients, launch file and parameters |
| [`man_interfaces`](man_interfaces) | Action and service definitions (`MoveDelta`, `MoveToFrame`, `Gripper`) |

## How it works

```
MoveDelta / MoveToFrame goal
  └─ manipulator_server
       1. TF lookup (base_link -> end_effector_link / target frame) -> target pose
       2. /compute_ik (moveit_msgs/srv/GetPositionIK)              -> joint angles
       3. /move_action (moveit_msgs/action/MoveGroup, Pilz PTP)    -> plan + execute

Gripper request
  └─ gripper_server
       └─ /robotiq_gripper_controller/gripper_cmd (control_msgs/action/GripperCommand)
```

Only one arm goal runs at a time; new goals are rejected while the arm is moving. Canceling a goal
also cancels the running MoveIt motion.

## Requirements

- ROS 2 Humble (system Python **3.10**)
- MoveIt 2, `tf_transformations`, `control_msgs`
- `webots_spot` with the Gen3 MoveIt config (`resource/gen3.srdf`, `moveit_kinematics.yaml`)

## Build

```bash
cd ~/robotic_ws/src
git clone git@github.com:RRL-ALeRT/webots_spot_manipulation.git
cd ~/robotic_ws
colcon build --packages-select man_interfaces webots_spot_manipulation
source install/setup.bash
```

## Run

```bash
# Terminal 1: Webots simulation with Spot + Gen3
ros2 launch webots_spot spot_launch.py

# Terminal 2: MoveIt (move_group + RViz)
ros2 launch webots_spot moveit_launch.py

# Terminal 3: manipulator_server + gripper_server + test frame "man_test"
ros2 launch webots_spot_manipulation manipulator_server.launch.py
```

The launch file sets `use_sim_time:=true` by default and loads
[`config/params.yaml`](webots_spot_manipulation/config/params.yaml).

It also publishes a static TF frame `man_test` to test `/move_to_frame` (default: 10 cm along z of
the Kinova camera frame `kinova_color`). Position and parent can be changed, or the frame disabled:

```bash
ros2 launch webots_spot_manipulation manipulator_server.launch.py test_parent_frame:=base_link x:=0.6 z:=0.4
ros2 launch webots_spot_manipulation manipulator_server.launch.py publish_test_frame:=false

ros2 run webots_spot_manipulation manipulator_client man_test
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use the Webots clock |
| `publish_test_frame` | `true` | Publish the static test frame |
| `test_frame` | `man_test` | Name of the test frame |
| `test_parent_frame` | `kinova_color` | Parent of the test frame |
| `x y z` | `0.0 0.0 0.1` | Position in the parent frame [m] |
| `roll pitch yaw` | `0.0 0.0 0.0` | Orientation in the parent frame [rad] |

## Examples

### Command line clients

```bash
# Move the end effector: dx dy dz [droll dpitch dyaw]  (meters / radians, axes of base_link)
ros2 run webots_spot_manipulation manipulator_client 0.1           # 10 cm forward
ros2 run webots_spot_manipulation manipulator_client 0 0 0.05      # 5 cm up
ros2 run webots_spot_manipulation manipulator_client 0 0 0 0 0 0.3 # rotate 0.3 rad about z

# Move to a TF frame (keeps the current orientation)
ros2 run webots_spot_manipulation manipulator_client PlaceBox

# Gripper
ros2 run webots_spot_manipulation gripper_client open
ros2 run webots_spot_manipulation gripper_client close
ros2 run webots_spot_manipulation gripper_client 0.4               # finger_joint position [rad]
```

### ros2 action / service CLI

```bash
# 5 cm along the x axis of the gripper (tool frame) instead of base_link
ros2 action send_goal /move_delta man_interfaces/action/MoveDelta \
  "{dx: 0.05, frame_id: end_effector_link}" --feedback

# Slower motion (velocity scaling 0..1, 0 = default from params.yaml)
ros2 action send_goal /move_delta man_interfaces/action/MoveDelta \
  "{dz: -0.1, velocity_scaling: 0.05}" --feedback

# 5 cm above frame "P", keep the current orientation
ros2 action send_goal /move_to_frame man_interfaces/action/MoveToFrame \
  "{target_frame: P, offset: {z: 0.05}, orientation_mode: 0}" --feedback

# Align with the orientation of the target frame
ros2 action send_goal /move_to_frame man_interfaces/action/MoveToFrame \
  "{target_frame: P, orientation_mode: 1}" --feedback

# Fixed orientation in base_link (quaternion x y z w)
ros2 action send_goal /move_to_frame man_interfaces/action/MoveToFrame \
  "{target_frame: P, orientation_mode: 2, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}" --feedback

# Gripper: command 0 = OPEN, 1 = CLOSE, 2 = POSITION
ros2 service call /gripper man_interfaces/srv/Gripper "{command: 1}"
ros2 service call /gripper man_interfaces/srv/Gripper "{command: 2, position: 0.3}"
```

### Python: pick and place

```python
import rclpy

from webots_spot_manipulation.gripper_client import GripperClient
from webots_spot_manipulation.manipulator_client import ManipulatorActionClient as Manipulator


def main():
    rclpy.init()
    arm = Manipulator()
    gripper = GripperClient()

    gripper.open_gripper()
    gripper.wait()

    # Approach 10 cm above the object, then go down
    arm.move_to_frame('P', offset=(0.0, 0.0, 0.10))
    if not arm.wait():
        print(f'approach failed: {arm.result.message if arm.result else "goal rejected"}')
        return
    arm.move_delta(dz=-0.10, velocity_scaling=0.05)
    arm.wait()

    if not gripper.close_gripper() or not gripper.wait():
        print('grasp failed')
    # gripper.result.stalled is True if the fingers stopped on the object

    arm.move_delta(dz=0.15)  # lift
    arm.wait()

    arm.move_to_frame('PlaceBox', offset=(0.0, 0.0, 0.20))
    arm.wait()
    gripper.open_gripper()
    gripper.wait()

    arm.destroy_node()
    gripper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Both clients can also be used without blocking, like the other clients in `man_pkg`:

```python
arm.move_delta(dx=0.1)
while not arm.get_status():
    rclpy.spin_once(arm)
```

To use the clients from another package, add `<exec_depend>webots_spot_manipulation</exec_depend>` to its `package.xml`.

### Golog++ agent (gologpp-ros)

[gologpp-ros](https://github.com/MASKOR/gologpp-ros) supports `man_interfaces`
(`src/actions/gpp_man_interfaces.cpp`). The example agent
[`agents/manipulation.gpp`](https://github.com/MASKOR/gologpp-ros/blob/ros2/agents/manipulation.gpp)
(locally `~/golog_ws/src/gologpp-ros/agents/manipulation.gpp`) defines these Golog++ actions:

| Golog++ action | ROS interface | Arguments |
|---|---|---|
| `moveDelta(x, y, z, roll, pitch, yaw, frame)` | `/move_delta` | [m], [rad], `frame`: `""` = `base_link`, `"end_effector_link"` = tool frame |
| `moveToFrame(target, x, y, z)` | `/move_to_frame` | TF frame + offset [m] in `base_link`, keeps the current orientation |
| `gripper(cmd, pos)` | `/gripper` | `cmd`: 0 = open, 1 = close, 2 = position `pos` [rad] |

```
procedure main(){
    gripper(0, 0.0);                                       // open
    moveDelta(0.5, 0.0, 0.1, 0.0, 0.0, 0.0, "");           // 50 cm forward, 10 cm up
    moveDelta(-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, "");          // 50 cm back
    gripper(1, 0.0);                                       // close
    moveToFrame("man_test", 0.0, 0.0, 0.05);               // 5 cm above frame "man_test"
}
```

Run it:

```bash
# 1. Simulation and MoveIt (see "Run")

# 2. Servers + test frame "man_test" (10 cm along z of the Kinova camera frame "kinova_color")
ros2 launch webots_spot_manipulation manipulator_server.launch.py

# 3. Start the agent (loads agents/manipulation.gpp)
ros2 run gologpp_agent gologpp_agent manipulation
```

A failed arm action (e.g. no IK solution or missing TF frame) fails the Golog++ action. The
`gripper` action returns the `success` of the service as a bool.

gologpp-ros needs `man_interfaces` at build time: build `robotic_ws` first and source it before
building `golog_ws`.

## Interfaces

### MoveDelta.action

| Field | Description |
|---|---|
| `dx, dy, dz` | Translation [m] |
| `droll, dpitch, dyaw` | Rotation [rad] about the axes of `frame_id`, around the end-effector origin |
| `frame_id` | Axes the delta is expressed in: `""` = `base_link`, `end_effector_link` = tool frame, or any TF frame |
| `velocity_scaling` | 0..1, 0 = default |

### MoveToFrame.action

| Field | Description |
|---|---|
| `target_frame` | TF frame to move to |
| `offset` | Offset [m] added to the target position, in `base_link` axes |
| `orientation_mode` | `KEEP_CURRENT=0` keep current orientation, `TARGET_FRAME=1` use the target frame's orientation, `FIXED=2` use `orientation` |
| `orientation` | Quaternion in `base_link`, only for `FIXED` |
| `velocity_scaling` | 0..1, 0 = default |

Both actions return `success`, `message` and the computed `target_pose`, and publish the current
step as feedback (`computing target pose`, `computing IK`, `planning and executing`).

### Gripper.srv

| Field | Description |
|---|---|
| `command` | `OPEN=0`, `CLOSE=1`, `POSITION=2` |
| `position` | `finger_joint` position [rad], only for `POSITION` (0.0 open .. 0.8 closed) |
| `max_effort` | 0 = default |
| **Response** `success` | Goal reached, or stalled on an object |
| `stalled`, `position`, `message` | Controller result |

## Parameters ([`config/params.yaml`](webots_spot_manipulation/config/params.yaml))

### manipulator_action_server

| Parameter | Default | Description |
|---|---|---|
| `planning_group` | `manipulator` | MoveIt group from `gen3.srdf` |
| `planning_frame` | `base_link` | Frame for target poses and `offset` |
| `ee_link` | `end_effector_link` | Link that is moved (IK link) |
| `joint_names` | `joint_1` .. `joint_6` | Joints used for the joint goal |
| `pipeline_id` | `''` | Planning pipeline, empty = move_group default (Pilz) |
| `planner_id` | `PTP` | Pilz planner: `PTP` (joint space) or `LIN` (straight line) |
| `velocity_scaling` | `0.1` | Default velocity scaling |
| `acceleration_scaling` | `0.1` | Acceleration scaling |
| `planning_time` | `5.0` | Allowed planning time [s] |
| `planning_attempts` | `10` | Planning attempts |
| `ik_timeout` | `2.0` | IK timeout [s] |
| `avoid_collisions` | `true` | Collision-aware IK |

### gripper_server

| Parameter | Default | Description |
|---|---|---|
| `controller_action` | `/robotiq_gripper_controller/gripper_cmd` | GripperCommand action of the controller |
| `open_position` | `0.0` | `finger_joint` open (`gen3.srdf` state "open") |
| `close_position` | `0.6899` | `finger_joint` closed (`gen3.srdf` state "close") |
| `max_effort` | `0.0` | Effort limit, 0 = none |
| `timeout` | `5.0` | Timeout for the controller [s] |

## Troubleshooting

| Error | Cause / fix |
|---|---|
| `IK failed: INVALID_GROUP_NAME (-15)` | `planning_group` does not match the SRDF loaded by `moveit_launch.py` (must be `manipulator` for the Gen3) |
| `IK failed: NO_IK_SOLUTION (-31)` | Target pose not reachable. Use a smaller delta, another `orientation_mode` or an offset |
| `planning/execution failed: UNKNOWN (0)` + `No ContextLoader for planner_id ''` in move_group | Pilz needs an explicit `planner_id` (`PTP`, `LIN`, `CIRC`) |
| `Could not transform ... to base_link` | Simulation not running or the target frame is not published in TF |
| `Rejecting goal: manipulator is already moving` | Wait until the current goal is finished or cancel it |
| `No module named 'rclpy._rclpy_pybind11'` (`cpython-311`) | Started with Python 3.11. Humble needs `/usr/bin/python3` (3.10); in VS Code select it via *Python: Select Interpreter* |

## License

[MIT](LICENSE)
