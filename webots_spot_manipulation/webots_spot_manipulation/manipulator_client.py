import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from man_interfaces.action import MoveDelta, MoveToFrame


class ManipulatorActionClient(Node):
    """Client for the webots_spot_manipulation actions /move_delta and /move_to_frame."""

    KEEP_CURRENT = MoveToFrame.Goal.KEEP_CURRENT
    TARGET_FRAME = MoveToFrame.Goal.TARGET_FRAME
    FIXED = MoveToFrame.Goal.FIXED

    def __init__(self, node_name='manipulator_client'):
        super().__init__(node_name)

        self._delta_client = ActionClient(self, MoveDelta, '/move_delta')
        self._frame_client = ActionClient(self, MoveToFrame, '/move_to_frame')
        self.done = True
        self.result = None

    def move_delta(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0,
                   frame_id='', velocity_scaling=0.0):
        """Move the end effector relative to its current pose (frame_id '' = base_link)."""
        goal_msg = MoveDelta.Goal()
        goal_msg.dx = float(dx)
        goal_msg.dy = float(dy)
        goal_msg.dz = float(dz)
        goal_msg.droll = float(droll)
        goal_msg.dpitch = float(dpitch)
        goal_msg.dyaw = float(dyaw)
        goal_msg.frame_id = frame_id
        goal_msg.velocity_scaling = float(velocity_scaling)
        return self._send_goal(self._delta_client, goal_msg)

    def move_to_frame(self, target_frame, offset=(0.0, 0.0, 0.0), orientation_mode=KEEP_CURRENT,
                      orientation=(0.0, 0.0, 0.0, 1.0), velocity_scaling=0.0):
        """Move the end effector to a TF frame, offset given in base_link."""
        goal_msg = MoveToFrame.Goal()
        goal_msg.target_frame = target_frame
        goal_msg.offset.x, goal_msg.offset.y, goal_msg.offset.z = map(float, offset)
        goal_msg.orientation_mode = orientation_mode
        (goal_msg.orientation.x, goal_msg.orientation.y,
         goal_msg.orientation.z, goal_msg.orientation.w) = map(float, orientation)
        goal_msg.velocity_scaling = float(velocity_scaling)
        return self._send_goal(self._frame_client, goal_msg)

    def _send_goal(self, client, goal_msg):
        self.done = False
        self.result = None

        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server {client._action_name} not available')
            self.done = True
            return False

        self._send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.done = True
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.result = future.result().result
        self.get_logger().info(f'Result: success={self.result.success}, {self.result.message}')
        self.done = True

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'State: {feedback_msg.feedback.state}')

    def get_status(self):
        return self.done

    def wait(self):
        """Spin until the current goal is finished, return True on success."""
        while rclpy.ok() and not self.done:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.result is not None and self.result.success


def main(args=None):
    """Usage: manipulator_client <frame>  |  manipulator_client dx dy dz [droll dpitch dyaw]."""
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    client = ManipulatorActionClient()

    try:
        values = [float(v) for v in argv]
        sent = client.move_delta(*values)
    except ValueError:
        sent = client.move_to_frame(argv[0])

    if sent:
        client.wait()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
