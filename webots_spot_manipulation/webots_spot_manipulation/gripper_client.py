import sys

import rclpy
from rclpy.node import Node

from man_interfaces.srv import Gripper


class GripperClient(Node):
    """Client for the webots_spot_manipulation service /gripper."""

    def __init__(self, node_name='gripper_client'):
        super().__init__(node_name)

        self.cli = self.create_client(Gripper, '/gripper')
        self.done = True
        self.result = None

    def open_gripper(self):
        """Open the gripper."""
        return self.send_request(Gripper.Request.OPEN)

    def close_gripper(self):
        """Close the gripper."""
        return self.send_request(Gripper.Request.CLOSE)

    def move_gripper(self, position):
        """Move finger_joint to position [rad] (0.0 open .. 0.8 closed)."""
        return self.send_request(Gripper.Request.POSITION, position)

    def send_request(self, command, position=0.0, max_effort=0.0):
        self.done = False
        self.result = None

        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /gripper not available')
            self.done = True
            return False

        req = Gripper.Request()
        req.command = command
        req.position = float(position)
        req.max_effort = float(max_effort)

        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.response_callback)
        return True

    def response_callback(self, future):
        self.result = future.result()
        self.get_logger().info(f'Result: success={self.result.success}, {self.result.message}')
        self.done = True

    def get_status(self):
        return self.done

    def wait(self):
        """Spin until the current request is finished, return True on success."""
        while rclpy.ok() and not self.done:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.result is not None and self.result.success


def main(args=None):
    """Usage: gripper_client open | close | <position>."""
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    client = GripperClient()

    command = argv[0] if argv else 'open'
    if command == 'open':
        sent = client.open_gripper()
    elif command == 'close':
        sent = client.close_gripper()
    else:
        sent = client.move_gripper(float(command))

    if sent:
        client.wait()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
