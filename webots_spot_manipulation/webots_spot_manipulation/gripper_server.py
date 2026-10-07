import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand

from man_interfaces.srv import Gripper


class GripperServer(Node):
    """Service /gripper that forwards open/close commands to the gripper controller (GripperCommand action)."""

    def __init__(self):
        super().__init__('gripper_server')

        self.declare_parameter('controller_action', '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('open_position', 0.0)
        self.declare_parameter('close_position', 0.6899)
        self.declare_parameter('max_effort', 0.0)
        self.declare_parameter('timeout', 5.0)

        self.cb_group = ReentrantCallbackGroup()
        self.action_client = ActionClient(
            self, GripperCommand, self.param('controller_action'), callback_group=self.cb_group)
        self.create_service(Gripper, 'gripper', self.gripper_cb, callback_group=self.cb_group)

        self.get_logger().info('Service "gripper" ready')

    def param(self, name):
        return self.get_parameter(name).value

    def gripper_cb(self, request, response):
        if request.command == Gripper.Request.OPEN:
            position = self.param('open_position')
        elif request.command == Gripper.Request.CLOSE:
            position = self.param('close_position')
        elif request.command == Gripper.Request.POSITION:
            position = request.position
        else:
            response.message = f'unknown command {request.command}'
            return response

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            response.message = f'{self.param("controller_action")} not available'
            self.get_logger().error(response.message)
            return response

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(request.max_effort or self.param('max_effort'))

        self.get_logger().info(f'Moving gripper to {position:.3f}')
        goal_handle = self.wait_for(self.action_client.send_goal_async(goal_msg))
        if goal_handle is None or not goal_handle.accepted:
            response.message = 'goal rejected or timed out'
            return response

        result = self.wait_for(goal_handle.get_result_async())
        if result is None:
            goal_handle.cancel_goal_async()
            response.message = 'timed out'
            return response

        response.position = result.result.position
        response.stalled = result.result.stalled
        # Closing on an object stalls the fingers before the goal is reached, that counts as success
        response.success = result.result.reached_goal or result.result.stalled
        response.message = 'stalled (object grasped?)' if response.stalled else 'goal reached'
        self.get_logger().info(f'Gripper: {response.message}, position {response.position:.3f}')
        return response

    def wait_for(self, future):
        """Block until the future is done; other executor threads complete it. None on timeout."""
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > self.param('timeout'):
                return None
            time.sleep(0.01)
        return future.result()


def main():
    rclpy.init()
    node = GripperServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
