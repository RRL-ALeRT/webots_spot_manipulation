import threading
import time
from copy import deepcopy

import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler
from tf_transformations import quaternion_inverse
from tf_transformations import quaternion_matrix
from tf_transformations import quaternion_multiply

from man_interfaces.action import MoveDelta, MoveToFrame


ERROR_CODE_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in dir(MoveItErrorCodes) if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
}


def error_name(code):
    return f'{ERROR_CODE_NAMES.get(code.val, "UNKNOWN")} ({code.val})'


class GoalFailed(Exception):
    pass


class GoalCanceled(Exception):
    pass


class ManipulatorActionServer(Node):
    """Move the end effector with MoveIt: /compute_ik for the target pose, /move_action to plan and execute."""

    def __init__(self):
        super().__init__('manipulator_action_server')

        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('ee_link', 'end_effector_link')
        self.declare_parameter('joint_names', [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('pipeline_id', '')
        self.declare_parameter('planner_id', 'PTP')
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('acceleration_scaling', 0.1)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('planning_attempts', 10)
        self.declare_parameter('ik_timeout', 2.0)
        self.declare_parameter('avoid_collisions', True)

        self.cb_group = ReentrantCallbackGroup()
        self.busy_lock = threading.Lock()
        self.joint_state = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1,
                                 callback_group=self.cb_group)

        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        self.move_cli = ActionClient(self, MoveGroup, '/move_action', callback_group=self.cb_group)

        ActionServer(
            self, MoveDelta, 'move_delta',
            execute_callback=self.execute_move_delta,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group)
        ActionServer(
            self, MoveToFrame, 'move_to_frame',
            execute_callback=self.execute_move_to_frame,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group)

        self.get_logger().info('Action servers "move_delta" and "move_to_frame" ready')

    def param(self, name):
        return self.get_parameter(name).value

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    # ---------------------------------------------------------------- action callbacks

    def goal_cb(self, goal_request):
        if self.busy_lock.locked():
            self.get_logger().warn('Rejecting goal: manipulator is already moving')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_move_delta(self, goal_handle):
        def compute_target(goal):
            planning_frame = self.param('planning_frame')
            p_ee, q_ee = self.lookup(planning_frame, self.param('ee_link'))
            _, q_ref = self.lookup(planning_frame, goal.frame_id or planning_frame)

            # Translation: delta given in the reference frame, rotated into the planning frame
            r_ref = quaternion_matrix(q_ref)[:3, :3]
            position = p_ee + r_ref @ np.array([goal.dx, goal.dy, goal.dz])

            # Rotation about the reference frame axes: q_ref * q_delta * q_ref^-1 * q_ee
            q_delta = quaternion_from_euler(goal.droll, goal.dpitch, goal.dyaw)
            q_delta = quaternion_multiply(quaternion_multiply(q_ref, q_delta), quaternion_inverse(q_ref))
            orientation = quaternion_multiply(q_delta, q_ee)
            return self.make_pose(position, orientation)

        return self.execute(goal_handle, MoveDelta, compute_target)

    def execute_move_to_frame(self, goal_handle):
        def compute_target(goal):
            if not goal.target_frame:
                raise GoalFailed('target_frame is empty')
            planning_frame = self.param('planning_frame')
            p_target, q_target = self.lookup(planning_frame, goal.target_frame)
            position = p_target + np.array([goal.offset.x, goal.offset.y, goal.offset.z])

            if goal.orientation_mode == MoveToFrame.Goal.TARGET_FRAME:
                orientation = q_target
            elif goal.orientation_mode == MoveToFrame.Goal.FIXED:
                o = goal.orientation
                orientation = np.array([o.x, o.y, o.z, o.w])
                norm = np.linalg.norm(orientation)
                if norm < 1e-9:
                    raise GoalFailed('orientation must not be a zero quaternion')
                orientation /= norm
            elif goal.orientation_mode == MoveToFrame.Goal.KEEP_CURRENT:
                _, orientation = self.lookup(planning_frame, self.param('ee_link'))
            else:
                raise GoalFailed(f'unknown orientation_mode {goal.orientation_mode}')
            return self.make_pose(position, orientation)

        return self.execute(goal_handle, MoveToFrame, compute_target)

    def execute(self, goal_handle, action_type, compute_target):
        result = action_type.Result()
        if not self.busy_lock.acquire(blocking=False):
            result.message = 'manipulator is already moving'
            goal_handle.abort()
            return result

        try:
            self.publish_state(goal_handle, action_type, 'computing target pose')
            result.target_pose = compute_target(goal_handle.request)
            p = result.target_pose.pose.position
            self.get_logger().info(f'Target pose: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}')

            self.publish_state(goal_handle, action_type, 'computing IK')
            target_angles = self.compute_ik(result.target_pose, goal_handle)

            self.publish_state(goal_handle, action_type, 'planning and executing')
            self.plan_and_execute(target_angles, goal_handle.request.velocity_scaling, goal_handle)

            result.success = True
            result.message = 'goal reached'
            goal_handle.succeed()
        except GoalCanceled:
            result.message = 'canceled'
            goal_handle.canceled()
        except GoalFailed as ex:
            result.message = str(ex)
            self.get_logger().error(f'Goal failed: {ex}')
            goal_handle.abort()
        finally:
            self.busy_lock.release()
        return result

    def publish_state(self, goal_handle, action_type, state):
        feedback = action_type.Feedback()
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    # ---------------------------------------------------------------- helpers

    def lookup(self, target_frame, source_frame):
        """Return (position, quaternion xyzw) of source_frame expressed in target_frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=1.0))
        except TransformException as ex:
            raise GoalFailed(f'Could not transform {source_frame} to {target_frame}: {ex}')
        tr, rot = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([rot.x, rot.y, rot.z, rot.w])

    def make_pose(self, position, orientation):
        pose = PoseStamped()
        pose.header.frame_id = self.param('planning_frame')
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = map(float, orientation)
        return pose

    def wait_for(self, future, goal_handle, timeout=None):
        """Block until the future is done; other executor threads complete it."""
        start = time.monotonic()
        while not future.done():
            if goal_handle.is_cancel_requested:
                raise GoalCanceled()
            if timeout is not None and time.monotonic() - start > timeout:
                raise GoalFailed('timed out')
            time.sleep(0.01)
        return future.result()

    def current_joint_state(self):
        start = time.monotonic()
        while self.joint_state is None:
            if time.monotonic() - start > 2.0:
                raise GoalFailed('no message on /joint_states')
            time.sleep(0.01)
        return self.joint_state

    def compute_ik(self, pose, goal_handle):
        if not self.ik_cli.wait_for_service(timeout_sec=2.0):
            raise GoalFailed('/compute_ik not available')

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.param('planning_group')
        req.ik_request.robot_state.joint_state = self.current_joint_state()
        req.ik_request.avoid_collisions = self.param('avoid_collisions')
        req.ik_request.ik_link_name = self.param('ee_link')
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout = Duration(seconds=self.param('ik_timeout')).to_msg()

        res = self.wait_for(self.ik_cli.call_async(req), goal_handle,
                            timeout=self.param('ik_timeout') + 5.0)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise GoalFailed(f'IK failed: {error_name(res.error_code)}')

        solution = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        try:
            return {name: solution[name] for name in self.param('joint_names')}
        except KeyError as ex:
            raise GoalFailed(f'IK solution is missing joint {ex}')

    def plan_and_execute(self, target_angles, velocity_scaling, goal_handle):
        if not self.move_cli.wait_for_server(timeout_sec=2.0):
            raise GoalFailed('/move_action not available')

        motion_plan_request = MotionPlanRequest()
        motion_plan_request.start_state.joint_state = self.current_joint_state()
        motion_plan_request.start_state.is_diff = True

        motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        motion_plan_request.workspace_parameters.header.frame_id = self.param('planning_frame')
        motion_plan_request.workspace_parameters.min_corner.x = -1.0
        motion_plan_request.workspace_parameters.min_corner.y = -1.0
        motion_plan_request.workspace_parameters.min_corner.z = -1.0
        motion_plan_request.workspace_parameters.max_corner.x = 1.0
        motion_plan_request.workspace_parameters.max_corner.y = 1.0
        motion_plan_request.workspace_parameters.max_corner.z = 1.0

        jc = JointConstraint()
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0

        constraints = Constraints()
        for (joint, angle) in target_angles.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))
        motion_plan_request.goal_constraints.append(constraints)

        motion_plan_request.pipeline_id = self.param('pipeline_id')
        motion_plan_request.planner_id = self.param('planner_id')
        motion_plan_request.group_name = self.param('planning_group')
        motion_plan_request.num_planning_attempts = self.param('planning_attempts')
        motion_plan_request.allowed_planning_time = self.param('planning_time')
        motion_plan_request.max_velocity_scaling_factor = velocity_scaling or self.param('velocity_scaling')
        motion_plan_request.max_acceleration_scaling_factor = self.param('acceleration_scaling')
        motion_plan_request.max_cartesian_speed = 0.0

        planning_options = PlanningOptions()
        planning_options.plan_only = False
        planning_options.look_around = False
        planning_options.look_around_attempts = 0
        planning_options.max_safe_execution_cost = 0.
        planning_options.replan = True
        planning_options.replan_attempts = 10
        planning_options.replan_delay = 0.1

        goal_msg = MoveGroup.Goal()
        goal_msg.request = motion_plan_request
        goal_msg.planning_options = planning_options

        move_goal_handle = self.wait_for(self.move_cli.send_goal_async(goal_msg), goal_handle)
        if not move_goal_handle.accepted:
            raise GoalFailed('MoveGroup goal rejected')

        try:
            res = self.wait_for(move_goal_handle.get_result_async(), goal_handle)
        except GoalCanceled:
            self.get_logger().info('Canceling MoveGroup goal')
            move_goal_handle.cancel_goal_async()
            raise

        if res.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise GoalFailed(f'planning/execution failed: {error_name(res.result.error_code)}')


def main():
    rclpy.init()
    node = ManipulatorActionServer()
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
