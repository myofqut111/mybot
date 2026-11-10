#!/usr/bin/env python3
"""Command-line navigation control for mybot without RViz.

由 Go2 nav_command.py 移植：去掉依赖旧版 /FollowWaypoints 的 patrol 子命令
（多点巡检统一走 /execute_patrol，见 README）。保留两个最常用命令：

设置 AMCL 初始位姿（导航启动后必须做一次）：
  ros2 run mybot_navigation nav_command.py initial-pose \
    --x 0.0 --y 0.0 --yaw 0.0

发送单个导航目标点：
  ros2 run mybot_navigation nav_command.py goal \
    --x 1.0 --y 0.5 --yaw 1.57

Yaw 默认弧度，加 --degrees 用角度。
"""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile


def yaw_to_quaternion(yaw):
    """Return the planar quaternion (z, w) for a yaw angle."""
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def angle_value(value, degrees):
    return math.radians(value) if degrees else value


def make_pose_stamped(node, frame_id, x, y, yaw):
    pose = PoseStamped()
    # Navigation targets are spatial. A zero stamp asks Nav2/TF for the
    # latest transform and remains valid if the action is queued briefly.
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
    return pose


def wait_for_subscriber(node, publisher, timeout):
    """Wait briefly so a one-shot initial-pose message is not sent too early."""
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if publisher.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return publisher.get_subscription_count() > 0


def publish_initial_pose(args):
    node = Node('mybot_set_initial_pose')
    publisher = node.create_publisher(
        PoseWithCovarianceStamped,
        args.topic,
        QoSProfile(depth=1),
    )
    try:
        if not wait_for_subscriber(node, publisher, args.wait_timeout):
            node.get_logger().error(
                'No subscriber found on {}. Start navigation and wait for '
                'AMCL to become active before setting the initial pose.'.format(
                    args.topic))
            return 1

        yaw = angle_value(args.yaw, args.degrees)
        message = PoseWithCovarianceStamped()
        message.header.frame_id = args.frame
        message.pose.pose.position.x = args.x
        message.pose.pose.position.y = args.y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = (
            yaw_to_quaternion(yaw))

        # ROS covariance ordering is x, y, z, roll, pitch, yaw.
        message.pose.covariance[0] = args.xy_std ** 2
        message.pose.covariance[7] = args.xy_std ** 2
        message.pose.covariance[35] = args.yaw_std ** 2

        for _ in range(args.repeat):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=args.interval)

        node.get_logger().info(
            'Initial pose published: frame={} x={:.3f} y={:.3f} '
            'yaw={:.3f} rad, xy_std={:.3f} m, yaw_std={:.3f} rad'.format(
                args.frame, args.x, args.y, yaw, args.xy_std, args.yaw_std))
        return 0
    finally:
        node.destroy_node()


def args_timeout(timeout):
    """Use a short bounded wait for action-server discovery."""
    return min(20.0, max(0.1, timeout))


def wait_for_action_result(
        node, client, action_name, goal, timeout, feedback_callback=None):
    if not client.wait_for_server(timeout_sec=args_timeout(timeout)):
        node.get_logger().error(
            'Action server {} is unavailable.'.format(action_name))
        return 2, None

    send_future = client.send_goal_async(
        goal, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=timeout)
    if not send_future.done() or send_future.result() is None:
        node.get_logger().error('Timed out while sending the action goal.')
        return 2, None

    goal_handle = send_future.result()
    if not goal_handle.accepted:
        node.get_logger().error('Goal was rejected by the action server.')
        return 2, None

    node.get_logger().info('Goal accepted.')
    result_future = goal_handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
    except KeyboardInterrupt:
        node.get_logger().warning('Cancel requested by user.')
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        return 130, None

    if not result_future.done() or result_future.result() is None:
        node.get_logger().error(
            'Navigation timed out after {:.1f} seconds; cancelling goal.'.format(
                timeout))
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        return 2, None

    wrapped_result = result_future.result()
    if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().error(
            'Navigation finished with action status {}.'.format(
                wrapped_result.status))
        return 2, wrapped_result.result
    return 0, wrapped_result.result


def send_single_goal(args):
    node = Node('mybot_send_navigation_goal')
    client = ActionClient(node, NavigateToPose, args.action)
    try:
        yaw = angle_value(args.yaw, args.degrees)
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(
            node, args.frame, args.x, args.y, yaw)

        last_report = {'time': 0.0}

        def feedback_callback(feedback_message):
            now = time.monotonic()
            if now - last_report['time'] < 1.0:
                return
            last_report['time'] = now
            feedback = feedback_message.feedback
            distance = getattr(feedback, 'distance_remaining', float('nan'))
            node.get_logger().info(
                'Navigating: distance_remaining={:.2f} m'.format(distance))

        node.get_logger().info(
            'Sending goal: frame={} x={:.3f} y={:.3f} yaw={:.3f} rad'.format(
                args.frame, args.x, args.y, yaw))
        code, _ = wait_for_action_result(
            node, client, args.action, goal, args.timeout, feedback_callback)
        if code == 0:
            node.get_logger().info('Goal reached successfully.')
        return code
    finally:
        client.destroy()
        node.destroy_node()


def positive_float(value):
    value = float(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError('value must be greater than zero')
    return value


def build_parser():
    parser = argparse.ArgumentParser(
        description='Operate mybot Nav2 from ROS 2 commands without RViz.')
    subparsers = parser.add_subparsers(dest='command', required=True)

    initial = subparsers.add_parser(
        'initial-pose', help='publish an AMCL initial pose')
    initial.add_argument('--x', type=float, required=True)
    initial.add_argument('--y', type=float, required=True)
    initial.add_argument('--yaw', type=float, required=True)
    initial.add_argument(
        '--degrees', action='store_true',
        help='interpret yaw as degrees instead of radians')
    initial.add_argument('--xy-std', type=positive_float, default=0.15)
    initial.add_argument('--yaw-std', type=positive_float, default=0.15)
    initial.add_argument('--frame', default='map')
    initial.add_argument('--topic', default='/initialpose')
    initial.add_argument('--wait-timeout', type=positive_float, default=5.0)
    initial.add_argument('--repeat', type=int, default=5)
    initial.add_argument('--interval', type=positive_float, default=0.2)
    initial.set_defaults(handler=publish_initial_pose)

    single = subparsers.add_parser(
        'goal', help='send one NavigateToPose goal')
    single.add_argument('--x', type=float, required=True)
    single.add_argument('--y', type=float, required=True)
    single.add_argument('--yaw', type=float, required=True)
    single.add_argument('--degrees', action='store_true')
    single.add_argument('--frame', default='map')
    single.add_argument('--action', default='/navigate_to_pose')
    single.add_argument('--timeout', type=positive_float, default=300.0)
    single.set_defaults(handler=send_single_goal)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    if getattr(args, 'repeat', 1) < 1:
        parser.error('--repeat must be at least 1')

    rclpy.init(args=None)
    try:
        return int(args.handler(args))
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
