"""ROS 2 node that coordinates per-waypoint inspection tasks."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fishbot_patrol_interfaces.msg import PatrolTaskRequest, PatrolTaskResult

from .core import EventKind, TaskManagerCore, TaskRequest, TaskResult


class PatrolTaskManagerNode(Node):
    """Dispatch inspection work and return one result per patrol point."""

    def __init__(self):
        """Configure publishers, subscriptions, state, and timeout timer."""
        super().__init__('patrol_task_manager')
        self.declare_parameter('task_timeout_s', 120.0)
        self.declare_parameter('tick_period_s', 0.05)
        self.declare_parameter('completed_cache_size', 256)

        timeout_s = float(self.get_parameter('task_timeout_s').value)
        tick_period_s = float(self.get_parameter('tick_period_s').value)
        cache_size = int(self.get_parameter('completed_cache_size').value)
        if timeout_s <= 0.0:
            raise ValueError('task_timeout_s must be greater than zero')
        if tick_period_s <= 0.0:
            raise ValueError('tick_period_s must be greater than zero')

        self._core = TaskManagerCore(timeout_s, cache_size)

        reliable = QoSProfile(depth=20)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        latched = QoSProfile(depth=20)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._dispatch_pub = self.create_publisher(
            PatrolTaskRequest, '/patrol/task/dispatch', reliable)
        self._result_pub = self.create_publisher(
            PatrolTaskResult, '/patrol/task/result', latched)
        self._status_pub = self.create_publisher(
            PatrolTaskResult, '/patrol/task/status', latched)
        self._request_sub = self.create_subscription(
            PatrolTaskRequest, '/patrol/task/request', self._on_request,
            reliable)
        self._external_result_sub = self.create_subscription(
            PatrolTaskResult, '/patrol/task/external_result',
            self._on_external_result, reliable)
        self._timer = self.create_timer(tick_period_s, self._on_tick)

        self.get_logger().info(
            'patrol task manager ready (timeout={:.3f}s)'.format(timeout_s))

    def _on_request(self, msg: PatrolTaskRequest) -> None:
        request = TaskRequest(
            execution_id=msg.execution_id,
            route_name=msg.route_name,
            point_index=int(msg.point_index),
            point_id=msg.point_id,
            task_id=msg.task_id,
            empty_wait_seconds=float(msg.empty_wait_seconds),
        )
        events = self._core.accept(request, time.monotonic())
        self._publish_events(events, dispatch_message=msg)

    def _on_external_result(self, msg: PatrolTaskResult) -> None:
        external = TaskResult(
            execution_id=msg.execution_id,
            route_name=msg.route_name,
            point_index=int(msg.point_index),
            point_id=msg.point_id,
            task_id=msg.task_id,
            status=int(msg.status),
            message=msg.message,
        )
        events = self._core.receive_external(external, time.monotonic())
        if not events:
            self.get_logger().warning(
                'ignored unmatched external result '
                'execution_id=%r point_index=%d' %
                (msg.execution_id, msg.point_index))
            return
        self._publish_events(events)

    def _on_tick(self) -> None:
        self._publish_events(self._core.tick(time.monotonic()))

    def _publish_events(self, events, dispatch_message=None) -> None:
        for event in events:
            if event.kind == EventKind.DISPATCH:
                # The original request retains its Header and exact payload.
                if dispatch_message is None:
                    self.get_logger().error(
                        'internal error: dispatch event has no request '
                        'payload')
                    continue
                self._dispatch_pub.publish(dispatch_message)
                continue

            msg = self._result_message(event.result)
            if event.kind == EventKind.STATUS:
                self._status_pub.publish(msg)
                if event.result.status == PatrolTaskResult.FAILED:
                    self.get_logger().warning(event.result.message)
            elif event.kind == EventKind.RESULT:
                self._result_pub.publish(msg)

    def _result_message(self, result: TaskResult) -> PatrolTaskResult:
        msg = PatrolTaskResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.execution_id = result.execution_id
        msg.route_name = result.route_name
        msg.point_index = result.point_index
        msg.point_id = result.point_id
        msg.task_id = result.task_id
        msg.status = result.status
        msg.message = result.message
        return msg


def main(args=None):
    """Run the patrol task manager node."""
    rclpy.init(args=args)
    node = PatrolTaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
