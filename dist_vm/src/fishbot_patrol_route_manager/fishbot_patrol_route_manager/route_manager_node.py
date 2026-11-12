"""ROS node that records AMCL poses and publishes the active patrol route."""

import math
import time
from typing import Any, Dict

from geometry_msgs.msg import PoseWithCovarianceStamped
from fishbot_patrol_interfaces.msg import PatrolPoint, PatrolRoute
from fishbot_patrol_interfaces.srv import (
    ClearRoute,
    DeletePoint,
    GetRoute,
    RecordPoint,
    SetActiveRoute,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .pose_samples import PoseSample, PoseSampleBuffer
from .route_store import RouteStore


class PatrolRouteManager(Node):
    """Persistent service API for recording and selecting patrol routes."""

    def __init__(self) -> None:
        super().__init__('patrol_route_manager')
        self.declare_parameter(
            'route_file', '~/.ros/fishbot_patrol_routes.json'
        )
        # AMCL publishes /amcl_pose on filter updates, not at a fixed timer
        # rate.  A stationary robot can therefore have multi-second gaps.
        # Keep a bounded freshness window while avoiding false failures during
        # normal stationary point recording.
        self.declare_parameter('pose_max_age', 15.0)
        self.declare_parameter('pose_sample_window', 3)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')

        route_file = migrate_legacy_path(self.get_parameter('route_file').value)
        self._pose_max_age = float(self.get_parameter('pose_max_age').value)
        sample_window = int(self.get_parameter('pose_sample_window').value)
        self._global_frame = str(
            self.get_parameter('global_frame').value
        ).strip()
        self._robot_base_frame = str(
            self.get_parameter('robot_base_frame').value
        ).strip()
        if self._pose_max_age <= 0.0 or not math.isfinite(self._pose_max_age):
            raise ValueError('pose_max_age must be finite and greater than zero')
        if not self._global_frame or not self._robot_base_frame:
            raise ValueError('global_frame and robot_base_frame must not be empty')
        self._samples = PoseSampleBuffer(sample_window)
        self._store = RouteStore(route_file)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._route_publisher = self.create_publisher(
            PatrolRoute, '/patrol_route/active', route_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10
        )
        self.create_service(
            RecordPoint, '/patrol_route/record_point', self._record_point
        )
        self.create_service(
            GetRoute, '/patrol_route/get', self._get_route
        )
        self.create_service(
            SetActiveRoute, '/patrol_route/set_active', self._set_active
        )
        self.create_service(
            ClearRoute, '/patrol_route/clear', self._clear_route
        )
        self.create_service(
            DeletePoint, '/patrol_route/delete_point', self._delete_point
        )

        self._publish_active_route()
        self.get_logger().info(
            'Patrol route manager loaded %s' % self._store.path
        )

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        try:
            sample = PoseSample(
                receipt_time=time.monotonic(),
                stamp_sec=message.header.stamp.sec,
                stamp_nanosec=message.header.stamp.nanosec,
                frame_id=message.header.frame_id,
                position=(pose.position.x, pose.position.y, pose.position.z),
                orientation=(
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                ),
            )
            if not message.header.frame_id:
                raise ValueError('AMCL pose frame_id is empty')
            if not all(math.isfinite(value) for value in (
                *sample.position, *sample.orientation
            )):
                raise ValueError('AMCL pose contains a non-finite value')
            if sum(value * value for value in sample.orientation) < 1.0e-12:
                raise ValueError('AMCL pose contains a zero-length quaternion')
            self._samples.add(sample)
        except ValueError as error:
            self.get_logger().warning('Ignoring invalid AMCL pose: %s' % error)

    def _lookup_tf_pose(self):
        """Return the latest map-frame robot pose, or its TF failure reason."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_base_frame, Time()
            )
        except TransformException as error:
            return None, str(error)

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x, translation.y, translation.z,
            rotation.x, rotation.y, rotation.z, rotation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return None, 'TF transform contains a non-finite value'
        if sum(value * value for value in values[3:]) < 1.0e-12:
            return None, 'TF transform contains a zero-length quaternion'

        return PoseSample(
            receipt_time=time.monotonic(),
            stamp_sec=transform.header.stamp.sec,
            stamp_nanosec=transform.header.stamp.nanosec,
            frame_id=self._global_frame,
            position=values[:3],
            orientation=values[3:],
        ), ''

    @staticmethod
    def _pose_dict(sample: PoseSample) -> Dict[str, Any]:
        return {
            'header': {
                'stamp': {
                    'sec': sample.stamp_sec,
                    'nanosec': sample.stamp_nanosec,
                },
                'frame_id': sample.frame_id,
            },
            'position': dict(zip(('x', 'y', 'z'), sample.position)),
            'orientation': dict(
                zip(('x', 'y', 'z', 'w'), sample.orientation)
            ),
        }

    @staticmethod
    def _point_message(data: Dict[str, Any]) -> PatrolPoint:
        message = PatrolPoint()
        message.point_id = data['point_id']
        message.task_id = data['task_id']
        message.wait_seconds = float(data['wait_seconds'])
        # 老路线文件里没有这条键，用 get 兜底成空串（= 不判定），不然
        # 一份旧 json 就能让整个节点在发布时报 KeyError。
        message.expect = data.get('expect', '')
        pose = data['pose']
        message.pose.header.frame_id = pose['header']['frame_id']
        # A patrol point is a spatial target, not a historical sensor sample.
        # Replaying the recording timestamp makes Nav2 request a TF transform
        # from the time at which the point was recorded, which may be hours
        # old.  Zero asks Nav2/TF for the latest available transform.
        message.pose.header.stamp.sec = 0
        message.pose.header.stamp.nanosec = 0
        for field in ('x', 'y', 'z'):
            setattr(
                message.pose.pose.position, field,
                float(pose['position'][field])
            )
        for field in ('x', 'y', 'z', 'w'):
            setattr(
                message.pose.pose.orientation, field,
                float(pose['orientation'][field])
            )
        return message

    def _route_message(self, data: Dict[str, Any]) -> PatrolRoute:
        message = PatrolRoute()
        message.name = data['name']
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = data['header']['frame_id']
        message.points = [
            self._point_message(point) for point in data['points']
        ]
        return message

    def _publish_active_route(self) -> None:
        active_name = self._store.active_route
        route = self._store.get_route(active_name) if active_name else None
        if route is None:
            message = PatrolRoute()
            message.header.stamp = self.get_clock().now().to_msg()
        else:
            message = self._route_message(route)
        self._route_publisher.publish(message)

    def _record_point(self, request, response):
        try:
            sample = self._samples.fresh_average(
                time.monotonic(), self._pose_max_age
            )
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response
        if sample is None:
            # AMCL only publishes after a filter update, so a stationary robot
            # can have a valid localization without a recent /amcl_pose message.
            # map -> base_link combines AMCL's map -> odom correction with the
            # latest odometry and is therefore the correct current fallback.
            sample, tf_error = self._lookup_tf_pose()
            if sample is None:
                response.success = False
                response.message = (
                    'No current robot pose: /amcl_pose is older than %.1fs '
                    'or unavailable, and TF %s -> %s is unavailable: %s' % (
                        self._pose_max_age,
                        self._global_frame,
                        self._robot_base_frame,
                        tf_error,
                    )
                )
                return response
        try:
            point = self._store.record_point(
                request.route_name, request.point_id, request.task_id,
                request.wait_seconds, self._pose_dict(sample),
                expect=request.expect
            )
            response.point = self._point_message(point)
            response.success = True
            response.message = 'Patrol point recorded'
            if self._store.active_route == request.route_name.strip():
                self._publish_active_route()
        except (OSError, TypeError, ValueError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _get_route(self, request, response):
        name = request.route_name.strip() or self._store.active_route
        route = self._store.get_route(name) if name else None
        if route is None:
            response.success = False
            response.message = 'Route does not exist'
            return response
        response.route = self._route_message(route)
        response.success = True
        response.message = 'Route loaded'
        return response

    def _set_active(self, request, response):
        try:
            self._store.set_active(request.route_name)
            self._publish_active_route()
            response.success = True
            response.message = 'Active route updated'
        except (OSError, ValueError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _clear_route(self, request, response):
        name = request.route_name.strip() or self._store.active_route
        try:
            if not name or not self._store.clear_route(name):
                response.success = False
                response.message = 'Route does not exist'
                return response
            self._publish_active_route()
            response.success = True
            response.message = 'Route cleared'
        except OSError as error:
            response.success = False
            response.message = str(error)
        return response


    def _delete_point(self, request, response):
        try:
            self._store.delete_point(request.route_name, request.point_id)
            # 删掉的可能是"当前激活路线"里的点，active_route 也许被清空
            self._publish_active_route()
            route = self._store.get_route(request.route_name)
            response.remaining = len(route['points']) if route else 0
            response.success = True
            response.message = ('Point deleted (%d left)'
                                % response.remaining)
        except (OSError, ValueError) as error:
            response.success = False
            response.message = str(error)
        return response


def main(args=None) -> None:
    """Run the patrol route manager node."""
    rclpy.init(args=args)
    node = PatrolRouteManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


def migrate_legacy_path(path):
    """改名兼容：老机器上还叫 fishbot_*.json，首次运行时原地复制一份新名字。"""
    import os as _os
    path = _os.path.expanduser(str(path))
    name = _os.path.basename(path)
    legacy = name.replace('fishbot_', 'fishbot_')
    if legacy == name:
        return path
    legacy_path = _os.path.join(_os.path.dirname(path), legacy)
    if not _os.path.exists(path) and _os.path.exists(legacy_path):
        try:
            import shutil
            shutil.copy2(legacy_path, path)
        except OSError:
            return legacy_path
    return path
