"""巡检 v2：可恢复、可解释、与 ROS 解耦的巡检执行器。

对外接口与 ``mybot_patrol_executor``（v1）保持一致，可以直接替换：

- action  ``/execute_patrol``
- 服务    ``/patrol/pause``、``/patrol/resume``
- 话题    ``/patrol/task/request``、``/patrol/task/result``、
          ``/patrol/preempt``、``/patrol/execution/status``
"""

__version__ = '0.2.0'
