string point_id
geometry_msgs/PoseStamped pose
string task_id
float32 wait_seconds
# 期望值：这个点位"应该"有没有目标。
# ""        = 不判定（老路线默认值，行为与以前完全一致）
# "present" = 应该有灭火器
# "absent"  = 不应该有（空点位，用来量误报）
# 执行器拿它和实际检测结果比对，自动算出 tp/fp/fn/tn 写进报告。
string expect

