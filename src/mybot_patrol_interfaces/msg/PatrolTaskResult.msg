uint8 UNKNOWN=0
uint8 RUNNING=1
uint8 SUCCEEDED=2
uint8 FAILED=3
uint8 CANCELED=4

std_msgs/Header header
string execution_id
string route_name
uint32 point_index
string point_id
string task_id
uint8 status
string message
