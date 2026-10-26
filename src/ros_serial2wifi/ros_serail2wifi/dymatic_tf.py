import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


# 3.自定义节点类；
class Tf2node(Node):
    def __init__(self):
        super().__init__("tf2node_node_py")
        self.sub =self.create_subscription(Odometry,"/odom",self.odom_callback, 10)
        self.br = TransformBroadcaster(self)

    def odom_callback(self,msg):
        tf =  TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = msg.pose.pose. orientation
        self.br.sendTransform(tf)

def main():
    # 2.初始化ROS2客户端；
    rclpy.init()
    # 4.调用spain函数，并传入节点对象；
    rclpy.spin(Tf2node())
    # 5.资源释放。 
    rclpy.shutdown()

if __name__ == '__main__':
    main()