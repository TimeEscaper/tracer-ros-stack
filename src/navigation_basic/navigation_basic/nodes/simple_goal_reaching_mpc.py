import math
import rclpy
import do_mpc
import casadi
from typing import Tuple
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from navigation_basic.controllers import DoMPCControllerGoalReaching


_NODE_NAME = "simple_goal_reaching_mpc"


class _MPCNode(Node):
    
    def __init__(self, 
                 node_name: str,
                 odom_topic: str,
                 cmd_topic: int,
                 goal: Tuple[float, float],
                 control_dt: float = 0.1,
                 control_horizon: int = 20,
                 linear_vel_bounds: Tuple[float, float] = (0., 0.3),
                 angular_vel_bounds: Tuple[float, float] = (0., math.pi / 10),
                 publish_dt: float = 0.05,
                 odom_topic_queue: int = 10,
                 cmd_topic_queue: int = 10):
        super().__init__(node_name)
        self._current_control = (0., 0.)
        self._current_pose = (0., 0., 0.)
        self._goal = goal
        
        self._controller = DoMPCControllerGoalReaching(dt=control_dt,
                                                       horizon=control_horizon,
                                                       u_linear_bound=linear_vel_bounds,
                                                       u_angular_bound=angular_vel_bounds)
        
        self._control_timer = self.create_timer(control_dt, self._control_callback)
        self._publish_timer = self.create_timer(publish_dt, self._publish_cmd_callback)
        
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, cmd_topic_queue)
        
        self._odom_sub = self.create_subscription(Odometry,
                                                  odom_topic,
                                                  self._odom_callback,
                                                  odom_topic_queue)
        
    def _odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = _MPCNode._extract_yaw(msg)
        self._current_pose = (x, y, theta)
        
    def _publish_cmd_callback(self):
        control = self._current_control
        msg = Twist()
        msg.linear.x = control[0]
        msg.angular.z = control[1]
        self._cmd_pub.publish(msg)

    def _control_callback(self):
        state = self._current_pose
        control = self._controller.predict(state, self._goal)
        print(control)
        self._current_control = control

    @staticmethod
    def _extract_yaw(msg: Odometry) -> float:
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        t3 = 2. * (w * z + x * y)
        t4 = 1. - 2. * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return yaw
        
def main(args=None):
    rclpy.init(args=args)
    
    node = _MPCNode(node_name=_NODE_NAME,
                    odom_topic="/odom",
                    cmd_topic="/diff_drive_base_controller/cmd_vel_unstamped",
                    goal=(2., 2.))
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
