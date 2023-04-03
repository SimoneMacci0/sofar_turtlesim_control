from rclpy.node import Node
import rclpy
import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleController(Node):

    def __init__(self):
        super().__init__("turtle_control_node")

        # Variables
        self.turtle_pose = None
        self.goal_x = 0
        self.goal_y = 0
        self.threshold = 0.1

        # Control gains
        self.Kl = 0.5
        self.Ka = 5.0

        # Pub and sub
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.on_pose_received, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
    
    def on_pose_received(self, msg: Pose):
        self.turtle_pose = msg

    def set_goal(self, x: float, y: float):
        self.goal_x = x
        self.goal_y = y
        self.control_loop_timer = self.create_timer(0.1, self.on_control_loop)

    def on_control_loop(self):
        # compute distance error
        e_d = math.sqrt((self.turtle_pose.x - self.goal_x)**2 + (self.turtle_pose.y - self.goal_y)**2)
        # compute angular error
        e_a = math.atan2(self.goal_y - self.turtle_pose.y, self.goal_x - self.turtle_pose.x) - self.turtle_pose.theta

        # exit loop if goal reached
        if e_d <= self.threshold:
            # cancel timer
            self.control_loop_timer.cancel()
            self.get_logger().info("Goal Reached!")
            self.ask_user_for_goal()
            # No memory leakage: after asking user for new input, a new timer object will be instantiated, while the current one will 
            # consistently exit its execution with the return command 
            return
           
        # Compute control inputs
        lin_control = self.Kl * e_d
        ang_control = self.Ka * e_a

        # Build twist msg
        cmd_vel = Twist()
        cmd_vel.linear.x = lin_control
        cmd_vel.angular.z = ang_control

        # Publish it
        self.cmd_vel_pub.publish(cmd_vel)

    def ask_user_for_goal(self):
        print("------")
        print("Input goal coordinates or press CTRL-c to exit: ")
        x = float(input("X: "))
        y = float(input("Y: "))
        self.set_goal(x, y)

def main():
    rclpy.init(args = None)

    turtle_controller = TurtleController()
    turtle_controller.ask_user_for_goal()

    rclpy.spin(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
