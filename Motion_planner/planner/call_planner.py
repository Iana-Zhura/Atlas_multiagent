import rclpy
from rclpy.node import Node
import os
from std_msgs.msg import Bool
from subprocess import call

class CallPlanner(Node):
    

    def __init__(self):
        self.is_heard = False
        super().__init__('call_planner')
        self.subscription = self.create_subscription(
            Bool,
            'render_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        
        self.get_logger().info('I heard: "%d"' % msg.data)
        path_dir = os.getcwd()
        if msg.data == True and self.is_heard == False:
        # if msg.data == True:
            print("Hi from planner")
            os.chmod(os.path.join(path_dir, "src/Motion_planner/planner/scripts", "planner.sh"), 0o755)
            self.exit_code = call(os.path.join(path_dir, "src/Motion_planner/planner/scripts", "planner.sh"), shell=True)
            print(self.exit_code)
            self.is_heard = True


def main(args=None):
    rclpy.init(args=args)

    call_planner = CallPlanner()

    rclpy.spin(call_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    CallPlanner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()