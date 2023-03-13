import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
from subprocess import call
from launch_ros.substitutions import FindPackageShare



class Call_Atlas(Node):
     
     def __init__(self):
        super().__init__('call_atlas')
        self.subscription = self.create_subscription(
            Bool,
            'render_activate',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

     def listener_callback(self, msg):
        path_dir = os.getcwd()
        self.get_logger().info('Rendering starts')
        # pkg_dir = FindPackageShare(package='render').find('render')
        # bash_dir = os.path.join(pkg_dir, 'scripts','inference.sh')
        if msg.data == True:
            os.chmod(os.path.join(path_dir, "src/render/scripts", "inference.sh"), 0o755)
            call(os.path.join(path_dir, "src/render/scripts", "inference.sh"), shell=True)





def main(args=None):
    print('Hi from Atlas.')
    rclpy.init(args=args)

    call_atlas = Call_Atlas()
    rclpy.spin(call_atlas)

    call_atlas.destroy_node()
    rclpy.shutdown()


    

if __name__ == '__main__':
    main()
