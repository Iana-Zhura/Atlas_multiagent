import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
import os
from subprocess import call
from launch_ros.substitutions import FindPackageShare



class Call_Atlas(Node):

    def __init__(self):
        self.exit_code = 1
        super().__init__('call_atlas')
        self.subscription = self.create_subscription(
            Bool,
            'dataset_collected',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.pub_render = self.create_publisher(Bool,'render_status', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    

    def listener_callback(self, msg):
     
        path_dir = os.getcwd()
        self.get_logger().info('Rendering starts')
        # pkg_dir = FindPackageShare(package='render').find('render')
        # bash_dir = os.path.join(pkg_dir, 'scripts','inference.sh')
        if msg.data == True:
            os.chmod(os.path.join(path_dir, "src/3D_reconstruction/render/scripts", "inference.sh"), 0o755)
            self.exit_code = call(os.path.join(path_dir, "src/3D_reconstruction/render/scripts", "inference.sh"), shell=True)
            print(self.exit_code)
     
    def timer_callback(self):
        
        msg = Bool()
        msg.data = True
       
        if self.exit_code == 0 : 
            self.pub_render.publish(msg)



def main(args=None):
    print('Hi from Atlas.')
    rclpy.init(args=args)

    call_atlas = Call_Atlas()
    rclpy.spin(call_atlas)

    call_atlas.destroy_node()
    rclpy.shutdown()


    

if __name__ == '__main__':
    main()
