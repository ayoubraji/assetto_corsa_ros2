import rclpy
from rclpy.node import Node

from deep_orange_msgs.msg import CtReport
from ac_actuation_msgs.msg import BrakeCmd, AcceleratorPedalCmd, SteeringCmd
from std_msgs.msg import UInt8

class TestControlPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_acc = self.create_publisher(AcceleratorPedalCmd, "accelerator_cmd", 10) 
        self.publisher_steer = self.create_publisher(SteeringCmd, "steering_cmd", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.callback)   
       
    def callback(self):
        msg = AcceleratorPedalCmd()
        msg.pedal_cmd = 300.0   
        self.publisher_acc.publish(msg)
        self.get_logger().info('Publishing control')
        '''

        msg = SteeringCmd()
        msg.angle_cmd = 0.0
        self.publisher_steer.publish(msg)'''
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TestControlPublisher()

    rclpy.spin(minimal_publisher)

   
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()