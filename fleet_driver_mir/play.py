import sys
import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import PathRequest, ModeRequest, RobotState, FleetState, \
    Location, RobotMode

class PubClass(Node):
    def __init__(self):
        super().__init__('mode_request_generator')
        self.pub = self.create_publisher(ModeRequest,'/robot_mode_requests', 10)
        timer_period=1
        self.timer=self.create_timer(timer_period,self.timer_callback)

    def timer_callback(self):
        msg=ModeRequest()
        msg.fleet_name="mir"
        msg.robot_name="MiR_R1442"
        msg.mode.mode= int(sys.argv[1])
        self.pub.publish(msg)

def main(args=None):
    print("asking for mode request {}".format(sys.argv[1]))
    rclpy.init(args=args)
    min_pub = PubClass()
    rclpy.spin(min_pub)

    min_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()