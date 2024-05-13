import rclpy
from rclpy.node import Node 
from bumperbot_msgs.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__("simple_service_client")

        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        self.req_ = AddTwoInts.Request()
        self.req_.a = a
        self.req_.b = b

        self.future_ = self.client_.call_async(self.req_)
        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future_):
        self.get_logger().info("Service response %d" % future_.result().sum)


def main():

    rclpy.init()

    if len(sys.argv) != 3:
        print("Wrong number of arguments, please try again with A and B arguments.")
        return -1
    
    add_two_ints = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(add_two_ints)
    add_two_ints.destroy_node()
    rclpy.shutdown()

if __name__  == "main":
    main()


