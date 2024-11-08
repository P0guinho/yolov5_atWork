import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')

        self.virtSrv = self.create_client(Empty, 'virtWall_cleanner')
        self.req = Empty.Request()

        self.main()

    def main(self):

        self.virtSrv.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal = MinimalClientAsync()

    rclpy.spin(minimal)

    minimal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()