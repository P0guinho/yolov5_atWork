import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class CleanVirtWall(Node):
    def __init__(self):
        super().__init__('clean_virtwall')

        self.srv_cleanVirtWall = self.create_service(Empty, 'virtWall_cleanner', self.srvCallback)

    def srvCallback(self, request, response):
        
        self.get_logger().info('Limpando Detecções')

        file_path = '/home/juliogsabka/atWork_ws/saida.txt'
        with open(file_path, 'w') as arquivo:
            self.get_logger().info('Detecções Limpas')

        return response
    
def main(args=None):
    rclpy.init(args=args)

    clean_virtwall = CleanVirtWall()

    rclpy.spin(clean_virtwall)

    clean_virtwall.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
