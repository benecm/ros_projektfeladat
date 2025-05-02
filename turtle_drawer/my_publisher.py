import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen

class RDrawer(Node):
    def __init__(self):
        super().__init__('r_drawer')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Várakozás a set_pen szolgáltatásra...')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Várakozás a teleport_absolute szolgáltatásra...')

    def call_teleport(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)
        future = self.teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def call_set_pen(self, r, g, b, width, off):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        future = self.pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def draw_R(self, x_start):
        self.call_set_pen(255, 0, 0, 3, 1)
        self.call_teleport(x_start, 2.0, 0.0)
        self.call_set_pen(255, 0, 0, 3, 0)
        self.call_teleport(x_start, 7.0, 0.0)
        self.call_teleport(x_start + 1.5, 7.0, 0.0)
        self.call_teleport(x_start + 1.5, 5.0, 0.0)
        self.call_teleport(x_start, 5.0, 0.0)
        self.call_teleport(x_start + 1.5, 2.0, 0.0)  # Folyamatos vonal

    def draw_O(self, x_start):
        self.call_set_pen(0, 255, 0, 3, 1)
        self.call_teleport(x_start, 2.0, 0.0)
        self.call_set_pen(0, 255, 0, 3, 0)
        self.call_teleport(x_start, 7.0, 0.0)
        self.call_teleport(x_start + 1.5, 7.0, 0.0)
        self.call_teleport(x_start + 1.5, 2.0, 0.0)
        self.call_teleport(x_start, 2.0, 0.0)

    def draw_B(self, x_start):
        self.call_set_pen(0, 0, 255, 3, 1)
        self.call_teleport(x_start, 2.0, 0.0)
        self.call_set_pen(0, 0, 255, 3, 0)
        # Felső rész
        self.call_teleport(x_start, 7.0, 0.0)
        self.call_teleport(x_start + 1.5, 7.0, 0.0)
        self.call_teleport(x_start + 1.5, 5.0, 0.0)
        self.call_teleport(x_start, 5.0, 0.0)
        # Alsó rész (nincs toll felemelés)
        self.call_teleport(x_start + 1.5, 5.0, 0.0)
        self.call_teleport(x_start + 1.5, 2.0, 0.0)
        self.call_teleport(x_start, 2.0, 0.0)

    def draw_T(self, x_start):
        self.call_set_pen(255, 165, 0, 3, 1)
        self.call_teleport(x_start + 0.75, 7.0, 0.0)  # Középre igazítás
        self.call_set_pen(255, 165, 0, 3, 0)
        self.call_teleport(x_start, 7.0, 0.0)
        self.call_teleport(x_start + 1.5, 7.0, 0.0)
        self.call_teleport(x_start + 0.75, 7.0, 0.0)
        self.call_teleport(x_start + 0.75, 2.0, 0.0)

    def draw_word_ROBOT(self):
        self.draw_R(1.0)    # 1.0-2.5
        self.draw_O(3.0)    # 3.0-4.5 (0.5 rés)
        self.draw_B(5.0)    # 5.0-6.5
        self.draw_O(7.0)    # 7.0-8.5
        self.draw_T(9.0)    # 9.0-10.5 (teljesen látható)
        self.get_logger().info('ROBOT szó kirajzolva!')

def main(args=None):
    rclpy.init(args=args)
    node = RDrawer()
    node.draw_word_ROBOT()
    rclpy.spin_once(node, timeout_sec=0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
