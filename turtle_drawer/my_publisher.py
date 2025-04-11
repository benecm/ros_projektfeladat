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

        # Várunk, hogy a szolgáltatások elérhetők legyenek
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Várakozás a set_pen szolgáltatásra...')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Várakozás a teleport_absolute szolgáltatásra...')

        #self.draw_R()

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


    def draw_line(self, distance, speed=1.0):
        msg = Twist()
        msg.linear.x = speed
        duration = distance / speed
        self.publisher.publish(msg)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=duration))
        msg.linear.x = 0.0
        self.publisher.publish(msg)

    def turn(self, angle_deg, speed=1.0):
        msg = Twist()
        msg.angular.z = speed
        duration = abs(angle_deg / (speed * 180.0 / 3.1416))  # degree to radians
        self.publisher.publish(msg)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=duration))
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def draw_O(self, x_start):
        self.call_set_pen(0, 0, 255, 3, 1)
        self.call_teleport(x_start, 2.0, 0.0)

        self.call_set_pen(0, 0, 255, 3, 0)
        self.call_teleport(x_start, 7.0, 0.0)
        self.call_teleport(x_start + 2.5, 7.0, 0.0)
        self.call_teleport(x_start + 2.5, 2.0, 0.0)
        self.call_teleport(x_start, 2.0, 0.0)

    def draw_B(self, x_start):
        self.call_set_pen(0, 0, 255, 3, 1)
        self.call_teleport(x_start, 2.0, 0.0)

        self.call_set_pen(0, 0, 255, 3, 0)
        self.call_teleport(x_start, 7.0, 0.0)
        self.call_teleport(x_start + 2.0, 7.0, 0.0)
        self.call_teleport(x_start + 2.0, 5.0, 0.0)
        self.call_teleport(x_start, 5.0, 0.0)

        self.call_set_pen(0, 0, 255, 3, 1)
        self.call_teleport(x_start, 5.0, 0.0)

        self.call_set_pen(0, 0, 255, 3, 0)
        self.call_teleport(x_start + 2.0, 5.0, 0.0)
        self.call_teleport(x_start + 2.0, 2.0, 0.0)
        self.call_teleport(x_start, 2.0, 0.0)

    def draw_T(self, x_start):
        self.call_set_pen(0, 0, 255, 3, 1)
        self.call_teleport(x_start, 7.0, 0.0)

        self.call_set_pen(0, 0, 255, 3, 0)
        self.call_teleport(x_start + 2.5, 7.0, 0.0)
        self.call_teleport(x_start + 1.25, 7.0, 0.0)
        self.call_teleport(x_start + 1.25, 2.0, 0.0)

    def draw_R(self, x_start):
        self.call_set_pen(0, 0, 255, 3, 1)
        self.call_teleport(x_start, 2.0, 0.0)
        self.call_set_pen(0, 0, 255, 3, 0)
        self.call_teleport(x_start, 7.0, 0.0)
        self.call_teleport(x_start + 2.5, 7.0, 0.0)
        self.call_teleport(x_start + 2.5, 5.0, 0.0)
        self.call_teleport(x_start, 5.0, 0.0)
        self.call_set_pen(0, 0, 255, 3, 1)
        self.call_teleport(x_start, 5.0, 0.0)
        self.call_set_pen(0, 0, 255, 3, 0)
        self.call_teleport(x_start + 2.5, 2.0, 0.0)


    def draw_word_ROBOT(self):
        # R betű (start: x=2)
        self.draw_R(2.0)

        # O betű (start: x=6)
        self.draw_O(6.0)

        # B betű (start: x=10)
        self.draw_B(10.0)

        # O betű újra (start: x=14)
        self.draw_O(14.0)

        # T betű (start: x=18)
        self.draw_T(18.0)

        self.get_logger().info('ROBOT szó kirajzolva!')

def main(args=None):
    rclpy.init(args=args)
    node = RDrawer()
    node.draw_word_ROBOT()
    rclpy.spin_once(node, timeout_sec=0)  # egyszer fut le, nem loopol
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
