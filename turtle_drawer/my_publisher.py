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

        self.draw_R()

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

    def draw_R(self):
        ## Koordináták: kb. 5x5-ös R betű, kezdőpont (2,2)
        # Lépés 1: Indulási hely (nem rajzolva)
        self.call_set_pen(0, 0, 255, 3, 1)  # ne rajzoljon
        self.call_teleport(2.0, 2.0, 0.0)

        # Lépés 2: Függőleges vonal (2,2) -> (2,7)
        self.call_set_pen(0, 0, 255, 3, 0)
        self.call_teleport(2.0, 7.0, 0.0)

        # Felső vízszintes (2,7) -> (4.5,7)
        self.call_teleport(4.5, 7.0, 0.0)

        # Jobb oldali visszaív (4.5,7) -> (4.5,5)
        self.call_teleport(4.5, 5.0, 0.0)

        # Közép vízszintes (4.5,5) -> (2.0,5)
        self.call_teleport(2.0, 5.0, 0.0)

        # Diagonál láb – lekapcsoljuk a rajzolást, visszaugrás
        self.call_set_pen(0, 0, 255, 3, 1)
        self.call_teleport(2.0, 5.0, 0.0)

        # Diagonál vonal (2.0,5) -> (4.5,2)
        self.call_set_pen(0, 0, 255, 3, 0)
        self.call_teleport(4.5, 2.0, 0.0)

        self.get_logger().info('Valódi R betű kirajzolva!')


def main(args=None):
    rclpy.init(args=args)
    node = RDrawer()
    rclpy.spin_once(node, timeout_sec=0)  # egyszer fut le, nem loopol
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
