import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan

class CMDcontrol(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)  
        self.sub = self.create_subscription(LaserScan, "base_scan", self.scan_callback, 10)

        self.obstacle_detected = False
        self.toggle_direction = True  # Alterna giro: True = derecha, False = izquierda
        self.timer = self.create_timer(0.5, self.timer_callback)

    def scan_callback(self, msg):
        # Obtenemos el rango frontal (unos 20 grados al frente)
        front_angles = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
        front_distances = [d for d in front_angles if d > 0.0]

        # Si hay un obstáculo a menos de 1 metro
        if front_distances and min(front_distances) < 1.4:
            if not self.obstacle_detected:
                # Solo cambia la dirección si es un nuevo obstáculo
                self.toggle_direction = not self.toggle_direction
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def timer_callback(self):
        msg = Twist()
        if self.obstacle_detected:
            msg.linear.x = 0.0
            msg.angular.z = -0.7 if self.toggle_direction else 0.7
        else:
            msg.linear.x = 0.5
            msg.angular.z = 0.0

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CMDcontrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "_main_":
    main()