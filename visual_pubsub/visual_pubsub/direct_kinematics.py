import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class ForwardKinematics(Node):

    def __init__(self):
        super().__init__('forward_kinematics')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.update_joints)  # Actualiza cada 0.1s

        # Longitudes de los eslabones
        self.l1 = 0.9  # Longitud del primer eslabón
        self.l2 = 0.6  # Longitud del segundo eslabón
        self.l3 = 0.5  # Longitud del tercer eslabón
        self.l4 = 0.4  # Longitud del cuarto eslabón

        # Ángulos de las articulaciones (iniciales)
        self.q = np.array([0.5, -0.5, 0.3, 0.2])  # [q1, q2, q3, q4]
        self.target_pos = np.array([1.2, 0.5, 0.0])  # Objetivo por defecto

        # Publicar estados de las articulaciones
        self.step_size = 0.05

    def forward_kinematics(self, q):
        # Desempaquetar las articulaciones
        q1, q2, q3, q4 = q

        # Calcular la posición (x, y, z) del extremo del brazo en 3D
        x = self.l4 * np.sin(q1) * np.sin(q4) + self.l3 * np.cos(q1) * np.cos(q2) * np.cos(q4) + self.l2 * np.cos(q1) * np.cos(q2) + self.l1 * np.cos(q1)
        y = self.l4 * np.sin(q1) * np.cos(q2) * np.cos(q4) + self.l2 * np.sin(q1) * np.cos(q2) + self.l3 * np.sin(q1) * np.sin(q2) * np.cos(q4)
        z = self.l4 * np.sin(q2) * np.cos(q4) + self.l3 * np.sin(q2) + self.l2 * np.sin(q2) + self.l1

        return np.array([x, y, z])

    def update_joints(self):
        current_pos = self.forward_kinematics(self.q)  # Posición calculada con cinemática directa
        self.get_logger().info(f'Current position: {current_pos}')

        # Publicar los estados de las articulaciones (q1, q2, q3, q4)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['q1', 'q2', 'q3', 'q4']
        msg.position = self.q.tolist()
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematics()
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()


if __name__ == '__main__':
    main()
