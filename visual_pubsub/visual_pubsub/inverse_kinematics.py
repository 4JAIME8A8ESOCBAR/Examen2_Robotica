import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np
import time


class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.target_sub = self.create_subscription(Point, 'target_position',
                                                   self.target_callback, 10)

        # Initial joint angles
        self.q = np.array([-0.22, 0.7, 0.03, 0.5])  # [q1, q2, q3, q4]

        # Robot link lengths
        self.l1 = 0.9  # Longitud del primer eslabón
        self.l2 = 0.6  # Longitud del segundo eslabón
        self.l3 = 0.5  # Longitud del tercer eslabón
        self.l4 = 0.4  # Longitud del cuarto eslabón

        self.timer = self.create_timer(0.1, self.update_joints)
        self.target_pos = np.array([1.2, 0.5, 0.0])  # Default target

        # Parameters for IK
        self.step_size = 0.01
        self.max_iterations = 100
        self.tolerance = 0.01
        self.damping_factor = 0.1  # For damped least squares method

    def forward_kinematics(self, q):
        q1, q2, q3, q4 = q
        # Forward kinematics equations for 4 DOF (using DH parameters)
        x = self.l4 * np.sin(q1) * np.sin(q4) + self.l3 * np.cos(q1) * np.cos(q2) * np.cos(q4) + self.l2 * np.cos(q1) * np.cos(q2) + self.l1 * np.cos(q1)
        y = self.l4 * np.sin(q1) * np.cos(q2) * np.cos(q4) + self.l2 * np.sin(q1) * np.cos(q2) + self.l3 * np.sin(q1) * np.sin(q2) * np.cos(q4)
        z = self.l4 * np.sin(q2) * np.cos(q4) + self.l3 * np.sin(q2) + self.l2 * np.sin(q2) + self.l1
        return np.array([x, y, z])

    def jacobian(self, q):
        q1, q2, q3, q4 = q
        # Jacobian for 4 DOF
        j11 = -self.l4 * np.sin(q1) * np.cos(q2) * np.cos(q4) - self.l2 * np.sin(q1) * np.cos(q2) + self.l3 * np.sin(q1) * np.sin(q2) * np.cos(q4)
        j12 = -self.l3 * np.sin(q1) * np.cos(q2) * np.cos(q4) - self.l2 * np.sin(q1) * np.cos(q2)
        j13 = self.l3 * np.sin(q1) * np.sin(q2) * np.cos(q4) + self.l3 * np.cos(q1) * np.cos(q2)
        j14 = self.l4 * np.cos(q1) * np.sin(q2) * np.cos(q4)
        j21 = self.l4 * np.cos(q1) * np.cos(q2) * np.cos(q4) + self.l2 * np.cos(q1) * np.cos(q2) + self.l3 * np.cos(q1) * np.sin(q2) * np.cos(q4)
        j22 = -self.l3 * np.cos(q1) * np.sin(q2) * np.cos(q4) - self.l2 * np.cos(q1) * np.sin(q2)
        j23 = -self.l3 * np.cos(q1) * np.cos(q2) * np.cos(q4) + self.l3 * np.sin(q1) * np.sin(q2)
        j24 = self.l4 * np.sin(q1) * np.cos(q2) * np.cos(q4)
        j31 = 0
        j32 = self.l4 * np.cos(q2) * np.cos(q4) + self.l2 * np.cos(q2) + self.l3 * np.sin(q2) * np.cos(q4)
        j33 = -self.l3 * np.sin(q2) * np.cos(q4)
        j34 = self.l4 * np.sin(q2) * np.sin(q4)
        return np.array([[j11, j12, j13, j14], [j21, j22, j23, j24], [j31, j32, j33, j34]])

    def target_callback(self, msg):
        self.target_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(
            f"New target received: [{msg.x}, {msg.y}, {msg.z}]")

    def update_joints(self):
        current_pos = self.forward_kinematics(self.q)
        error = self.target_pos - current_pos
        error_norm = np.linalg.norm(error)

        self.get_logger().info(f"Current position: {current_pos}")
        self.get_logger().info(f"Target position: {self.target_pos}")
        self.get_logger().info(f"Error: {error_norm}")

        if error_norm > self.tolerance:
            # Compute Jacobian
            J = self.jacobian(self.q)

            # Calculate pseudo-inverse of Jacobian
            J_pseudo_inv = np.linalg.pinv(J)  # Pseudo-inverse of Jacobian
            J_dls = J_pseudo_inv @ error  # Apply update with pseudo-inverse

            # Apply update with step size
            self.q += J_dls * self.step_size

        # Publish updated joint states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['q1', 'q2', 'q3', 'q4']
        msg.position = self.q.tolist()
        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
