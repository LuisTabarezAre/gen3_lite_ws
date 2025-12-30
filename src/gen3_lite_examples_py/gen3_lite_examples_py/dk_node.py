import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from scipy.spatial.transform import Rotation as R

from gen3_lite_examples_py.differential_kinematics import dk_gen3

class Gen3DifferentialKinematics(Node):

    def __init__(self):
        super().__init__('differential_kinematics')

        # -------------------------------
        # Parameters (robot geometry)
        # -------------------------------
        self.declare_parameter('a2', 0.0)
        self.declare_parameter('d2', 0.0)
        self.declare_parameter('d3', 0.0)
        self.declare_parameter('d4', 0.0)
        self.declare_parameter('d5', 0.0)
        self.declare_parameter('d6', 0.0)
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter(
            'joint_names',
            [
                'joint_1',
                'joint_2',
                'joint_3',
                'joint_4',
                'joint_5',
                'joint_6'
            ]
        )

        # -------------------------------
        # Load parameters
        # -------------------------------
        self.joint_names = list(self.get_parameter('joint_names').value)

        self.params = {
            'a2': float(self.get_parameter('a2').value),
            'd2': float(self.get_parameter('d2').value),
            'd3': float(self.get_parameter('d3').value),
            'd4': float(self.get_parameter('d4').value),
            'd5': float(self.get_parameter('d5').value),
            'd6': float(self.get_parameter('d6').value),
        }

        self.base_frame = self.get_parameter('base_frame').value

        # -------------------------------
        # ROS interfaces
        # -------------------------------
        self.pose_pub = self.create_publisher(TwistStamped,'/dk/pose',10)

        self.js_sub = self.create_subscription(JointState,'/joint_states',
            self.joint_state_callback,
            10
        )
        # -------------------------------
        # Internal state
        # -------------------------------
        self._last_warn_time = self.get_clock().now()

        self.get_logger().info(
            f'Gen3 DK node started with joints: {self.joint_names}'
        )

    # =========================================================
    # JointState callback
    # =========================================================
    def joint_state_callback(self, msg: JointState):
        """
        Compute DK whenever a complete JointState is received
        """

        # Build name -> position map
        pos_map = dict(zip(msg.name, msg.position))
        vel_map = dict(zip(msg.name, msg.velocity))
        # Check if all joints are available
        if not all(j in pos_map for j in self.joint_names):
            now = self.get_clock().now()
            if (now - self._last_warn_time).nanoseconds > 2e9:
                missing = set(self.joint_names) - set(pos_map.keys())
                self.get_logger().warn(f'Waiting for joints: {sorted(missing)}')
                self._last_warn_time = now
            return

        # Ordered joint vector
        q  = np.array([pos_map[j] for j in self.joint_names])
        dq = np.array([vel_map[j] for j in self.joint_names])

        # -------------------------------
        # Differential kinematics
        # -------------------------------
        dp, w = dk_gen3(q,dq,  self.params)

        # -------------------------------
        # Publish TwistStamped
        # -------------------------------
        msg_out = TwistStamped()
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = self.base_frame

        msg_out.twist.linear.x = float(dp[0])
        msg_out.twist.linear.y = float(dp[1])
        msg_out.twist.linear.z = float(dp[2])

        msg_out.twist.angular.x = float(w[0])
        msg_out.twist.angular.y = float(w[1])
        msg_out.twist.angular.z = float(w[2])

        self.pose_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = Gen3DifferentialKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
