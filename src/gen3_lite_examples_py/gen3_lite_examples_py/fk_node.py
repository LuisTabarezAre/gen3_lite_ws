import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from gen3_lite_examples_py.forward_kinematics import fk_gen3

class Gen3ForwardKinematics(Node):

    def __init__(self):
        super().__init__('forward_kinematics')

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
        self.pose_pub = self.create_publisher(PoseStamped,'/fk/pose',10)

        self.js_sub = self.create_subscription(JointState,'/joint_states',
            self.joint_state_callback, 10)
        
        # -------------------------------
        # Internal state
        # -------------------------------
        self._last_warn_time = self.get_clock().now()

        self.get_logger().info(
            f'Gen3 FK node started with joints: {self.joint_names}'
        )

    # =========================================================
    # JointState callback
    # =========================================================
    def joint_state_callback(self, msg: JointState):
        """
        Compute FK whenever a complete JointState is received
        """

        # Build name -> position map
        joint_map = dict(zip(msg.name, msg.position))

        # Check if all joints are available
        if not all(j in joint_map for j in self.joint_names):
            now = self.get_clock().now()
            if (now - self._last_warn_time).nanoseconds > 2e9:
                missing = set(self.joint_names) - set(joint_map.keys())
                self.get_logger().warn(
                    f'Waiting for joints: {sorted(missing)}'
                )
                self._last_warn_time = now
            return

        # Ordered joint vector
        q = np.array([joint_map[j] for j in self.joint_names])

        # -------------------------------
        # Forward kinematics
        # -------------------------------
        pos, R_mat = fk_gen3(q, self.params)

        quat = R.from_matrix(R_mat).as_quat()  # [x, y, z, w]

        # -------------------------------
        # Publish PoseStamped
        # -------------------------------
        msg_out = PoseStamped()
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = self.base_frame

        msg_out.pose.position.x = float(pos[0])
        msg_out.pose.position.y = float(pos[1])
        msg_out.pose.position.z = float(pos[2])

        msg_out.pose.orientation.x = float(quat[0])
        msg_out.pose.orientation.y = float(quat[1])
        msg_out.pose.orientation.z = float(quat[2])
        msg_out.pose.orientation.w = float(quat[3])

        self.pose_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = Gen3ForwardKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
