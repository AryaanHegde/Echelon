import rclpy
from rclpy.node import Node
from rclpy import init, spin
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import time
import random
import math
import tf_transformations
from .calculations import inverse_kinematics, forward_kinematics, angle_conversion

class IKPublisher(Node):
    def __init__(self):
        super().__init__('ik_publisher')
        # This node will publish the joint angles to the robot via the joint_states topic
        self.joint_angles_pub = self.create_publisher(JointState, '/joint_states', 10)
        # This node will get the target pose by subscribing to the target_pose_marker topic
        self.pose_sub = self.create_subscription(Marker, '/target_pose_marker', self.publish_joint_states, 10)

    def publish_joint_states(self, pose_marker_data):
        # Get cartesian coordinates and degree-valued rpy angles
        x,y,z = [pose_marker_data.pose.position.x, pose_marker_data.pose.position.y, pose_marker_data.pose.position.z]
        pose_ori = pose_marker_data.pose.orientation
        r,p,y_ = angle_conversion.rad_to_deg(tf_transformations.euler_from_quaternion([pose_ori.x, pose_ori.y, pose_ori.z, pose_ori.w]))
        
        # Solve the inverse kinematics
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['world_base_joint', 'base_shoulder_joint', 'shoulder_elbow_joint', 'elbow_wrist_main_joint', 'wrist_main_wrist_secondary_joint', 'wrist_secondary_wrist_tertiary_joint']
        lengths = [0.227076, 0.381, 0.1524, 0.2286, 0.0762, 0.02921]
        solution = inverse_kinematics.solve(lengths, [x,y,z,r,p,y_])

        # Adjust angles due to RHR convention
        solution[1] *= -1
        solution[2] *= -1

        # Publish radian-valued joint angles to joint states topic
        msg.position = angle_conversion.deg_to_rad(solution)
        self.joint_angles_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()