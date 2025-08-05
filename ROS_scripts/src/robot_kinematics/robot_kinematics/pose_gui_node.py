import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import tf_transformations
from tkinter import Tk, Scale, HORIZONTAL, Label
import threading
from .calculations import angle_conversion

class PoseGUI(Node):
    def __init__(self):
        super().__init__('pose_gui_node')
        # This node will publish the target pose to the target_pose_marker by fetching the data every 0.1 seconds from a custom Python GUI
        self.publisher = self.create_publisher(Marker, 'target_pose_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)  # 10 Hz

        # Default pose
        self.pose = {
            'x': 0.4,
            'y': 0.0,
            'z': 0.3,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }

        # Start the python gui in another thread
        threading.Thread(target=self.run_gui, daemon=True).start()

    def run_gui(self):
        # Basic GUI interface using sliders
        root = Tk()
        root.title("Target Pose Sliders")

        sliders = {}
        for i, (label, default) in enumerate(self.pose.items()):
            Label(root, text=label).pack()

            # Bound the sliders appropriately
            slider = Scale(root, from_=-180 if 'roll' in label or 'yaw' in label or 'pitch' in label else -1.0,
                           to=180 if 'roll' in label or 'yaw' in label or 'pitch' in label else 1.0,
                           resolution=1 if 'roll' in label or 'yaw' in label or 'pitch' in label else 0.01,
                           orient=HORIZONTAL, length=300)
            slider.set(default)
            slider.pack()
            sliders[label] = slider

        # Function that updates the self.pose variable by fetching data from the GUI
        def update_pose():
            for k in self.pose:
                self.pose[k] = sliders[k].get()
            root.after(100, update_pose)

        update_pose()
        root.mainloop()

    def publish_marker(self):
        # Extract pose, format to radians and convert to quaternions
        x = self.pose['x']
        y = self.pose['y']
        z = self.pose['z']
        r,p,y_ = angle_conversion.deg_to_rad([self.pose['roll'], self.pose['pitch'], self.pose['yaw']])

        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(r, p, y_)

        # Marker cube is defined with respect to world frame
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_pose"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set position and orientation (later is via quaternions) of the marker and render this object
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish the marker data to the target_pose_marker for ik_publisher to access
        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PoseGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
