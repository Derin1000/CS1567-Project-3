import rclpy
from rclpy.node import Node
import math
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray

class FollowBreadcrumbsNode(Node):
    def __init__(self):
        super().__init__('follow_crumbs_node')

        #velocity publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        
        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detections_callback,
            10)

        self.BREADCRUMB_ID = 120

        self.tag_positions = {}

        #state variables
        self.last_detection_time = self.get_clock().now()

        #main state control loop running at 10hz
        self.timer = self.create_timer(0.1, self.state_machine)

    def tf_callback(self, msg):
        for tag in msg.transforms:
            try:
                tag_id = int(tag.child_frame_id.split(':')[1])
            except (IndexError, ValueError):
                continue

            x = tag.transform.translation.x
            y = tag.transform.translation.y 
            z = tag.transform.translation.z

            #storing the 3D position of each tag
            self.tag_positions[tag_id] = (x, y, z)
            self.last_detection_time = self.get_clock().now()