import rclpy
from rclpy.node import Node
import math
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray

class FollowBreadcrumbsNode(Node):
    def __init__(self):
        super().__init__('follow_breadcrumbs_node')

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

        self.FINAL_TAG_ID = 20
        self.STOP_DISTANCE = 0.5
        self.IMAGE_WIDTH_CENTER = 320.0

        #controller gains
        self.KP_ANGULAR = 0.003
        self.KP_LINEAR = 0.3

        self.visible_tags = {}

        self.last_detection_time = self.get_clock().now()
        self.TIMEOUT_THRESHOLD = 0.6

        self.current_cmd = Twist()

        self.timer = self.create_timer(0.1, self.control_loop)

    def detections_callback(self, msg):
        if len(msg.detections) > 0: 
            self.last_detection_time = self.get_clock().now()

        for tag in msg.detections:
            tag_id = tag.id
            center_x = float(tag.centre.x)

            if tag_id not in self.visible_tags:
                self.visible_tags[tag_id] = {}
            self.visible_tags[tag_id]['center_x'] = center_x

    def tf_callback(self, msg):
        for tag in msg.transforms:
            try:
                tag_id = int(tag.child_frame_id.split(":")[1])
            except (IndexError, ValueError):
                continue

            z_dist = tag.transform.translation.z

            if tag_id not in self.visible_tags:
                self.visible_tags[tag_id] = {}
            self.visible_tags[tag_id]['z_dist'] = z_dist
            self.last_detection_time = self.get_clock().now()

    def control_loop(self):
        cmd = Twist()
        now = self.get_clock().now()
        time_since_last_detection = (now - self.last_detection_time).nanoseconds / 1e9

        valid_ids = [
            tag_id for tag_id, data in self.visible_tags.items()
            if 'center_x' in data and 'z_dist' in data
        ]

        if valid_ids:
            target_id = max(valid_ids)
            target_data = self.visible_tags[target_id]

            x_pixel = target_data['center_x']
            z_dist = target_data['z_dist']

            #calculate horizontal pixel error from center of image

            pixel_error = self.IMAGE_WIDTH_CENTER - x_pixel

            if target_id == self.FINAL_TAG_ID and z_dist < self.STOP_DISTANCE:
                self.get_logger().info(f"Reached Final AprilTag ({self.FINAL_TAG_ID})! Stopping.")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                cmd.angular.z = self.KP_ANGULAR * pixel_error

                if z_dist > self.STOP_DISTANCE:
                    distance_error = z_dist - self.STOP_DISTANCE
                    cmd.linear.x = min(0.25, self.KP_LINEAR * distance_error)
                else:
                    cmd.linear.x = 0.0

            self.current_cmd = cmd
            self.cmd_pub.publish(cmd)
        else:
            if time_since_last_detection < self.TIMEOUT_THRESHOLD:
                self.cmd_pub.publish(self.current_cmd)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)

        self.visible_tags.clear()

def main(args=None):
    rclpy.init(args=args)
    node = FollowBreadcrumbsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()