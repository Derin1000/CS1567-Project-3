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

        self.FINAL_TAG_ID = 20
        self.STOP_DISTANCE = 0.5

        self.visible_tags = {}
        self.last_detection_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.control_loop)

    def tf_callback(self, msg):
        for tag in msg.detections:
            try:
                tag_id = int(tag.child_frame_id.split(":")[1])
            except (IndexError, ValueError):
                continue

            x = tag.transform.translation.x
            y = tag.transform.translation.y
            z = tag.transform.translation.z

            self.visible_tags[tag_id] = (x, y, z)
            self.last_detection_time = self.get_clock().now()

    def control_loop(self):
        cmd = Twist()
        now = self.get_clock().now()
        time_since_last_detection = (now - self.last_detection_time).nanoseconds / 1e9

        if self.visible_tags:
            target_id = max(self.visible_tags.keys())
            x, y, z = self.visible_tags[target_id]

            angle_error = math.atan2(x,z)
            distance = math.hypot(x,z)

            if target_id == self.FINAL_TAG_ID and distance < self.STOP_DISTANCE:
                self.get_logger().info(f"Reached Final AprilTag ({self.FINAL_TAG_ID})! Stopping.")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                if distance > self.STOP_DISTANCE:
                    cmd.linear.x = min(0.2, 0.35 * (distance - self.STOP_DISTANCE))
                    cmd.angular.z = -0.8 * angle_error
                else:
                    cmd.angular.z = 0.25
        else: 
            if time_since_last_detection > 0.8:
                cmd.angular.z = 0.25

        self.cmd_pub.publish(cmd)     

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