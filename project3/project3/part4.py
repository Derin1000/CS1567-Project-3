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
        self.PAUSE_DURATION = 1.5   

        self.visible_tags = {}
        self.last_detection_time = self.get_clock().now()

        #target track state
        self.current_target_id = None
        self.is_pausing = False
        self.pause_start_time = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def tf_callback(self, msg):
        for tag in msg.transforms:
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

        #handle pausing state
        if self.is_pausing:
            elapsed_pause = (now - self.pause_start_time).nanoseconds / 1e9
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            if elapsed_pause >= self.PAUSE_DURATION:
                self.is_pausing = False
                self.get_logger().info("Pause complete. Resuming navigation...")
            return
        
        if self.visible_tags:
            if self.current_target_id is None:
                self.current_target_id = min(self.visible_tags.keys())
            else:
                valid_targets = [tag_id for tag_id in self.visible_tags.keys() if tag_id >= self.current_target_id]
                if valid_targets:
                    self.current_target_id = min(valid_targets)

            if self.current_target_id in self.visible_tags:
                x, y, z = self.visible_tags[self.current_target_id]
                angle_error = math.atan2(x, z)
                distance = math.hypot(x, z)

                #allows robot to stop at a certain distance from the tag
                if distance <= self.STOP_DISTANCE:
                    self.get_logger().info(f"Reached tag {self.current_target_id}. stopping...")
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                    if self.current_target_id == self.FINAL_TAG_ID:
                        self.get_logger().info("Final tag reached. Stopping navigation.")
                        self.timer.cancel()
                    else:
                        self.is_pausing = True
                        self.pause_start_time = now
                        self.current_target_id += 1
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