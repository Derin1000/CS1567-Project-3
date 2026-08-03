import rclpy
from rclpy.node import Node
import math
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray

class SoccerPlayerNode(Node):
    def __init__(self):
        super().__init__('soccer_player_node')

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

        self.LEFT_GOAL_ID = 117
        self.RIGHT_GOAL_ID = 118
        self.BALL_ID = 119

        self.tag_positions = {}

        self.state = 'SCAN'

        #state variables
        self.setup_x = 0.0
        self.setup_y = 0.0
        self.kick_target = 0.0
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

        def detections_callback(self, msg):
            if len(msg.detections) > 0:
                self.last_detection_time = self.get_clock().now()

        def control_loop(self):
            cmd = Twist()

            if self.state == 'SCAN':
                has_left = self.LEFT_GOAL_ID in self.tag_positions
                has_right = self.RIGHT_GOAL_ID in self.tag_positions
                has_ball = self.BALL_ID in self.tag_positions

                if has_left and has_right and has_ball:
                    self.get_logger().info("All 3 tags detected! Computing kick setup position...")
                    self.calculate_setup_position()
                    self.state = 'ALIGN_SETUP'
                else:
                    cmd.angular.z = 0.25 # keep spinning slowly to find all tags

            elif self.state == 'ALIGN_SETUP':
                angle_error = math.atan2(self.setup_x, self.setup_z) 

                if abs(angle_error) > 0.08:
                    self.state = 'DRIVE_TO_SETUP'

                else:
                    cmd.angular.z = -1.2 * angle_error

            elif self.state == 'DRIVE_TO_SETUP':
                # need to finish

                if distance < 0.15:
                    self.get_logger().info("In position behind ball. Rotating toward goal center...")
                    self.state = 'ALIGN_TO_GOAL'
                else: 
                    cmd.linear.x = min(0.2, 0.4 * distance)
                    cmd.angular.z = -1.0 * angle_error

            elif self.state == 'ALIGN_TO_GOAL':
                if self.BALL_ID in self.tag_positions:
                    ball_x, _, _ = self.tag_positions[self.BALL_ID]
                    angle_to_ball = math.atan2(ball_x, 1.0)

                    if abs(angle_to_ball) < 0.05:
                        self.get_logger().info("Aligned to goal! Kicking!")
                        self.state = 'KICK'
                        self.kick_start_time = self.get_clock().now()
                    else:
                        cmd.angular.z = -1.0 * angle_to_ball
                else:
                    cmd.angular.z = 0.1 # keep spinning to find the ball

            elif self.state == 'KICK':
                elapsed = (self.get_clock().now() - self.kick_start_time).nanoseconds / 1e9
                if elapsed < 3.0:
                    cmd.linear.x = 0.35 # kick forward

                else:
                    self.get_logger().info("Goal scored! Stopping...")
                    self.state = 'DONE'

            elif self.state == 'DONE':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

                self.cmd_pub.publish(cmd)

            def calculate_setup_position(self):
                #need to finish

        def main(args=None):
            rclpy.init(args=args)
            node = SoccerPlayerNode()
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