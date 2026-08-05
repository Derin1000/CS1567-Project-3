from turtle import distance

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

        #constants for tag IDs
        self.LEFT_GOAL_ID = 117
        self.RIGHT_GOAL_ID = 118
        self.BALL_ID = 119

        #track 3d tag locations relative to camera frame (x,y,z)
        self.tag_positions = {}

        #state machine states: SCAN, ALIGN_SETUP, DRIVE_TO_SETUP, ALIGN_TO_GOAL, KICK, DONE
        self.state = 'SCAN'

        #state variables
        self.setup_x = 0.0
        self.setup_y = 0.0
        self.kick_target_x = None
        self.last_detection_time = self.get_clock().now()

        #main state control loop running at 10hz
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
                    #self.calculate_setup_position()
                    self.state = 'ALIGN_SETUP'
                else:
                    cmd.angular.z = 0.25 # keep spinning slowly to find all tags

            elif self.state == 'ALIGN_SETUP':
                angle_error = math.atan2(self.setup_x, self.setup_z) 

                if abs(angle_error) < 0.1:
                    self.get_logger().info("Aligned to setup position! Driving to setup spot...")  # fix 1 
                    self.state = 'DRIVE_TO_SETUP'
                else:
                    cmd.angular.z = -1.0 * angle_error

            elif self.state == 'DRIVE_TO_SETUP':
                distance = math.hypot(self.setup_x, self.setup_z)
                angle_error = math.atan2(self.setup_x, self.setup_z)

                if distance < 0.2: #arrived at setup spot
                    self.get_logger().info("In position behind ball. Rotating toward goal center...")
                    self.state = 'ALIGN_TO_GOAL'
                else:
                    cmd.linear.x = min(0.2, 0.4 * distance)
                    cmd.angular.z = -0.8 * angle_error

            elif self.state == 'ALIGN_TO_GOAL':
                if has_ball:
                    ball_x, _, _ = self.tag_positions[self.BALL_ID]
                    angle_to_ball = math.atan2(ball_x, 1.0)

                    if abs(angle_to_ball) < 0.08:
                        self.get_logger().info("Aligned to goal! Kicking!")
                        self.state = 'KICK'
                        self.kick_start_time = self.get_clock().now()
                    else:
                        cmd.angular.z = -1.0 * angle_to_ball
                else:
                    cmd.angular.z = 0.15 # keep spinning to find the ball

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
            lx, _, lz = self.tag_positions[self.LEFT_GOAL_ID]
            rx, _, rz = self.tag_positions[self.RIGHT_GOAL_ID]
            bx, _, bz = self.tag_positions[self.BALL_ID]

            #1. goal center point
            goal_x = (lx + rx) / 2.0
            goal_z = (lz + rz) / 2.0

            #2. vector from ball to goal center
            vec_x = goal_x - bx
            vec_z = goal_z - bz
            length = math.hypot(vec_x, vec_z)

            if length == 0:
                return

            #3. normalized direction vector
            dir_x = vec_x / length
            dir_z = vec_z / length

            #3. position robot 0.4 meters behind the ball to the goal
            offset_distance = 0.35
            self.setup_x = bx - (dir_x * offset_distance)
            self.setup_z = bz - (dir_z * offset_distance)

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