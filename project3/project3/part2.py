import rclpy
from rclpy.node import Node
import math
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
#import cv2
#from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

class Part2(Node): 
    def __init__(self):
        super().__init__('part2')

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        self.subscription
        
        #self.subscription = self.create_subscription(
        #    Image,
        #    '/image_raw',
        #    self.openCV_callback,
        #    10)
        #self.subscription
        
        #self.subscription = self.create_subscription(
        #    AprilTagDetectionArray,
        #    '/detections',
        #    self.corners_callback,
        #    10)
        #self.subscription
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reset = self.create_publisher(Empty, '/commands/reset_odometry', 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.move_callback)
        
        #instance variables for velocity constants
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.target_linear = 0.0
        self.target_angular = 0.8
        
        #current orientation via odometry
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.angular_pos = 0.0
        
        #self.br = CvBridge()
        
        #self.rectangles = []
        self.ids = []
        #self.centers = []
        
        self.target_id = -1
        self.target_lock = False
        self.target_pos = (-1, -1, -1)
        
        self.delta_linear = 0.05 #0.1
        self.delta_angular = 0.1 #0.2
        
        #self.decel_pos_ang = 0
        
    def move_callback(self):
        cmd = Twist()
        
        
        #if not self.target_lock:   #if target apriltag hasn't been identified, keep turning
        #    self.target_linear = 0.0
        #    self.target_angular = 0.2
    
        if self.target_pos[0] <= 0.01 and self.target_pos[0] != -1:
            self.target_angular = 0.0
        
        if self.target_lock and self.target_pos[1] <= 0.055:
            self.target_linear = 0.0
        
        if self.target_angular != self.current_angular:    #angular move - update angular speed
            print(self.target_angular, " ", self.current_angular)
            if abs(self.target_angular - self.current_angular) < self.delta_angular:
                self.current_angular = self.target_angular
                if self.target_angular == 0:
                    self.target_linear = 0.5
            else:
                if self.target_angular > self.current_angular:
                    self.current_angular += self.delta_angular
                elif self.target_angular < self.current_angular:
                    self.current_angular -= self.delta_angular
                    
       
        if abs(self.target_linear - self.current_linear) < self.delta_linear:       #linear move - update linear speed
            self.current_linear = self.target_linear
        else:
            if self.target_linear > self.current_linear:
                self.current_linear += self.delta_linear
            elif self.target_linear < self.current_linear:
                self.current_linear -= self.delta_linear
                    
            
            
            
        
        
        
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        self.pub_vel.publish(cmd)
        
        self.prev_angular = self.angular_pos
        
    #def openCV_callback(self, msg):
    #    frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #    for i in range(len(self.rectangles)):
    #        cv2.line(frame, self.rectangles[i][0], self.rectangles[i][1], (255, 0, 0), 2)
    #        cv2.line(frame, self.rectangles[i][1], self.rectangles[i][2], (255, 0, 0), 2)
    #        cv2.line(frame, self.rectangles[i][2], self.rectangles[i][3], (255, 0, 0), 2)
    #        cv2.line(frame, self.rectangles[i][3], self.rectangles[i][0], (255, 0, 0), 2)
    #        
    #        cv2.putText(frame, str(self.ids[i]), self.centers[i], cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    #        
    #    
    #    cv2.imshow("Camera", frame)
    #    cv2.waitKey(1)
        
    #def corners_callback(self, msg):
    #    rectangles = []
    #    ids = []
    #    centers = []
    #    for tag in msg.detections:
    #        ids.append(tag.id)
    #        centers.append((int(tag.centre.x), int(tag.centre.y)))
    #        curRect = []
    #        for i in range(4):
    #            curRect.append((int(tag.corners[i].x), int(tag.corners[i].y)))
    #        
    #        rectangles.append(curRect)
    #        
    #    self.ids = ids
    #    self.rectangles = rectangles
    #    self.centers = centers
        
    def tf_callback(self, msg):
        if not self.target_lock:
            for tag in msg.transforms:
                if ":" in tag.child_frame_id:
                    
                    x = tag.transform.rotation.x
                    y = tag.transform.rotation.y
                    z = tag.transform.rotation.z
                    w = tag.transform.rotation.w
                    
                    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
                    sinp = 2 * (w * y - z * w)
                    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
                    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
                    
                    x = tag.transform.translation.x
                    y = tag.transform.translation.y
                    z = tag.transform.translation.z

                    
                    tagID = int(tag.child_frame_id.split(":")[1])
                    if tagID == self.target_id:     #target apriltag detected
                        print('DETECTED')
                        self.target_lock = True
                        self.target_pos = (x, y, z)
                        break
                
                
                #print(f"id: {tagID} - location: ({x} , {y}, {z}) - ({roll}, {pitch}, {yaw}): ({math.degrees(roll)}, {math.degrees(pitch)}, {math.degrees(yaw)})")
            
    def odom_callback(self, msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w
            siny_cosp = 2 * w * z
            cosy_cosp = 1 - 2 * z * z
            yaw = math.atan2(siny_cosp, cosy_cosp)
            degree = yaw * 180 / math.pi
            #print ('x: %f y: %f Orientation: %f' % (x, y, degree))
            self.x_pos = x
            self.y_pos = y
            self.angular_pos = degree
        

def main(args=None):
    rclpy.init(args=args)
    aNode = Part2()
    try:
        aNode.pub_reset.publish(Empty())
        aNode.target_id = int(input("Target AprilTagID: "))
        rclpy.spin(aNode)
        
    except KeyboardInterrupt:
        pass
    finally:
        aNode.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
if __name__== '__main__':
    main()
