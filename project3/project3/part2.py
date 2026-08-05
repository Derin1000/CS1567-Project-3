import rclpy
from rclpy.node import Node
import math
from tf2_msgs.msg import TFMessage
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys

class Part2(Node): 
    def __init__(self):
        super().__init__('part2')

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        self.subscription
        

        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.move_callback)
        

        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.target_linear = 0.0
        self.target_angular = 0.8
        

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.angular_pos = 0.0
        

        

        self.ids = []

        
        self.target_id = -1
        self.target_lock = False
        self.linear_move = False
        self.target_pos = (-1, -1, -1)
        
        self.delta_linear = 0.025 #0.1
        self.delta_angular = 0.1 #0.2
        
        self.decel = False
        self.init_y = -1
        self.peak_speed_reached = False
        

        
    def move_callback(self):
        cmd = Twist()
        
        print(self.target_lock, " ", self.target_pos[1])
    
        if self.target_pos[0] <= 0.01 and self.target_pos[0] != -1:
            self.target_angular = 0.0
            self.linear_move = True
        
        if self.linear_move:
            if self.target_pos[0] > 0.01:
                self.target_angular = -0.2
            elif self.target_pos[0] < -0.01:
                self.target_angular = 0.2
            print('\tTURNING: ', self.target_pos[0])
        
        if self.target_lock and self.peak_speed_reached and abs(self.target_pos[1] - self.init_y) <= 0.4 and self.target_linear > 0:
            self.target_linear = 0.0
            print(self.target_pos[1])
            if self.current_linear <= 0.03 and self.decel:
                sys.exit(0)
        
        #angular move - update angular speed
        print(self.target_angular, " ", self.current_angular)
        if abs(self.target_angular - self.current_angular) < self.delta_angular:
            self.current_angular = self.target_angular
            if self.target_angular == 0 and not self.decel:
                self.target_linear = 0.5
        else:
            if self.target_angular > self.current_angular:
                self.current_angular += self.delta_angular
            elif self.target_angular < self.current_angular:
                self.current_angular -= self.delta_angular
                    
       
        if abs(self.target_linear - self.current_linear) < self.delta_linear:       #linear move - update linear speed
            self.current_linear = self.target_linear
            if self.target_linear == 0.5:
                self.peak_speed_reached = True
        else:
            if self.target_linear > self.current_linear and not self.decel:
                self.current_linear += self.delta_linear
            elif self.target_linear < self.current_linear:
                self.current_linear -= self.delta_linear
                self.decel = True
                    
            
            
            
        
        
        
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        self.pub_vel.publish(cmd)
        
        self.prev_angular = self.angular_pos
        
    
        
    def tf_callback(self, msg):

        if not self.target_lock or self.linear_move:
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
                        self.init_y = y
                        break
                
                
                #print(f"id: {tagID} - location: ({x} , {y}, {z}) - ({roll}, {pitch}, {yaw}): ({math.degrees(roll)}, {math.degrees(pitch)}, {math.degrees(yaw)})")
            
        

def main(args=None):
    rclpy.init(args=args)
    aNode = Part2()
    try:
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
