import rclpy
from rclpy.node import Node
import math
from tf2_msgs.msg import TFMessage
import cv2

class Part1(Node): 
    def __init__(self):
        super().__init__('part1')

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        self.subscription
        
    def tf_callback(self, msg):
        
        for tag in msg.transforms:
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

            
            tagID = tag.child_frame_id.split(":")[1]
            
            #print(f"id: {tagID} - location: ({x} , {y}, {z}) - ({roll}, {pitch}, {yaw}): ({math.degrees(roll)}, {math.degrees(pitch)}, {math.degrees(yaw)})")
            

        

def main(args=None):
    rclpy.init(args=args)
    aNode = Part1()
    try:
        rclpy.spin(aNode)
        
    except KeyboardInterrupt:
        pass
    finally:
        aNode.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
if __name__== '__main__':
    main()
