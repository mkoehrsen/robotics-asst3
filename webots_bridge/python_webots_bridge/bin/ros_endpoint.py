#!/usr/bin/env python3

import sys

from webots_bridge import connection, messages
from webots_bridge.devices import devices

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float64

port, = sys.argv[1:]
port = int(port)

print("Listening for connection")
conn = connection.listen(port)

print("Registering ROS node")
rospy.init_node("ros_endpoint", anonymous=True)

print("Subscribing to 'motor' topic")
def set_motors(ros_twist):
    # math from http://wiki.ros.org/simple_drive#Tank_to_Twist_Calculation
    lin = ros_twist.linear.x
    ang = ros_twist.angular.x
    left = lin + ang
    right = lin - ang
    for m, v in (("LEFT_MOTOR", left), ("RIGHT_MOTOR", right)):
        conn.send(devices[m], messages.MotorVelocity(velocity=v))
rospy.Subscriber("motor", Twist, set_motors)

print("Preparing 'camera' publisher")
class CameraProcessor(object):
    def __init__(self):
        self.pub = rospy.Publisher("camera", Image, queue_size=10)
        self.handlers = {
            "CameraInfo": self.set_camera_info,
            "CameraImage": self.publish_image
        }
    def process_message(self, msg):
        self.handlers[msg.__class__.__name__](msg)
    def set_camera_info(self, camera_info):
        self.camera_info = camera_info
    def convert_image(self, camera_image):
        return Image(
            height = self.camera_info.height,
            width = self.camera_info.width,
            data = camera_image.image,
            # webots promises 8-bit BGRA for the native ("internal") format
            encoding = "bgra8",
            is_bigendian = True
        )
    def publish_image(self, camera_image):
        self.pub.publish(self.convert_image(camera_image))
camera_processor = CameraProcessor()

handlers = {
    devices["CAMERA"]: camera_processor.process_message
}

while not conn.closed():
    device, msg = conn.receive()
    handlers[device](msg)