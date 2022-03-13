import sys

import rospy
from sensor_msgs.msg import Image, CompressedImage

import cv2

from cv_bridge import CvBridge

class LazyWriter(object):
    def __init__(self, message_type, outfile):
        self.writer = None
        self.outfile = outfile
        self.convert = self.select_converter(message_type)

    @staticmethod
    def select_converter(message_type):
        bridge = CvBridge()
        return {
            Image: bridge.imgmsg_to_cv2,
            CompressedImage: bridge.compressed_imgmsg_to_cv2
        }[message_type]

    def write(self, msg):
        cv_img = self.convert(msg, desired_encoding='bgr8') # passthrough
        if not self.writer:
            height, width = cv_img.shape[:2]
            encoder = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(self.outfile, encoder, 24, (width,height))
        self.writer.write(cv_img)

    def close():
        if self.writer:
            self.writer.release()

topic, message_type, outfile = sys.argv[1:]
message_type = {
    "Image": Image,
    "CompressedImage": CompressedImage
}[message_type]

writer = LazyWriter(message_type, outfile)
rospy.init_node('video_saver_node', anonymous=True)
rospy.Subscriber(topic, message_type, writer.write)
rospy.spin()