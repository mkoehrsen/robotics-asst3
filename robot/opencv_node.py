#!/usr/bin/env python

import sys
import time
from collections import deque
from threading import Condition
 
# Import libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point # Point (x, y, z) message type
from std_msgs.msg import UInt16 # Unsigned integer message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Converts between OpenCV and ROS images

# Global constants and variables
NUM_FILT_POINTS      = 5 # Number of filtering points for the Moving Average Filter
DESIRED_IMAGE_HEIGHT = 240 # A smaller image makes the detection less CPU intensive

# A dictionary of two empty buffers (arrays) for the Moving Average Filter
filt_buffer = {'width':[], 'height':[]}

# A dictionary of general parameters derived from the camera image size,
# which will be populated later with the 'get_image_params' function
params = {'image_height':None, 'image_width': None,'resized_height':None,'resized_width': None,
    'x_ax_pos':None, 'y_ax_pos':None, 'scaling_factor':None}

class CameraTopicVideoCapture(object):
  """
  A fake cv2 camera source which actually pulls frames from a topic.
  We don't need to implement much of VideoCapture.
  """
  def __init__(self, topic):
    rospy.Subscriber('camera', Image, self._set_next_frame)
    self.bridge = CvBridge()
    self.deq = deque(maxlen=1)
    self.condition = Condition()
  def _set_next_frame(self, img):
      with self.condition:
        self.deq.append(img)
        self.condition.notify()
  def read(self):
      with self.condition:
        self.condition.wait_for(lambda: len(self.deq))
      img = self.deq.popleft()
      return 1, self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
  def isOpened(self):
    return True
  
def detect_target(camera_factory):
  """ Main entry function for this node. """
 
  # Publishes the video frames from the detection process
  # detect_image_pub = rospy.Publisher('detect_image', Image, queue_size=10)
  detect_image_pub = rospy.Publisher('detect_image/compressed', CompressedImage, queue_size=10)

  # Publishes the (x, y, 0) coordinates for the detected target
  target_coord_pub = rospy.Publisher('/target_coord', Point, queue_size=10)

  # Publishes the detected target's computed enclosing radius
  target_radius_pub = rospy.Publisher('/target_radius', UInt16, queue_size=10)

  # Publishes the image frame width after scaling used in the detection process
  image_width_pub = rospy.Publisher('/image_width', UInt16, queue_size=10)
     
  # Set the node's name
  rospy.init_node('detection_node', anonymous=True)
     
  # The node will run 30 times per second
  rate = rospy.Rate(30) # 30 Hz
     
  # Create a VideoCapture object
  #vid_cam = cv2.VideoCapture(0) # '0''is de index for the default webcam
  vid_cam = camera_factory()

  # Check if the camera opened correctly
  if vid_cam.isOpened() is False: 
    print('[ERROR] Couldnt open the camera.')
    return
      
  print('-- Camera opened successfully')
  
  # Compute general parameters
  get_image_params(vid_cam) 
  print("-- Original image width, height: ", {params['image_width']}, {params['image_height']})
  print("-- Resized image width, height: ", {params['resized_width']}, {params['resized_height']})
     
  # To convert between OpenCV and ROS images
  bridge = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():
      start_time = time.time()

      # Get the target coordinates
      tgt_cam_coord, frame, contour, radius = get_target_coordinates(vid_cam)
      
      # If a target was found, filter their coordinates
      if tgt_cam_coord['width'] is not None and tgt_cam_coord['height'] is not None:
          # Apply Moving Average filter to target camera coordinates
          tgt_filt_cam_coord = moving_average_filter(tgt_cam_coord)

      # No target was found, set target camera coordinates to the Cartesian origin,
      # so the drone doesn't move
      else:
          # The Cartesian origin is where the x and y Cartesian axes are located
          # in the image, in pixel units
          tgt_cam_coord = {'width':params['y_ax_pos'], 'height':params['x_ax_pos']} # Needed just for drawing objects
          tgt_filt_cam_coord = {'width':params['y_ax_pos'], 'height':params['x_ax_pos']}

      # Convert from camera coordinates to Cartesian coordinates (in pixel units)
      tgt_cart_coord = {'x':(tgt_filt_cam_coord['width'] - params['y_ax_pos']),
                        'y':(params['x_ax_pos'] - tgt_filt_cam_coord['height'])}

      # Draw objects over the detection image frame just for visualization
      frame = draw_objects(tgt_cam_coord, tgt_filt_cam_coord, frame, contour)

      # Publish the detection image after converting from OpenCV to ROS
      # detect_image_pub.publish(bridge.cv2_to_imgmsg(frame))
      detect_image_pub.publish(bridge.cv2_to_compressed_imgmsg(frame))

      # Publish the detected target's coordinates (x, y, 0)
      tgt_coord_msg = Point(tgt_cart_coord['x'], tgt_cart_coord['y'], 0)
      target_coord_pub.publish(tgt_coord_msg)

      # Publish de detected target's enclosing radius
      target_radius_pub.publish(radius)

      # Publish the image frame's resized width
      image_width_pub.publish(params['resized_width'])

      # Show the detection image frame on screen
      # Optionally you can comment this line when running this node remotely through SSH:
      # cv2.imshow("Detect and Track", frame)

      delta_time = end_time = time.time()
      detection_time = round(end_time-start_time, 3)
      #print("Detection time: " + str(detection_time ))

      # Catch aborting key from computer keyboard
      key = cv2.waitKey(1) & 0xFF
      # If the 'q' key is pressed, break the 'while' infinite loop
      if key == ord("q"):
          break
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
def get_image_params(vid_cam):
  """ Computes useful general parameters derived from the camera image size."""

  # Grab a frame and get its size
  is_grabbed, frame = vid_cam.read()
  params['image_height'], params['image_width'], _ = frame.shape

  # Compute the scaling factor to scale the image to a desired size
  if params['image_height'] != DESIRED_IMAGE_HEIGHT:
      # Rounded scaling factor. Convert 'DESIRED_IMAGE_HEIGHT' to float or the division will throw zero
      params['scaling_factor'] = round((float(DESIRED_IMAGE_HEIGHT) / params['image_height']), 3) 
  else:
      params['scaling_factor'] = 1

  print("params['scaling_factor']: ", params['scaling_factor'])
  print("params['scaling_factor']: ", DESIRED_IMAGE_HEIGHT / params['image_height'])
  # Compute resized width and height and resize the image
  params['resized_width'] = int(params['image_width'] * params['scaling_factor'])
  params['resized_height'] = int(params['image_height'] * params['scaling_factor'])
  dimension = (params['resized_width'], params['resized_height'])
  # dimension = (int(params['resized_width']), int(params['resized_height']))
  frame = cv2.resize(frame, dimension, interpolation = cv2.INTER_AREA)

  # Compute the position for the X and Y Cartesian coordinates in camera pixel units
  params['x_ax_pos'] = int(params['resized_height']/2 - 1)
  params['y_ax_pos'] = int(params['resized_width']/2 - 1)

  return

def get_target_coordinates(vid_cam):
    """ Detects a target by using color range segmentation and returns its 'camera pixel' coordinates."""

    # Use the 'threshold_inRange.py' script included with the code to get
    # your own bounds with any color
    # To detect a blue target:
    ###HSV_LOWER_BOUND = (107, 119, 41)
    ###HSV_UPPER_BOUND = (124, 255, 255)
    # looking for yellow, which I think is is 30, 255, 255
    HSV_LOWER_BOUND = (20, 100, 100)    
    HSV_UPPER_BOUND = (40, 255, 255)

    # Grab a frame in BGR (Blue, Green, Red) space color
    is_grabbed, frame = vid_cam.read()

    # Resize the image frame for the detection process, if needed
    if params['scaling_factor'] != 1:
        dimension = (params['resized_width'], params['resized_height'])
        frame = cv2.resize(frame, dimension, interpolation = cv2.INTER_AREA)

    # Blur the image to remove high frequency content
    blurred = cv2.GaussianBlur(frame, (11, 11), 0) 
    
    # Change color space from BGR to HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Histogram equalisation to minimize the effect of variable lighting
    # hsv[:, :, 0] = cv2.equalizeHist(hsv[:, :, 0]) # on the H-channel
    # hsv[:, :, 1] = cv2.equalizeHist(hsv[:, :, 1]) # on the S-channel
    # hsv[:, :, 2] = cv2.equalizeHist(hsv[:, :, 2]) # on the V-channel

    # Get a mask with all the pixels inside our defined color boundaries
    mask = cv2.inRange(hsv, HSV_LOWER_BOUND, HSV_UPPER_BOUND)

    # Erode and dilate to remove small blobs
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find all contours in the masked image
    ret = cv2.findContours(mask, 
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #print("ret", len(ret), ret[0][0].shape)
    #_, contours, _ = ret
    contours, _ = ret

    # Centroid coordinates to be returned:
    cX = None
    cY = None

    # To save the larges contour, presumably the detected object
    largest_contour = None
    tgt_radius = 0

    # Check if at least one contour was found
    if len(contours) > 0:
        # Get the largest contour of all posibly detected
        largest_contour = max(contours, key=cv2.contourArea)

        # Compute the radius of an enclosing circle aorund the largest contour
        (x,y), tgt_radius  = cv2.minEnclosingCircle(largest_contour)
         
        center = (int(x),int(y))
        tgt_radius = int(tgt_radius)
        # cv2.circle(frame, center, tgt_radius , (3, 186, 252), 3)

        # Compute contour raw moments
        M = cv2.moments(largest_contour)
        # Get the contour's centroid
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

    # Return centroid coordinates (camera pixel units), the analyzed frame and the largest contour
    return {'width':cX, 'height':cY}, frame, largest_contour, tgt_radius


def moving_average_filter(coord):
    """ Applies Low-Pass Moving Average Filter to a pair of coordinates."""

    # Append new coordinates to filter buffers
    filt_buffer['width'].append(coord['width'])
    filt_buffer['height'].append(coord['height'])

    # If the filters were full already with a number of NUM_FILT_POINTS values, 
    # discard the oldest value (FIFO buffer)
    if len(filt_buffer['width']) > NUM_FILT_POINTS:
        filt_buffer['width'] = filt_buffer['width'][1:]
        filt_buffer['height'] = filt_buffer['height'][1:]
    
    # Compute filtered camera coordinates
    N = len(filt_buffer['width']) # Get the number of values in buffers (will be < NUM_FILT_POINTS at the start)

    # Sum all values for each coordinate
    w_sum = sum( filt_buffer['width'] )
    h_sum = sum( filt_buffer['height'] )
    # Compute the average
    w_filt = int(round(w_sum / N))
    h_filt = int(round(h_sum / N))

    # Return filtered coordinates as a dictionary
    return {'width':w_filt, 'height':h_filt}


def draw_objects(cam_coord, filt_cam_coord, frame, contour):
  """ Draws visualization objects from the detection process.
  Position coordinates of every object are always in 'camera pixel' units"""

  # Draw the Cartesian axes
  cv2.line(frame, (0, params['x_ax_pos']), (params['resized_width'], params['x_ax_pos']), (0, 128, 255), 1)
  cv2.line(frame, (params['y_ax_pos'], 0), (params['y_ax_pos'], params['resized_height']), (0, 128, 255), 1)
  cv2.circle(frame, (params['y_ax_pos'], params['x_ax_pos']), 1, (255, 255, 255), -1)

  # Draw the detected object's contour, if any
  if contour is not None:
      cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

  # Compute Cartesian coordinates of unfiltered detected object's centroid
  x_cart_coord = cam_coord['width'] - params['y_ax_pos']
  y_cart_coord = params['x_ax_pos'] - cam_coord['height']

  # Compute Cartesian coordinates of filtered detected object's centroid
  x_filt_cart_coord = filt_cam_coord['width'] - params['y_ax_pos']
  y_filt_cart_coord = params['x_ax_pos'] - filt_cam_coord['height']

  # Draw unfiltered centroid as a red dot, including coordinate values
  cv2.circle(frame, (cam_coord['width'], cam_coord['height']), 5, (0, 0, 255), -1) 
  cv2.putText(frame, str(x_cart_coord) + ", " + str(y_cart_coord), 
      (cam_coord['width'] + 25, cam_coord['height'] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

  # Draw filtered centroid as a yellow dot, including coordinate values
  cv2.circle(frame, (filt_cam_coord['width'], filt_cam_coord['height']), 5, (3, 186, 252), -1)
  cv2.putText(frame, str(x_filt_cart_coord) + ", " + str(y_filt_cart_coord), 
      (filt_cam_coord['width'] + 25, filt_cam_coord['height'] - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (3, 186, 252), 1)

  return frame # Return the image frame with all drawn objects


if __name__ == '__main__':

  frame_source, = sys.argv[1:]
  camera_factory = {
    "camera": lambda: cv2.VideoCapture(0),
    "topic": lambda: CameraTopicVideoCapture("camera")
  }[frame_source]

  try:
    detect_target(camera_factory)
  except rospy.ROSInterruptException:
    pass