#!/usr/bin/env python3
import threading
import rospy
import time
from std_msgs.msg import String, UInt16
from geometry_msgs.msg import Twist, Point

speed_const = 4

# maps command name to a handler function
opcodes = {}
def opcode(name=None):
    """ A decorator that registers the function as a command handler. """
    def decor(fn):
        if name is not None:
            n = name
        else:
            n = fn.__code__.co_name
        assert n not in opcodes
        opcodes[n] = fn
        return fn
    return decor

def listener():
    rospy.init_node('motor_driver', anonymous=True)
    rospy.Subscriber('/command', String, commandCallback)
    rospy.spin()

last_command_data = []
def commandCallback(message):
    received_command = message.data

    # stop the last command if there is one
    last_cmd = None
    if last_command_data:
        last_cmd, stopper = last_command_data[-1]
        stopper.set()
        del last_command_data[:]
    
    if received_command != last_cmd:
        print('Received command: ' + received_command)

    handler = opcodes.get(received_command)
    stopper = threading.Event()
    last_command_data.append((received_command, stopper))
    if handler is None:
        print('Unknown command!')
    else:
        # run the handler in a thread so it can loop while we wait for another command
        handler_thread = threading.Thread(target=handler, args=[stopper], daemon=True)
        handler_thread.start()
                
motor_pub = rospy.Publisher("motor", Twist, queue_size=10)

class AutoMode(object):
    """ Listen to data from the vision node, and navigate there when asked. """

    MIN_TGT_RADIUS_PERCENT = 0.010
    CENTER_WIDTH_PERCENT = 0.25

    def __init__(self):
        rospy.Subscriber('/target_coord', Point, self.set_target_coord)
        rospy.Subscriber('/target_radius', UInt16, self.set_target_radius)
        rospy.Subscriber('/image_width', UInt16, self.set_image_width)
        self.command_pub = rospy.Publisher('/command', String, queue_size=10)
        self.image_width = 0
        self.target_radius = 0
        self.target_coord = Point(0,0,0)
        
    def set_image_width(self, msg):
        self.image_width = msg.data
        
    def set_target_radius(self, msg):
        self.target_radius = msg.data
        
    def set_target_coord(self, msg):
        self.target_coord = msg
        
    def step(self):
        """
        Make one navigation decision given current information.
        Return True if that decision was to pursue.
        """
        if (
            self.target_radius >= self.image_width * self.MIN_TGT_RADIUS_PERCENT
        ):
            if self.target_radius <= self.image_width/3:
                if abs(self.target_coord.x) <= self.image_width * self.CENTER_WIDTH_PERCENT:
                    forward()
                elif self.target_coord.x < 0:
                    left()
                elif self.target_coord.x > 0:
                    right()
                else:
                    raise Exception("Shouldn't happen...double-check logic for the cases?")
                return True
            else:
                print("Reached target, stopping")
                self.command_pub.publish("stop")
                return False
        else:
            print("Target not found, waiting")
            stopMotors()
            return True
    
def publish_twist(lin, ang):
    t = Twist()
    t.linear.x = lin
    t.angular.x = ang
    motor_pub.publish(t)

auto_mode = AutoMode()
@opcode()
def auto(stop_event):
    in_pursuit = True
    while not stop_event.is_set() and in_pursuit:
        in_pursuit = auto_mode.step()
        stop_event.wait(0.05)

@opcode("stop")
def stopMotors(*args):
    publish_twist(0,0)

@opcode()
def forward(*args):
    publish_twist(speed_const, 0)

@opcode()
def backward(*args):
    publish_twist(-speed_const, 0)
    
@opcode()
def left(*args):
    publish_twist(speed_const/2, -speed_const/2)

@opcode()
def right(*args):
    publish_twist(speed_const/2, speed_const/2)

if __name__ == '__main__':
    print('Ready to receive commands!')
    listener()
    print('Node is shutting down, stopping motors')
    stopMotors()