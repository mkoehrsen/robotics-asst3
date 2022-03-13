#!/usr/bin/env python3
# Parts of this code are based on the 'teleop_twist_keyboard' node

import rospy
from std_msgs.msg import String
import sys, select, termios, tty

command = 'stop'
last_command = 'stop'

msg = """
Reading from the keyboard and publishing to /command!
---------------------------
Moving around:
        i     
   j    k    l
        ,     

CTRL-C to quit
"""

def talker():
    global command
    global last_command
    
    pub = rospy.Publisher('/command', String, queue_size=10)
    rospy.init_node('keyb_commander', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    print(msg)
    # rospy.rosinfo(msg)
    while not rospy.is_shutdown():
        key_timeout = 0.6
        k = getKey(key_timeout)
        
        if not k:
            command = None
        elif k == "i":
            command = 'forward'
        elif k == ",":
            command = 'backward'
        elif k == "j":
            command = 'left'
        elif k == "l":
            command = 'right'
        elif k == "k":
            command = 'stop'
        elif k == "a":
            command = 'auto'
        elif k == '\x03': # To detect CTRL-C
            break
            
        if command and command != last_command:
            pub.publish(command)
            print("Published command: " + command)
            last_command = command
        rate.sleep()

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

