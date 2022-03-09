import rospy
from geometry_msgs.msg import Twist

# publish to the motor topic

rospy.init_node('my_crappy_motor_driver', anonymous=True)

pub = rospy.Publisher('motor', Twist, queue_size=10)
rate = rospy.Rate(1)

def xTwist(linear, angular):
	t = Twist()
	t.linear.x = linear
	t.angular.x = angular
	return t

for i in range(10):
	pub.publish(xTwist(100,0))
	rate.sleep()
	pub.publish(xTwist(0,100))
	rate.sleep()
	pub.publish(xTwist(0,-100))
	rate.sleep()
	pub.publish(xTwist(-100,0))
	rate.sleep()

pub.publish(xTwist(0,0))
