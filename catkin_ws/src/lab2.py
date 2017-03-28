#!/usr/bin/env python

import rospy, tf
from kobuki_msgs.msg import BumperEvent
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):

	pass


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    
	pass  # Delete this 'pass' once implemented




#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):	
	global pub

	l = .23 # convert wheelbase to meters


	twist_msg = Twist()    
	stop_msg = Twist()

	twist_msg.linear.x = 0.5*(u1+u2)
	twist_msg.angular.z = float(u1-u2)/l

	stop_msg.linear.x = 0.5*(u1+u2)
	stop_msg.angular.z = float(u1-u2)/l

    
	now = rospy.Time.now().secs    
	while(rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
		pub.publish(twist_msg)    


	pub.publish(stop_msg)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):

	global pose

	initialX = pose.position.x    
	initialY = pose.position.y
	atTarget = False

	vel = Twist()

	while(not atTarget and not rospy.is_shutdown()):
		currentX = pose.position.x
		currentY = pose.position.y
		currentDistance = math.sqrt((currentX - initialX)**2 + (currentY - initialY)**2)

		if (currentDistance >= distance):
			atTarget = True
			vel.linear.x = 0
		else:
			vel.linear.x = speed
			pub.publish(vel)
			rospy.sleep(0.15)


#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	global odom_list
	global pose

	turn = 0

	if(angle > 180 or angle<-180):
		print "angle is too large or too small"

	vel = Twist()
	done = True

	error = angle-math.degrees(pose.orientation.z)

	if angle > 0:
		turn = 1
	else:
		turn = -1

	while ((abs(error) >= 2) and not rospy.is_shutdown()):
		if(turn == 1):
			vel.angular.z = 0.5
		else:
			vel.angular.z = -0.5
		pub.publish(vel)

	vel.angular.z = 0.0
	pub.publish(vel)


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented





#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        publishTwist(0,0)




# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code:
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    global xpos
    global ypos
    global theta

    pose = Pose()

    (position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)

    pass # Delete this 'pass' once implemented




# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables
    global pub
    global pose
    global odom_tf
    global odom_list
    pose = Pose()

    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size = 10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('...', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry
    odom_list = tf.TransformListener()

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"
	
    driveStraight(.1,10)

	#make the robot keep doing something...
	#rospy.Timer(rospy.Duration(...), timerCallback)
    # Make the robot do stuff...
    print "Lab 2 complete!"
