#!/usr/bin/env python

import rospy, tf, numpy, math

from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
	print "hit"
	global pose
	global theta
	
	#get goal pose
	goal = goal.pose
	# get quaternion, convert to euler
	quat = [goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w]
	euler = euler_from_quaternion(quat)

	#get the final theta of the goal pose
	theta1 = math.degrees(euler[2])
	print "euler: ", euler
	print "theta1: ", theta1
	
	#find the global angle to get between positions
	g_theta = math.degrees(math.atan((goal.position.y-pose.pose.position.y)/(goal.position.x-pose.pose.position.x)))
	print "g_theta: ", g_theta
	
	#find the first angle to rotate through
	first_theta = g_theta - math.degrees(pose.pose.orientation.z)
	print "first_theta: ", first_theta
	
	#find the distance
	distance = math.sqrt(math.pow(goal.position.x - pose.pose.position.x, 2) + math.pow(goal.position.x - pose.pose.position.y, 2))

	#find the angle to rotate at the end
	f_theta = theta1 - g_theta
	
	rotate(first_theta)
	driveStraight(0.5, distance)
	rotate(f_theta)
	
	pass


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    	driveStraight(.2, 1)
	rotate(90)
	driveStraight(.2, 0.5)
	rotate(135)
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

	initialX = pose.pose.position.x    
	initialY = pose.pose.position.y
	atTarget = False

	vel = Twist()

	while(not atTarget and not rospy.is_shutdown()):
		currentX = pose.pose.position.x
		currentY = pose.pose.position.y
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
	global theta

	turn = 0

	if(angle > 180 or angle<-180):
		print "angle is too large or too small"

	vel = Twist()
	done = True
        
	start = theta

	atTarget = False


	if angle < 0:
		turn = -1
	else:
		turn = 1

	#print pose.pose.orientation.z
	#current = abs(theta - start)
	#print current
	isStarted = False
	print "initial theta: ", theta
	print "angle: ", angle
	while not atTarget and not rospy.is_shutdown():
		current = abs(theta - start)
		if current > 180:
			current = 360 - current
		if(current > abs(angle)):
			atTarget = True
			vel.angular.z = 0.0
			print "done"
			print "curr, ", current
			print "start: ", start
			print "theta: ", theta
			print "angle: ", angle
			pub.publish(vel)
		else:
			vel.angular.z = turn*0.5
			#print "curr, ", current
			#print "start: ", start
			#print "theta: ", theta
			#print "angle: ", angle
			if(isStarted == False):
				rospy.sleep(0.2)
				isStarted = True
				start = theta
			pub.publish(vel)
		rospy.sleep(0.2)
			
	
	#print pose.pose.orientation.z 
	#print theta
	


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented





#Bumper Event Callback function
def readBumper(msg):
    print "gets here"
    if (msg.state == 1):
	print "here too"
	#rotate(90)
        publishTwist(0,0)




# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code:
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    global xpos
    global ypos
    global theta

    pose = PoseStamped()
    #print "hi"
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)


    pose.pose.position.x=position[0]
    pose.pose.position.y=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    pose.pose.orientation.z = yaw
    theta = math.degrees(yaw)



# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables
    global pub
    global pose
    global odom_tf
    global odom_list
    global theta
    pose = PoseStamped()

    theta = 0


    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size = 10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    click_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, navToPose)

    # Use this object to get the robot's Odometry
    odom_list = tf.TransformListener()

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"
	
    rospy.Timer(rospy.Duration(.1), timerCallback)

    while(1):
	rospy.sleep(0.2)
        
    #executeTrajectory()
    #odom_list = tf.TransformListener()

	#make the robot keep doing something...
	#rospy.Timer(rospy.Duration(...), timerCallback)
    # Make the robot do stuff...
    print "Lab 2 complete!"
