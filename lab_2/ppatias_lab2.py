#!/usr/bin/env python

#===================================================================================================================================================================

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#===================================================================================================================================================================

wheelDistance  = 30.5 # In cm
wheelRadius = 3.8 # In cm

#===================================================================================================================================================================

def publishTwist(linVel, angVel):
    global pub
    msg = Twist()           # Make msg of type twist
    msg.linear.x = linVel   # Make linear velocity
    msg.angular.z = angVel  # Make angular Velocity
    pub.publish(msg) 		# Publish the msg

#===================================================================================================================================================================

def spinWheels(u1, u2, time):

    global pub

    v = (wheelRadius/2) * (u1 + u2)                     # Calculate
    w = (wheelDistanve/wheelDistance) * (u1 - u2))      # Required constants

    initTime = rospy.Time().now().secs                  # start timer

    while(rospy.Time().now().secs - initTime < time):   # calculate when to stop running
        
        makeVelMsg(v,w)                                 # movement message

    makeVelMsg(0,0)                                     # stop message


#===================================================================================================================================================================

def driveStraight(speed, distance):

    global pose      

    initX = pose.position.x     # Read first x and 
    initY = pose.position.y     # y position
    there = False 	            # distance flag

    while(not there and not rospy.is_shutdown()): # main while loop of the function

        currX = pose.position.x 	# reaed x and y postions
        currY = pose.position.y     # to know how much we have moved
        diffX = currX - initX		
        diffY = currY - initY		
        currPos = math.sqrt(diffX * diffX + diffY * diffY)
        
        if (currPos >= distance):   # check how much we moved
            there = True		    # if we got there
            publishTwist(0, 0)      # stop and exit
            
        else:
            publishTwist(speed,0)   # else keep going
            rospy.sleep(0.15)

#===================================================================================================================================================================

def rotate(angle):

    global pose 

    initAngle = pose.orientation.z              # find initial orintation
    
    desiredAngle = initAngle + angle            # find where do we wanna be looking
    
    if(desiredAngle > math.pi):                 # different case based on the sign of the rotation
        desiredAngle = desiredAngle -2*math.pi
    elif(desiredAngle < -math.pi):
        desiredAngle = desiredAngle + 2*math.pi

    there = False                               # rotation flag

    while (not there):                          # rotation calculation
        currentTheta = pose.orientation.z

        if(desiredAngle > 0):                   # if orientation is good stop
            if(currentTheta >= desiredAngle):
                there = True
                makeVelMsg(0,0)
            else:                               # else keep going
                makeVelMsg(0,math.pi/4)
    
        else: 
            if(currentTheta <= desiredAngle):   # if orientation is good stop
                there = True
                makeVelMsg(0,0)
            else:                               # else keep going
                makeVelMsg(0,-math.pi/4)
            
            rospy.sleep(0.15)                   # to ensure code keeps running and no issues appear

#===================================================================================================================================================================

def readBumper(msg):
    if (msg.state == 1): # When the bumper is pressed the code starts
        
        publishTwist(0, 0)  # Stop the robot from what it was doing
        executeTrajectory() # execute this instead
    else:
        publishTwist(0, 0)  # Stop the movement and continue where you left off

#===================================================================================================================================================================

def executeTrajectory():
    
    driveStraight(1, 0.6)   # move 60 cm
    
    rotate(math.pi)         # rotate pi
    
    driveStraight(1, 0.45)  # move 45 cm
      
    rotate(-math.pi)        # rotate -pi

#===================================================================================================================================================================

def navToPose(goal):
    global pose 
    
    initx = pose.position.x                     # Find x position
    inity = pose.position.y                     # Find y position
    initt = math.degrees(pose.orientation.z)    # Find z orientation

    diffX = goal.pose.position.x - initx        # x dist required
    diffY = goal.pose.position.y - inity        # y dist required

    dist = math.sqrt(diffX * diffX + diffY * diffY)     # distance required
    theta_t = math.degrees(math.atan2(diffY, diffX))    # theta required
    theta1 = theta_t - initt                            # make theta required

   
    a = goal.pose.orientation.x             # Quaternion values
    b = goal.pose.orientation.y             # as given from
    c = goal.pose.orientation.z             # rviz to show
    d = goal.pose.orientation.w             # the pose of the goal

    Y = 2 * (a*b + c*d)                     # Quaternion to euler
    X = ((a*a) + (d*d) - (b*b) - (c*c)      # Quaternion to euler
    yaw = math.atan2(Y, X)                  # find the theta that I want

    theta_f = -1 * math.degrees(yaw)        # Final orientation 

    rotate(theta1)                          # first rotation

    driveStraight(0.11, dist)               # drive straight

    rotate(theta_f)                         # last rotation

#===================================================================================================================================================================

def driveArc(radius, speed, angle):

    global odom_list
    global pose
    
    w = speed / radius          # find angular velocity
  
    diff = angle - math.degrees(pose.orientation.z)         # calc required rotation

    while ((abs(diff) >= 2) and not rospy.is_shutdown()):   # make the arc movement and be close enough to the required movement
        publishTwist(speed, w)                              # make the arc movement by going straight and turning at the same time
        diff = angle - math.degrees(pose.orientation.z) 

    publishTwist(0, 0)  # Once the movement is done stop robot

#===================================================================================================================================================================

def timerCallback(event):
    global pose

    pose = Pose() # make an object pose woth name pose
    

    odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(1.0)) 
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0)) 
    
    pose.position.x = position[0] 
    pose.position.y = position[1]
    pose.position.z = position[2]
   
    ODq = orientation			                    # Quaternion
    q = [ODq[0], ODq[1], ODq[2], ODq[3]]            # to
    roll, pitch, yaw = euler_from_quaternion(q)     # Euler

    pose.orientation.z = yaw  	                    # make required theta orientation
    theta = math.degrees(yaw)                       # make into degrees

#===================================================================================================================================================================

if __name__ == '__main__': # Main Function
    
    rospy.init_node('ppatias_lab2')

    global pub          # Globals
    global pose         # Used
    global odom_tf      # To Run
    global odom_list    # The Code

    pose = Pose()
       
    pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, None, queue_size=10)                        # publish a move to bot
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1)  # subscribe to read bumper
    nav_sub = rospy.Subscriber('/move_base_simple/goal1', PoseStamped, navToPose, queue_size=10)        # subscribe to read goal
    odom_list = tf.TransformListener() 
    
    rospy.sleep(rospy.Duration(1, 0)) 

    print "Starting Lab 2"    
    
    rospy.Timer(rospy.Duration(0.01), timerCallback)
     
    #while (not rospy.is_shutdown()):
        #rospy.spin()   

    #spinWheels(0.2, 0.0, 2)
    #driveStraight(1,1)
    #rotate(179)
    #readBumper(msg)
    #navToPose(goal)
    #driveArc(1, 0.2, 90)

    print "Lab 2 complete!"

