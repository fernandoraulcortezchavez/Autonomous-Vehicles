#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import numpy as np
from time import sleep
from pynput.keyboard import Key, Listener

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a         d
        s     
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
        'w':(1,0,0,0),
        's':(-1,0,0,0),
        'a':(0,1,0,0),
        'd':(0,-1,0,0),
        'q':(0,0,0,1),
        'e':(0,0,0,-1),
        'u':(0,0,1,0),
        'j':(0,0,-1,0),
        '0':(0,0,0,0),
    }

speedBindings={
}

# Declare global variables for topics
pub = None
takeoff = None
land = None
reset = None
speed = 1
turn = 1
manual_mode = True
circle_mode = False
trajectory_mode = False
controller_on = True
global_pose = [0.0, 0.0, 0.0, 0.0]

def QuadLand():
    global land
    empty = Empty()
    land.publish(empty)

def QuadTakeoff():
    global takeoff
    empty = Empty()
    takeoff.publish(empty)

def on_press(key):
    #print("On press")
    global pub
    global takeoff
    global land
    global reset
    global speed
    global turn
    global circle_mode
    global controller_on
    global trajectory_mode

    if pub is None or takeoff is None or land is None or reset is None:
        return
    if key != Key.shift_r and key != Key.shift_l:
        print('{0} pressed'.format(
        key))
    # Movement keys
    if key.char in moveBindings.keys():
        circle_mode = False
        trajectory_mode = False
        x = moveBindings[key.char][0]
        y = moveBindings[key.char][1]
        z = moveBindings[key.char][2]
        th = moveBindings[key.char][3]
        twist = Twist()
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        pub.publish(twist)
        print(x,y,z,th)
        
    #elif key.char in speedBindings.keys():
    #    speed = speed * speedBindings[key][0]
    #    turn = turn * speedBindings[key][1]

    # Control keys
    elif key.char == 'v':
        circle_mode = False
	trajectory_mode = False
        empty = Empty()
        takeoff.publish(empty)
        print("Takeoff")
    elif key.char == 'b':
        circle_mode = False
	trajectory_mode = False
        empty = Empty()
        land.publish(empty)
        print("Land")
    elif key.char == 'n':
        circle_mode = False
	trajectory_mode = False
        empty = Empty()
        reset.publish(empty)
        print("Reset")
    elif key.char == '0':
        print("Turning off controller")
	circle_mode = False
	trajectory_mode = False
	controller_on = False
        empty = Empty()
        land.publish(empty)

    # Routine keys
    elif key.char == 'c':
        print('Circle')
        circle_mode = True
	trajectory_mode = False
    elif key.char == 't':
        circle_mode = False
	trajectory_mode = True


def on_release(key):
    global pub
    global takeoff
    global land
    global reset
    global speed
    global turn
    global controller_on
    
    if pub is None or takeoff is None or land is None or reset is None:
        return False
    if key == Key.esc:
        # Stop listener
        return False
    if key != Key.shift_r and key != Key.shift_l:
        print('{0} release'.format(
        key))
    if key.char == '0':
        controller_on = False
        return False
    if key.char in moveBindings.keys():
        x = 0
        y = 0
        z = 0
        th = 0
        twist = Twist()
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        pub.publish(twist)
        print(x,y,z,th)
    

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def get_odometry(msg):
        global global_pose
        print(len(global_pose))
        global_pose[0] = msg.pose.pose.position.x
        global_pose[1] = msg.pose.pose.position.y
        global_pose[2] = msg.pose.pose.position.z
	quaternion = msg.pose.pose.orientation
	quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
        global_pose[3] = yaw
        print("Global Pose: ", global_pose)

def DetermineControllerSpeeds(pose_current_list, pose_goal_list):
    kp_x, kp_y, kp_theta = 0.05, 0.05, 0.05
    threshold_distance = 0.08
    threshold_angle = np.pi/8

    pose_current = np.array([[pose_current_list[0]], [pose_current_list[1]], [pose_current_list[3]]], np.float32) 
    pose_goal = np.array([[pose_goal_list[0]], [pose_goal_list[1]], [pose_goal_list[3]]], np.float32) 
    
    # Obtain delta of goal and current poses (direction vector in the global frame)
    delta_pose = pose_goal - pose_current    
    #print(delta_pose)
    
    # Calculate Euclidian distance from quadcopter to goal
    linear_distance =  np.sqrt(np.square(delta_pose[0][0]) + np.square(delta_pose[1][0]))
    print("Linear Distance: ", linear_distance)

    # Check if quadcopter is near the goal
    if linear_distance <= threshold_distance:
        diff_theta = delta_pose[2][0]
        
        # Check if the quadcopter is almost oriented as the goal to stop moving it
        if abs(diff_theta) <= threshold_angle:
            return [0.0, 0.0, 0.0, 0.0, True]
        
        # Correct orientation by considering rotation speed only 
        v_theta = kp_theta * delta_pose[2][0]
        if abs(v_theta) > 1.0:
            v_theta /= abs(v_theta) # Transform theta speed to 1 or -1 if it is too big
        return [0.0, 0.0, 0.0, v_theta, False]

    # Check current theta of quadcopter
    theta_current = pose_current[2][0]
    
    # Multiply the difference of poses times the global-to-local rotation matrix 
    rotation_matrix = np.array([[np.cos(theta_current), np.sin(theta_current), 0],[-np.sin(theta_current), np.cos(theta_current), 0],[0, 0, 1]], np.float32)
    delta_pose_local = np.matmul(rotation_matrix, delta_pose)

    # Calculate necessary x speed
    v_x = kp_x * delta_pose_local[0][0]
    if abs(v_x) > 1.0:
        v_x /= abs(v_x) # Transform x speed to 1 or -1 if it is too big

    # Calculate necessary y speed
    v_y = kp_y * delta_pose_local[1][0]
    if abs(v_y) > 1.0:
        v_y /= abs(v_y) # Transform y speed to 1 or -1 if it is too big

    #print(delta_pose_local)
    #print(rotation_matrix)
    return [v_x, v_y, 0.0, 0.0, False]

def GenerateCirclePoses(radius, num_points, angle):
    poses_list = []
    for i in range(num_points):
        px = radius * np.cos(2*np.pi*i/num_points)
        py = radius * np.sin(2*np.pi*i/num_points)
        pose = [px, py, 0.0, angle]
        poses_list.append(pose)
    return poses_list

if __name__=="__main__":
    key_listener = Listener(on_press=on_press, on_release=on_release) 
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 10)
    takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
    land = rospy.Publisher('bebop/land', Empty, queue_size = 1)
    reset = rospy.Publisher('bebop/reset', Empty, queue_size = 1)
    odom = rospy.Subscriber('bebop/odom', Odometry, get_odometry)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 1.0)
    rate = rospy.Rate(10)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    
    try:
	key_listener.start()
        #print(msg)
        #print(vels(speed,turn))
        print("STARTING")
        while(controller_on):
            print("Main loop")
            print(controller_on)
            #if(circle_mode):
            #    x = 0.2
            #    y = 0.0
            #    z = 0.0
            #    th = 0.4
            #    print('Speeds')
            #    twist = Twist()
            #    twist.linear.x = x*1; twist.linear.y = y*1; twist.linear.z = z*1;
            #    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*1
            #    while(circle_mode): 
            #        pub.publish(twist)
            #        print(x,y,z,th)
            #        rate.sleep() # Ensure the circle_mode loops 5 times per second
            if(circle_mode):
                poses_circle = GenerateCirclePoses(0.5, 8, np.pi/2)
                goal_number = 0
                pose_goal = poses_circle[0]
                while(circle_mode):
		    required_speeds = DetermineControllerSpeeds(global_pose, pose_goal)
		    if required_speeds[4] and goal_number >= len(poses_circle)-1:
                        print("FINISHED")
			# Made it to the goal
			twist = Twist()
                        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                        pub.publish(twist)
                        QuadLand()
                        circle_mode = False
                        sleep(4)
                    elif required_speeds[4] and goal_number < len(poses_circle)-1:
                        # Reached one of the points
                        goal_number += 1
                        pose_goal = poses_circle[goal_number] 
                    else:
                        # Try to reach next goal
                        twist = Twist()
                        twist.linear.x = required_speeds[0]; twist.linear.y = required_speeds[1]; twist.linear.z = 0
                        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = required_speeds[3]
                        pub.publish(twist)
                     
                
	    if(trajectory_mode):
                pose_goal = [0.80, -1.0, 0.0, global_pose[3]]
                halfWay = False
		while(trajectory_mode):
		    required_speeds = DetermineControllerSpeeds(global_pose, pose_goal)
                    print("Required speeds: ", required_speeds)
		    if required_speeds[4]:
			# Made it to the goal
			twist = Twist()
                        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                        pub.publish(twist)
                        QuadLand()
                        sleep(4)
                        print("MADE IT")
 
                        # Check if it finished the first half of trajectory
                        if not halfWay:
                            halfWay = True
                            pose_goal = [0.0, 0.0, 0.0, 0.0]
                            QuadTakeoff()
                        else:
                            # Finished both halves of trajectory
                            trajectory_mode = False
		            break
		    else:
			twist = Twist()
                        twist.linear.x = required_speeds[0]; twist.linear.y = required_speeds[1]; twist.linear.z = 0
                        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = required_speeds[3]
                        pub.publish(twist)
		    rate.sleep() # Ensure the circle_mode loops 5 times per second	
            rate.sleep()
        print("Waiting for key controller to end")
        key_listener.join()
            
    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        empty = Empty()
        land.publish(empty)
