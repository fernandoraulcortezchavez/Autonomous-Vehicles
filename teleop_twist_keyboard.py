#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import numpy as np
import cv2
from time import sleep
from pynput.keyboard import Key, Listener
import RedFrameDetector

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
bridge = CvBridge()

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

def get_image(ros_img):
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_img, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    cont = RedFrameDetector.FindRedFrame(cv_image)
    RedFrameDetector.DrawRedFrame(cont, cv_image)

    #(rows, cols, channels) = cv_image.shape
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    # print(cv_image.shape)
    return

def DetermineControllerSpeeds(pose_current_list, pose_goal_list):
    if pose_goal_list[2] == 1:
        theta_current = pose_current_list[3] 
        theta_goal = pose_goal_list[3]
        return TurnPID(theta_current, theta_goal)
    #kp_x, kp_y, kp_theta = 0.05, 0.05, 0.05
    kp_x, kp_y, kp_theta = 0.068, 0.068, 0.25
    threshold_distance = 0.20
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
    v_y = kp_y * delta_pose_local[1][0]
    max_speed = max(abs(v_x), (v_y))
    if max_speed > 1.0:
        v_x /= max_speed
        v_y /= max_speed

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

def GenerateCircuitPoses(gpose):
    x0, y0, z0, theta0 = gpose[0], gpose[1], gpose[2], gpose[3]
    reference_pose = [x0, y0, theta0]
    pose1 = [1.5, 0, 0, theta0]
    pose2 = [3.5, 0, 0, theta0]
    pose3 = [3.5, -1, 0, theta0]
    pose4 = [0, 0, 1, np.unwrap([theta0+np.pi])[0]]
    pose5 = [2, -1, 0, np.unwrap([theta0+np.pi])[0]]
    pose6 = [0, -1, 0, np.unwrap([theta0+np.pi])[0]]
    pose7 = [0, 0, 0, theta0]
    relative_poses_list = [pose1, pose2, pose3, pose4, pose5, pose6, pose7]
    print(relative_poses_list)
    print("Generated local points")
    rotation_matrix = np.array([[np.cos(theta0), -np.sin(theta0), 0],[np.sin(theta0), np.cos(theta0), 0],[0, 0, 1]], np.float32)
    global_poses_list = []
    for i in range(len(relative_poses_list)):
        # Rotation matrix local to global
        prod = np.matmul(rotation_matrix, relative_poses_list[i][0:3])
        new_global_pose = prod + reference_pose
        xg, yg, thetag = new_global_pose[0], new_global_pose[1], new_global_pose[2]
        cur_pose = [xg, yg, 0.0, thetag]
        global_poses_list.append(cur_pose)
    print("Global goal poses")
    print(global_poses_list)
    return global_poses_list 

def GenerateCircuitPoints(gpose):
    x0, y0, z0, theta0 = gpose[0], gpose[1], gpose[2], gpose[3]
    pose1 = [4.1+x0, 0+y0, 0, theta0]
    pose2 = [4.1+x0, 0+y0, 0, theta0]
    pose3 = [4.1+x0, -3+y0, 0, theta0]
    pose4 = [4.1+x0, -3+y0, 1, np.unwrap([np.pi])[0]] #turn
    pose5 = [2+x0, -3+y0, 0, np.unwrap([theta0+np.pi])[0]]
    pose6 = [0+x0, -3+y0, 0, np.unwrap([theta0+np.pi])[0]]
    pose7 = [0+x0, 0+y0, 0, theta0]
    poses_list = [pose1, pose2, pose3, pose4, pose5, pose6, pose7]
    print("poses_list")
    print(poses_list)
    return poses_list

def TurnPID(theta_current, theta_goal):
    kp_theta = 0.3
    threshold_angle = np.pi/15
# Check if the quadcopter is almost oriented as the goal to stop moving it
    if abs(diff_theta) <= threshold_angle:
        return [0.0, 0.0, 0.0, 0.0, True]

# Correct orientation by considering rotation speed only 
    v_theta = kp_theta * (theta_goal - theta_current)
    if abs(v_theta) > 1.0:
        v_theta /= abs(v_theta) # Transform theta speed to 1 or -1 if it is too big
        return [0.0, 0.0, 0.0, v_theta, False]

if __name__=="__main__":
    key_listener = Listener(on_press=on_press, on_release=on_release) 
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 10)
    takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
    land = rospy.Publisher('bebop/land', Empty, queue_size = 1)
    reset = rospy.Publisher('bebop/reset', Empty, queue_size = 1)
    odom = rospy.Subscriber('bebop/odom', Odometry, get_odometry)
    #camera = rospy.Subscriber('bebop/image_raw', Image, get_image) 
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 1.0)
    rate = rospy.Rate(8)
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
                print("Trajectory")
                poses_circuit = GenerateCircuitPoints(global_pose)
                goal_number = 0
                pose_goal = poses_circuit[0]
                while(trajectory_mode):
		    required_speeds = DetermineControllerSpeeds(global_pose, pose_goal)
		    if required_speeds[4] and goal_number >= len(poses_circuit)-1:
                        print("FINISHED")
			# Made it to the goal
			twist = Twist()
                        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                        pub.publish(twist)
                        QuadLand()
                        circle_mode = False
                        sleep(4)
                    elif required_speeds[4] and goal_number < len(poses_circuit)-1:
                        # Reached one of the points
                        goal_number += 1
                        pose_goal = poses_circuit[goal_number] 
                    else:
                        # Try to reach next goal
                        twist = Twist()
                        if goal_number < 4:
                            twist.linear.x = required_speeds[0]; twist.linear.y = required_speeds[1]; twist.linear.z = 0
                            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = required_speeds[3]
                        else:
                            twist.linear.x = -1*required_speeds[0]; twist.linear.y = -1*required_speeds[1]; twist.linear.z = 0
                            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = required_speeds[3]
                        pub.publish(twist)
              

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
