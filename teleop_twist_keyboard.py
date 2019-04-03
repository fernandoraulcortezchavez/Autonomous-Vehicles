#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Empty
import sys
import time
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
        'z':(1,1),
        'x':(1,1),
        'c':(1,1),
    }

# Declare global variables for topics
pub = None
takeoff = None
land = None
reset = None


def on_press(key):
    global pub
    global takeoff
    global land
    global reset
    
    if key != Key.shift_r and key != Key.shift_l:
        print('{0} pressed'.format(
        key))
        
    # Movement keys
    if key.char in moveBindings.keys():
        x = moveBindings[key][0]
        y = moveBindings[key][1]
        z = moveBindings[key][2]
        th = moveBindings[key][3]
        twist = Twist()
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        pub.publish(twist)
        print(x,y,z,th)
        
    elif key.char in speedBindings.keys():
        speed = speed * speedBindings[key][0]
        turn = turn * speedBindings[key][1]

    # Control keys
    elif key.char == 'v':
        empty = Empty()
        takeoff.publish(empty)
    elif key.char == 'b':
        empty = Empty()
        land.publish(empty)
    elif key.char == 'n':
        empty = Empty()
        reset.publish(empty)
                     

def on_release(key):
    global pub
    global takeoff
    global land
    global reset
    
    if key == Key.esc:
        # Stop listener
        return False
    if key != Key.shift_r and key != Key.shift_l:
        print('{0} release'.format(
        key))
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

if __name__=="__main__":
    global pub
    global takeoff
    global land
    global reset
    
    pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
    takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
    land = rospy.Publisher('bebop/land', Empty, queue_size = 1)
    reset = rospy.Publisher('bebop/reset', Empty, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    
    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            time.sleep(0.001)
            
    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
