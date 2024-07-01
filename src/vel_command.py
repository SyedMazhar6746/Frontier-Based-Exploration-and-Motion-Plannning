#!/usr/bin/python

import numpy as np
import rospy
import signal
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from std_msgs.msg import Float64MultiArray  

# Signal handler to stop the loop when Ctrl+C is pressed
def signal_handler(sig, frame):
    rospy.loginfo("Ctrl+C pressed. Stopping the loop...")
    rospy.signal_shutdown("Ctrl+C pressed")

def vel_move():   
    rate = rospy.Rate(100)     
    move = Float64MultiArray() 
    while not rospy.is_shutdown():     
        wl = rospy.get_param('left_wheel')     
        wr = rospy.get_param('right_wheel')   
        move.data = [wl, wr]   
        pub.publish(move) 
        rate.sleep()   

   
if __name__ == '__main__':   

    rospy.init_node("velocity_command")  
    rospy.set_param('left_wheel', 4.0)                  
    rospy.set_param('right_wheel', 4.0)      
    signal.signal(signal.SIGINT, signal_handler)  
    pub = rospy.Publisher('/kobuki/commands/wheel_velocities', Float64MultiArray, queue_size=10)  
    vel_move()   
    rospy.spin()   
 