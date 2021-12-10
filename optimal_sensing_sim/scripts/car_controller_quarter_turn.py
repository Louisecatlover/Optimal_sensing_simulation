#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control Your Turtlebot3!
---------------------------
Moving around:
up  : k       w
          a       d
down: m       s

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(x_vel, y_vel, z_vel, angular_vel):
    return "linear_vel %.1f %.1f %.1f\tangular_vel %.1f "% (x_vel, y_vel, z_vel, angular_vel)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('car teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    '''
    status = 0 
    target_linear_vel = 0
    target_angular_vel = 0
    control_linear_vel = 0
    control_angular_vel = 0
    '''
    x_vel = 0
    y_vel = 0
    z_vel = 0
    angular_vel = 0
    print msg

    twist = Twist()

    rospy.sleep(0.00001)
    start_t = rospy.get_time()    
    
    while not rospy.is_shutdown():
        current_t = rospy.get_time()
        duration = current_t - start_t
        print duration
        key = getKey()

        if duration <= 10 :
            x_vel = 0.5      
        elif 10 < duration <=15 :
            angular_vel = -0.1
        elif 15 < duration <=35 :  
            angular_vel = 0.0
        elif 35 < duration <=40 :
            angular_vel = 0.1
        elif 40 < duration <=50 :
            angular_vel = 0.0
        elif 50 < duration :
            x_vel = 0           
        
        
        twist.linear.x = x_vel; twist.linear.y = y_vel; twist.linear.z = z_vel
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angular_vel
        pub.publish(twist)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
