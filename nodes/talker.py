#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic


# maybe tf2 publishers and subscribers need to be structured using TransformBroadcaster()?
# look towards the power point presentation


import rospy
from std_msgs.msg import String

# importing tf2
import tf2_ros
import tf2_geometry_msgs.msg
from geometry_msgs.msg import Pose
import tf_conversions

# other imports
import math

def talker():
    # states that the talker node is publishing to the chatter topic using the
    # message string, with class std_msg.String -- (this line sets up class instance?)
    #pub = rospy.Publisher('chatter', String, queue_size=10) 
    
    # new
    pub = rospy.Publisher('poser', Pose, queue_size=10)
    br = tf2_ros.TransformBroadcaster()
    t = tf2_geometry_msgs.msg.TransformStamped()

    # states that the talker node is publishing to the mover topic using the
    # message pose_change, with class tf2_msg.msg.Pose
    # Tells ropsy the name of your node (talker) that is broadcasting
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # Defining the actual string that talker is broadcasting to chatter
        hello_str = "hello world %s" % rospy.get_time()
        
        # Defining pose being published to poser topic
        
        
        rospy.loginfo(hello_str)
        # Publishing string hello_str to the chatter topic
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$