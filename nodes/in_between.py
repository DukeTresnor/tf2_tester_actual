#!/usr/bin/env python
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
    pub = rospy.Publisher('new_chatter', String, queue_size=10)
    rospy.init_node('in_between', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        mug_str = "mugs are fun %s" % rospy.get_time()
        rospy.loginfo(mug_str)
        pub.publish(mug_str)
        rate.sleep()

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)



def listener():
    
    # listener = tf2.TransformListener()
    # use /\ /\ this somewhere here?

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    # old subscriber
    #rospy.init_node('in_between', anonymous=True)

    # new subscriber
    rospy.Subscriber('poser', Pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
        listener()
    except rospy.ROSInterruptException:
        pass
