#!/usr/bin/env python
import rospy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs


if __name__ == '__main__':
    # Initiating your node
    rospy.init_node('nodeD')

    # Creating a listener object
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Creating reference variables -- these should be set to the child ids that
    #   are in your launch files
    from_frame = 'my_frame1_child'
    to_frame = 'my_frame2_child'

    # Setting up the publisher object
    pub = rospy.Publisher('switch_transform', PoseStamped, queue_size=10)

    # Creating a PoseStamped sobject
    stamped_trans = tf2_geometry_msgs.PoseStamped()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Looks up the transform between from_frame and to_frame.
            trans = tfBuffer.lookup_transform(from_frame, to_frame, rospy.Time(1))
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            rate.sleep()
            continue


        # Populating our PoseStamped object
        stamped_trans.pose = trans


        # Publishing trans, the transform that we're looking for
        rospy.loginfo(stamped_trans)
        pub.publish(stamped_trans) 