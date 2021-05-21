#!/usr/bin/env python
# license removed for brevity

# The goal for this node is to listen to TF2 for 2 specific frames
# Find the transform between them
# Publish this transform info as a stamped pose (for other nodes to subscribe to)

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs

# ref -- https://answers.ros.org/question/323075/transform-the-coordinate-frame-of-a-pose-from-one-fixed-frame-to-another/

# transfor_pose function -- this function listens to TF2,
# creates and populates a dummy stamped pose with frome_frame
# as its parent, and finds the transform between this dummy
# frame and to_frame
def transform_pose(input_pose, from_frame, to_frame):
    # **Assuming /tf2 topic is being broadcasted
    # this means assuming you have a TF2 structure open
    # ie assuming that you can open a rqt tf2 tree


    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        
        pub = rospy.Publisher('switch_transform', PoseStamped, queue_size=10)
        #rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10)
        
        rospy.loginfo(output_pose_stamped)
        pub.publish(output_pose_stamped)

        #rospy.spin()

        #return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise



if __name__ == '__main__':
    try:
        rospy.init_node('nodeC', anonymous=True)
        # This line below \/ \/ looks towards the launch file for parameters
        # Below is the structure used to create nodes in the launch file.
        # rospy.get_param('') looks towards the param name section for info
        #<node name="turtle2_tf2_broadcaster" pkg="learning_tf2" type="turtle_tf2_broadcaster.py" respawn="false" output="screen" >
        #<param name="turtle" type="string" value="turtle2" /> 
        
        #tf_param = rospy.get_param('')
        
        # populate this subscriber -- it's supposed to have
        # pose name, message type (it's a stamped pose), callback function
        # the callback function is transform_pose.
        # Since this node subscribes to some particular pose (my_pose ?),
        # it will run the callback function whenever a message is sent outfrom
        # the topic my_pose
        
        #rospy.Subscriber('', tf2_geometry_msgs.PoseStamped, transform_pose, tf_param)

        # Then just run transform_pose normally
        # you might need to modify code to only take the frame id info?
        while not rospy.is_shutdown():

            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)


            # Add lines here that gather data from the launch file -- ropsy.getparams

            # getting child frames
            stat_frame1_id = rospy.get_param('~child1_id')
            stat_frame2_id = rospy.get_param('~child2_id')

            # getting transform for frame1
            input_frame = PoseStamped()
            #input_frame = Pose()
            input_frame.position.x = rospy.get_param('frame1_position_x')
            input_frame.position.y = rospy.get_param('frame1_position_y')
            input_frame.position.z = rospy.get_param('frame1_position_z')
            input_frame.orientation.x = rospy.get_param('frame1_orientation_x')
            input_frame.orientation.y = rospy.get_param('frame1_orientation_y')
            input_frame.orientation.z = rospy.get_param('frame1_orientation_z')
            input_frame.orientation.w = rospy.get_param('frame1_orientation_w')




            transform_pose(input_frame, stat_frame1_id, stat_frame2_id)

            rospy.spin()
        
        

        
    except rospy.ROSInterruptException:
        pass