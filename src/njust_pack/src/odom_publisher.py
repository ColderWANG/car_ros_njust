#!/usr/bin/python3


import rospy
import tf
from nav_msgs.msg import Odometry

if __name__=='__main__':
    rospy.init_node('odom_publisher', anonymous=False)


    tf_broadcaster=tf.TransformBroadcaster()
    def odom_callback(data):
        global tf_broadcaster
        x=data.pose.pose.position.x
        y=data.pose.pose.position.y
        z=data.pose.pose.position.z

        translation=(x,y,z)
        x=data.pose.pose.orientation.x
        y=data.pose.pose.orientation.y
        z=data.pose.pose.orientation.z
        w=data.pose.pose.orientation.w
        rot=(x,y,z,w)
        time=data.header.stamp
        tf_broadcaster.sendTransform(translation,rot,time,"base_footprint","odom")

    raw_odom_subscriber=rospy.Subscriber("odom",Odometry,queue_size=10,
                                        callback=odom_callback)



    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
