#!/usr/bin/python3

import tf
import rospy
from nav_msgs.msg import Odometry

if __name__=='__main__':

    odom_publisher=rospy.Publisher("odom_output",Odometry)
    tf_broadcaster=tf.TransformBroadcaster()

    def fun_callback(data):
        global odom_publisher
        data.header.frame_id="odom_new"
        ratio=1.0
        data.pose.pose.position.x*=ratio
        data.pose.pose.position.y*=ratio
        data.pose.pose.position.z*=ratio
        x=data.pose.pose.position.x
        y=data.pose.pose.position.y
        z=data.pose.pose.position.z
        translation=(x,y,z)

        rot=(0,0,0,1)
        tf_broadcaster.sendTransform(translation,rot,rospy.Time.now(),"odom_combined","odom_repair")


        odom_publisher.publish(data)


    rospy.init_node('odom_repair', anonymous=False)

    odom_subscriber=rospy.Subscriber("odom_input",Odometry,callback=fun_callback)
    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass