#!/usr/bin/python3

import tf
import rospy
import serial
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
import rospy
from libnmea_navsat_driver.driver import RosNMEADriver

tf_broadcaster_odom_map=tf.TransformBroadcaster(queue_size=10)
tf_broadcaster_imu_odom=tf.TransformBroadcaster(queue_size=10)
tf_broadcaster_GPS_imu=tf.TransformBroadcaster(queue_size=10)

rospy.init_node('test')
pub=rospy.Publisher("imu",Imu,queue_size=100)
pub_odom=rospy.Publisher("odom",Odometry,queue_size=100)
pub_twist=rospy.Publisher("twist",TwistWithCovarianceStamped,queue_size=100)
timer=rospy.Rate(50)
try:
    while not rospy.is_shutdown():



        data=Imu()
        data.header.frame_id="imu"
        data.header.stamp=rospy.get_rostime()
        data.angular_velocity.x=0
        data.angular_velocity.y=0
        data.angular_velocity.z=0
        data.linear_acceleration.x=0
        data.linear_acceleration.y=0
        data.linear_acceleration.z=9.81
        pub.publish(data)

        data=Odometry()
        data.header.frame_id="odom"
        data.child_frame_id="base_link"
        data.twist.twist.linear.x=0.0
        data.twist.twist.linear.y=0.0

        data.header.stamp=rospy.get_rostime()
        pub_odom.publish(data)

        data=TwistWithCovarianceStamped()
        data.header.stamp=rospy.get_rostime()
        data.twist.twist.linear.x=0.1
        pub_twist.publish(data)




        if True:
            tf_broadcaster_odom_map.sendTransform((0,
                                        0,
                                        0),
                                        (0,0,0,1),
                                        rospy.Time.now(),
                                        "odom",
                                        "map")

        if False:
            tf_broadcaster_imu_odom.sendTransform((1,
                                        0,
                                        0),
                                        (0,0,0,1),
                                        rospy.Time.now(),
                                        "base_link",
                                        "odom")

        timer.sleep()
        

except rospy.ROSInterruptException:
    exit()


