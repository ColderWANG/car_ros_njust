#!/usr/bin/python3
#將GPS的odom坐標減去offset

import rospy
from nav_msgs.msg import Odometry

rospy.init_node('gps', anonymous=False)

x_offset=rospy.get_param("~x_offset",0.0)
y_offset=rospy.get_param("~y_offset",0.0)

GPS_odom_publisher=rospy.Publisher("GPS_odom",Odometry,queue_size=100)
def raw_odom_callback(data):
    #data=Odometry(data)
    data.pose.pose.position.x-=x_offset
    data.pose.pose.position.y-=y_offset
    data.pose.pose.position.z=0

    conv=list(data.pose.covariance)
    conv[6*0+0]=10.0#conv_x
    conv[6*1+1]=10.0#conv_y
    conv[6*2+2]=30.0#conv_z
    data.pose.covariance=tuple(conv)
    GPS_odom_publisher.publish(data)

raw_odom_subscriber=rospy.Subscriber("GPS_raw_odom",Odometry,queue_size=100,
                                      callback=raw_odom_callback)



try:
    while not rospy.is_shutdown():
        rospy.spin()
except rospy.ROSInterruptException:
    pass



