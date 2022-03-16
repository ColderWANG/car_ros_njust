#!/usr/bin/python3

import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from yolov5.wr_detect import c_yolo_camera_t
from red_flag import c_red_flag_t

goal_reached_flag=False
if __name__=='__main__':

    rospy.init_node("start_race")

    def move_base_result_callback(data):
        global goal_reached_flag
        print("call_Back:")
        print(data.status.text)
        if data.status.text=='Goal reached.':
            goal_reached_flag=True


    goal_topic_name='move_base_simple/goal'
    move_base_result_topic_name="move_base/result"
    goal_publisher=rospy.Publisher(goal_topic_name,PoseStamped,queue_size=10)
    move_base_result_listener=rospy.Subscriber(move_base_result_topic_name,MoveBaseActionResult,move_base_result_callback)

    mission_index=1
    timer=rospy.Rate(10)
    rospy.sleep(2)
    camera=None
    try:
        if mission_index<=3:###########################################################!@!@!!!!!!!!!!!!!!!!!!!!!!!!
            camera=c_yolo_camera_t()
            print("yolo_inited")

        if mission_index==1:# red flag
            print("waiting for red flag!")
            camera_flag=c_red_flag_t(0)
            time=rospy.get_rostime()
            last_sec=time.to_sec()
            #camera_flag.debug_window()
            flag_count=0
            while (not rospy.is_shutdown()) and rospy.get_rostime().to_sec()-time.to_sec()<30:
                speed=camera_flag.next_frame()/(rospy.get_rostime().to_sec()-last_sec)
                last_sec=rospy.get_rostime().to_sec()

                print("flag_speed:",speed,",elapsed:",rospy.get_rostime().to_sec()-time.to_sec(),
                "count:",flag_count)
                
                if speed>750:
                    flag_count+=1
                if flag_count>5:
                    break
                rospy.sleep(0.033)
            print("red_flag mission end")
            camera_flag.release()
            del camera_flag
            mission_index+=1
            
        if mission_index==2:#go!
            goal_reached_flag=False
            goal=PoseStamped()
            goal.header.stamp=rospy.get_rostime()
            goal.header.frame_id="map"
            goal.pose.position.x=95.157
            goal.pose.position.y=-18.5577
            goal.pose.position.z=0
            goal.pose.orientation.x=0
            goal.pose.orientation.y=0
            goal.pose.orientation.z=-0.06637291753268841
            goal.pose.orientation.w=0.997794886646649
            goal_publisher.publish(goal)
            print("go")
            rospy.sleep(3)
            goal_reached_flag=False
            while not rospy.is_shutdown():
                timer.sleep()
                if goal_reached_flag is True:
                    break
            mission_index+=1
            goal_reached_flag=False
            print("next_mission")

        if mission_index==3:#traffic light
            counter=0
            camera.open_camera()
            time=rospy.get_rostime()
            while (not rospy.is_shutdown()) and rospy.get_rostime().to_sec()-time.to_sec()<15:
                result=camera.run_once()
                print(result,rospy.get_rostime().to_sec()-time.to_sec())
                if 'roundGreen' in result:
                    counter+=1
                else:
                    counter=0
                if counter>=2:
                    break
                timer.sleep()
            del camera
            mission_index+=1
            
        if mission_index==4:# mid goal 1
            goal_reached_flag=False
            goal=PoseStamped()
            goal.header.stamp=rospy.get_rostime()
            goal.header.frame_id="map"
            goal.pose.position.x=198.579
            goal.pose.position.y=-161.8359832
            goal.pose.position.z=0
            goal.pose.orientation.x=0
            goal.pose.orientation.y=0
            goal.pose.orientation.z=-0.7809412356071616
            goal.pose.orientation.w=0.6246045040890754
            goal_publisher.publish(goal)
            print("go")
            rospy.sleep(3)
            goal_reached_flag=False
            while not rospy.is_shutdown():
                timer.sleep()
                if goal_reached_flag is True:
                    break
            mission_index+=1
            goal_reached_flag=False
            print("waiting 10 sec")
            rospy.sleep(10)
            print("next_mission")

        if mission_index==5:#mid goal 2
            goal_reached_flag=False
            goal=PoseStamped()
            goal.header.stamp=rospy.get_rostime()
            goal.header.frame_id="map"
            goal.pose.position.x=290.8829
            goal.pose.position.y=13.74150753
            goal.pose.position.z=0
            goal.pose.orientation.x=0
            goal.pose.orientation.y=0
            goal.pose.orientation.z=0.5896885162250958
            goal.pose.orientation.w=0.8076307657786724
            goal_publisher.publish(goal)
            print("go")
            rospy.sleep(3)
            goal_reached_flag=False
            while not rospy.is_shutdown():
                timer.sleep()
                if goal_reached_flag is True:
                    break
            mission_index+=1
            goal_reached_flag=False
            print("waiting 10 sec")
            rospy.sleep(10)
            print("next_mission")

        if mission_index==6:#mid goal 3
            goal_reached_flag=False
            goal=PoseStamped()
            goal.header.stamp=rospy.get_rostime()
            goal.header.frame_id="map"
            goal.pose.position.x=154.3399
            goal.pose.position.y=81.597
            goal.pose.position.z=0
            goal.pose.orientation.x=0
            goal.pose.orientation.y=0
            goal.pose.orientation.z=-0.7925736491197151
            goal.pose.orientation.w=0.6097761972404784
            goal_publisher.publish(goal)
            print("go")
            rospy.sleep(3)
            goal_reached_flag=False
            while not rospy.is_shutdown():
                timer.sleep()
                if goal_reached_flag is True:
                    break
            mission_index+=1
            goal_reached_flag=False
            print("waiting 10 sec")
            rospy.sleep(10)
            print("next_mission")

        if mission_index==7:#goal
            goal_reached_flag=False
            goal=PoseStamped()
            goal.header.stamp=rospy.get_rostime()
            goal.header.frame_id="map"
            goal.pose.position.x=35.14585
            goal.pose.position.y=48.256
            goal.pose.position.z=0
            goal.pose.orientation.x=0
            goal.pose.orientation.y=0
            goal.pose.orientation.z=0.9843778852817441
            goal.pose.orientation.w=0.17606867685151004
            goal_publisher.publish(goal)
            print("go")
            rospy.sleep(3)
            goal_reached_flag=False
            while not rospy.is_shutdown():
                timer.sleep()
                if goal_reached_flag is True:
                    break
            mission_index+=1
            goal_reached_flag=False
            print("end")

            
    except rospy.ROSInterruptException:
        pass