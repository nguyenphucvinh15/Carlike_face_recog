#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from faceTrackingGazebo.msg import nav_signal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray     

import time

def callback (msg):
    print ("sdfsdf")
    rospy.loginfo ("signal_c:%d", msg.signal)
    

def main_task ( ):
    rospy.init_node ('main_task', anonymous=True)
    s1 = nav_signal()   
    # rospy.Subscriber ("Navigaton_signal", nav_signal,callback)
    s3 = PoseWithCovarianceStamped()
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    check = True
    check_nam = True
    nam = False
    nu = False
    get_nu = False
    get_nam = False
    delay = False
    hieu = 0.2
    oreo = 0.1
    hieu2 = 0.3

    s4 = GoalStatusArray()
    
    while not rospy.is_shutdown():
        s1 = rospy.wait_for_message("Navigaton_signal", nav_signal)
        s2 = PoseStamped()
        s2.header.stamp = rospy.Time.now()
        s2.header.frame_id = "map"
        s3 = rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped)
        

        if check and not nu:
            s2.pose.position.x = 4.397176742553711
            s2.pose.position.y = 2.5868980884552        
            s2.pose.orientation.z = -0.03512595572980609
            s2.pose.orientation.w = 0.9993828932066366
        print(s3.pose.pose.position.x)
        print(s3.pose.pose.position.y)
        print(s3.pose.pose.orientation.z)
        print(s3.pose.pose.orientation.w)
        print('\n')

        if s1.signal == 1:
            print(abs (s3.pose.pose.position.x - s2.pose.position.x), abs (s3.pose.pose.position.y - s2.pose.position.y), abs (s3.pose.pose.orientation.z - s2.pose.orientation.z), abs (s3.pose.pose.orientation.w - s2.pose.orientation.w))
            if abs (s3.pose.pose.position.x - s2.pose.position.x) <= hieu2 and abs (s3.pose.pose.position.y - s2.pose.position.y) <= hieu2 and abs (s3.pose.pose.orientation.z - s2.pose.orientation.z) <= hieu and abs (s3.pose.pose.orientation.w - s2.pose.orientation.w) <= oreo:
                if get_nu == False: 
                    print ('Waiting for luggages...')
                    if nu == False: 
                        delay = True
                    check = False
                nu = True

        
            
        print(nu)
        print(get_nu)

        if nu == True and get_nu == False:
            if s1.signal == 1:
                if delay == True: 
                    time.sleep(10)
                    delay = False

            s2.pose.position.x = 11.317031860351562
            s2.pose.position.y = 1.9289944171905518
            s2.pose.orientation.z = 0.005188368265885695
            s2.pose.orientation.w = 0.9999865403267874

            print(abs (s3.pose.pose.position.x - s2.pose.position.x), abs (s3.pose.pose.position.y - s2.pose.position.y), abs (s3.pose.pose.orientation.z - s2.pose.orientation.z), abs (s3.pose.pose.orientation.w - s2.pose.orientation.w))
            if abs (s3.pose.pose.position.x - s2.pose.position.x) <= hieu2 and abs (s3.pose.pose.position.y - s2.pose.position.y) <= hieu2 and abs (s3.pose.pose.orientation.z - s2.pose.orientation.z) <= hieu and abs (s3.pose.pose.orientation.w - s2.pose.orientation.w) <= oreo:
                get_nu = True
                nu = False
                check = True
        

################## Nam



        if nu  and get_nu and check_nam and not nam:
            s2.pose.position.x = 4.49432373046875
            s2.pose.position.y = -2.619346857070923     
            s2.pose.orientation.z = 0.027596359797388184
            s2.pose.orientation.w = 0.9996191479388203

        if s1.signal == 0:
            print(abs (s3.pose.pose.position.x - s2.pose.position.x), abs (s3.pose.pose.position.y - s2.pose.position.y), abs (s3.pose.pose.orientation.z - s2.pose.orientation.z), abs (s3.pose.pose.orientation.w - s2.pose.orientation.w))
            if abs (s3.pose.pose.position.x - s2.pose.position.x) <= hieu2 and abs (s3.pose.pose.position.y - s2.pose.position.y) <= hieu2 and abs (s3.pose.pose.orientation.z - s2.pose.orientation.z) <= hieu and abs (s3.pose.pose.orientation.w - s2.pose.orientation.w) <= oreo:
                if get_nam == False:
                    print ('Waiting for luggages...')
                    if nam == False: 
                        delay = True
                    check_nam = False
                nam = True

        print('nam: ', nam)
        print(get_nam)
        if nam == True and get_nam == False:
            if s1.signal == 0:
                if delay == True: 
                    time.sleep(10)
                    delay = False

            s2.pose.position.x = 11.572015762329102
            s2.pose.position.y = -2.1392040252685547
            s2.pose.orientation.z = 0.002816052179382668
            s2.pose.orientation.w = 0.9999960349172006

            print(abs (s3.pose.pose.position.x - s2.pose.position.x), abs (s3.pose.pose.position.y - s2.pose.position.y), abs (s3.pose.pose.orientation.z - s2.pose.orientation.z), abs (s3.pose.pose.orientation.w - s2.pose.orientation.w))
            if abs (s3.pose.pose.position.x - s2.pose.position.x) <= 0.4 and abs (s3.pose.pose.position.y - s2.pose.position.y) <= hieu2 and abs (s3.pose.pose.orientation.z - s2.pose.orientation.z) <= hieu and abs (s3.pose.pose.orientation.w - s2.pose.orientation.w) <= oreo:
                nam = False
                check_nam = True
                get_nam = True    

        if nam and nu and get_nam and get_nu:
            s2.pose.position.x = 0.0401660038525
            s2.pose.position.y = 0.0202300842876
            s2.pose.orientation.z = 0.00119534656567
            s2.pose.orientation.w = 0.999999285573


        pub.publish(s2)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        main_task()
    except rospy.ROSInterruptException:
        pass
        
