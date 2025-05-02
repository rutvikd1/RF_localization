#!/usr/bin/env python3 
import rospy
from std_msgs.msg import Float32MultiArray
import rosbag
import message_filters
import pandas as pd
from datetime import datetime

def callback(position,power):
    filename = "merged_data_150724_trial_run1.csv"
    print("here")

    pos_dat = position.data
    pow_dat = power.data

    rospy.loginfo(rospy.get_caller_id() + f"Total message I get is {pos_dat + pow_dat} ")

    # rospy.loginfo(rospy.get_caller_id() + f"postion message I get is {pos_dat} ")
    
    now1= datetime.now()
    data0 = {"ET":[rospy.get_time()],
            "North":[pos_dat[0]],
            "East":[pos_dat[1]],
            "Down":[pos_dat[2]],
            "Solution":[pos_dat[3]],
            "Power1":[pow_dat[0]]}
    data1 = pd.DataFrame(data0,index = [0])


    data1.to_csv(filename,mode = "a",index = False,header = False)
    
                 
if __name__ == '__main__':
    print("Started")
    rospy.init_node("comb_subscriber")
        
    position = message_filters.Subscriber("relative_pos",Float32MultiArray)
    power = message_filters.Subscriber("hrf_node",Float32MultiArray)

    ts = message_filters.ApproximateTimeSynchronizer([position,power],queue_size=10,slop=0.08,allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()
