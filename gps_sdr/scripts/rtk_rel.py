#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32,Float32MultiArray
import serial
from pyrtcm import RTCMReader
from pyubx2 import UBXMessage, SET, UBXReader

def rtk_rel_pos():

        pub = rospy.Publisher('relative_pos',Float32MultiArray, queue_size=10)
        rospy.init_node("gps_node",anonymous=False)
        rate = rospy.Rate(5)

        com_rover = "/dev/ttyUSB1" #USB0
        baud_rover = 38400 #38400
        stream_rover = serial.Serial(com_rover , baud_rover )

        com_base = "/dev/ttyUSB0"
        baud_base = 57600 
        stream_base = serial.Serial(com_base , baud_base)

        ubr = UBXReader(stream_rover)    
        rtr = RTCMReader(stream_base)
        
        while not rospy.is_shutdown():
            
            (raw_data_base, parsed_data_base) = rtr.read()
            # print(parsed_data_base)
            try:
                rtcm_to_be_sent = parsed_data_base.serialize()
                stream_rover.write(rtcm_to_be_sent)
                print("sent")
            except AttributeError:
                print('failed to send rtcm')
            del parsed_data_base
                
            (raw_data_rover, parsed_data_rover) = ubr.read()

            try:
                msg_type = parsed_data_rover.identity
                print(msg_type)
                
            except AttributeError:
                print(f"error here, data type of rover data was {type(parsed_data_rover)}")
                continue

            if msg_type=='NAV-RELPOSNED':
                msg_to_send = [parsed_data_rover.relPosN, parsed_data_rover.relPosE,parsed_data_rover.relPosD,parsed_data_rover.carrSoln]
                msg_obj = Float32MultiArray(data = msg_to_send)

                rospy.loginfo(msg_obj)
                pub.publish(msg_obj)
                rate.sleep()
                del parsed_data_rover

            else: 
                continue    
     
        # del parsed_data_base
        del parsed_data_rover, raw_data_rover
        stream_base.close()
        stream_rover.close() 

if __name__=='__main__':

    try:
        rtk_rel_pos()

    except rospy.ROSInterruptException:
        pass 


    


