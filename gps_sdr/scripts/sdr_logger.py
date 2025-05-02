#!/usr/bin/env python3

import rospy
from rtlsdr import *
from pylab import *
from std_msgs.msg import Float32,Float32MultiArray
import numpy as np
import matplotlib
matplotlib.use('Agg')

def talker():

    pub = rospy.Publisher('sdr_power', Float32MultiArray, queue_size=10)
    rospy.init_node('sdr_node')
    rate = rospy.Rate(5)

    sdr = RtlSdr()
    sdr.sample_rate = 2.4e6
    sample_rate = sdr.sample_rate
    sdr.center_freq = 909.80e6
    center_freq = sdr.center_freq
    sdr.gain = 10

    while not rospy.is_shutdown():

        samps = sdr.read_samples(256*1024)
        # Pxx,freqs = psd(samps, NFFT = 1024, Fs = sample_rate/1e6, Fc = center_freq/1e6)

        real_sq = samps.real*samps.real
        imag_sq = samps.imag *samps.imag
        Power1 = sum(real_sq + imag_sq)/len(samps)
        # Power2 = np.trapz(Pxx[299:728],freqs[299:728])

        mesg2 = [Power1] #,Power2]
        stre = Float32MultiArray(data = mesg2) #"stiring from the SECOND node"
        rospy.loginfo(stre)
        pub.publish(stre)
        rate.sleep()
        
if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
