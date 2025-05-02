#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Top Block
# GNU Radio version: 3.10.7.0

if __name__ == '__main__':
   import ctypes
   import sys
   if sys.platform.startswith('linux'):
       try:
           x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
           x11.XInitThreads()
       except:
           print("Warning: failed to XInitThreads()")
import datetime, psutil, sys, os, signal, argparse, time
from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import soapy
import matplotlib.pyplot as plt
import numpy as np
import keyboard
import pandas as pd
from datetime import datetime
import rospy
from std_msgs.msg import Float32,Float32MultiArray


class top_block(gr.top_block):

   def __init__(self, args):
       gr.top_block.__init__(self, "Top Block")

       ##################################################
       # Variables
       ##################################################
       self.samp_rate = args.sampleRate
       self.out_fname = args.outfname
       self.gain = args.gain
       self.nsamps = args.nsamps
       self.center_freq = args.CenterFreq

       ##################################################
       # Blocks
       ##################################################

       self.soapy_hackrf_source_0 = None
       dev = 'driver=hackrf'
       stream_args = ''
       tune_args = ['']
       settings = ['']

       self.soapy_hackrf_source_0 = soapy.source(dev, "fc32", 1, "hackrf=0,bias_tx=true",
                                 stream_args, tune_args, settings)
       self.soapy_hackrf_source_0.set_sample_rate(0, self   .samp_rate)
       self.soapy_hackrf_source_0.set_bandwidth(0, 0)
       self.soapy_hackrf_source_0.set_frequency(0, self.center_freq)
       self.soapy_hackrf_source_0.set_gain(0, 'AMP', False)
       self.soapy_hackrf_source_0.set_gain(0, 'LNA', min(max(self.gain, 0.0), 40.0))
       self.soapy_hackrf_source_0.set_gain(0, 'VGA', min(max(16, 0.0), 62.0))

       self.blocks_head_0 = blocks.head(gr.sizeof_gr_complex*1, self.nsamps)
       self.blocks_vector_sink_0 = blocks.vector_sink_c()
       self.blocks_correctiq_0 = blocks.correctiq()

       ##################################################
       # Connections
       ##################################################
       self.connect((self.blocks_correctiq_0, 0), (self.blocks_head_0, 0))
       self.connect((self.blocks_head_0, 0), (self.blocks_vector_sink_0, 0))
       self.connect((self.soapy_hackrf_source_0, 0), (self.blocks_correctiq_0, 0))

def get_args():
   parser = argparse.ArgumentParser()
   parser.add_argument("-s","--sampleRate",required=False,type=int,
               default=3.0e6,
               help="Sample rate, in Hz.",
            )
   parser.add_argument("-n","--nsamps",required=False,type=int,
       default=512*1024,
       help="Number of samples to collect per instant.",
            )
   parser.add_argument("-g","--gain",required=False,type=float,
       default=0,
       help="RX Gain, in dB.",
            )
   parser.add_argument("-f","--CenterFreq",type=float,
       default=909.8e6,
       help="Center frequency.",
            )    
   parser.add_argument("-o","--outfname",required=False,
       default=f'HackRF_collect_{datetime.now().strftime("%Y_%m_%d_%H_%M_%S")}_RX0',
       help="Output filepath. Default is ./HackRF_collect_YYYY_mm_DD_HH_MM_SS_RX0",
            )
   parser.add_argument("-q","--acquire",required=False,
       default=False,
       help="Flag used to activate GPS PRN acquisition. Set to True, or 1, to run acquisition after collection.",
            )
   parser.add_argument("-i","--instances",required=False,type=int,
       default=1,
       help="Number of instances to collect samples.",
            )
   args = parser.parse_args()
   return args

def main(top_block_cls=top_block, options=None):

   ### Get process ID and set to highest priority
   p = psutil.Process(os.getpid())
   if sys.platform=='win32':
       p.nice(psutil.REALTIME_PRIORITY_CLASS)
       if str(p.nice()) != "Priority.REALTIME_PRIORITY_CLASS":
           print('This script must be run with admin privileges in order to'
                 ' set priority to REALTIME!\nSetting instead'
                 ' to HIGH priority.')
   else:
       p.nice(-20)
       if int(p.nice()) != -20:
           print('This script must be run with admin privileges in order to '
                 'grant the process realtime priority.\n'
                 'Priority requested: -20\n'
                 f'Priority set: {p.nice()}')

   args = get_args()
   pub = rospy.Publisher('hrf_power',Float32MultiArray,queue_size=10)
   rospy.init_node('hrf_node')
   rate = rospy.Rate(1)
   

   rel = 0
   while rospy.is_shutdown():
        
        rel = rel + 1
        for instance in range(args.instances):
            ### Set up and run top block
            tb = top_block_cls(args)

            tb.start()
            tb.wait()

            samples = tb.blocks_vector_sink_0.data()
            I_sams = np.real(samples)
            q_sams = np.imag(samples)
            
            pwr1 = np.sum(I_sams*I_sams + q_sams*q_sams)/len(samples)
            
            mesg = [pwr1]
            stre = Float32MultiArray(data = mesg)
            rospy.loginfo(stre)
            pub.publish(stre)
            rate.sleep()

   tb.stop()
#    return samples

"""       if args.acquire:
           from pyGPSacquire import pyGPSacquire
           print('Running acquisition for RX0:')
           pyGPSacquire(samples, args.sampleRate, VERBOSE=True)

       # Save samples to file
       with open(f"{args.outfname}_instance_{instance + 1}", 'wb') as f:
           np.array(samples).tofile(f)
"""
        

if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException:
       pass
        