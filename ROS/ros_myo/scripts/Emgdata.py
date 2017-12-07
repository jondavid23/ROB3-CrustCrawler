#!/usr/bin/env python

# The line above defines this script as an exacutable
# It is needed here because we run the script
"""
Script that subscribes to the myo_emg topic and prints
the value of the EMG sensors.

Author: Jon David Ragnarsson <jragna16 at student.aau.dk>
"""
from __future__ import print_function

from random import randrange
from time import sleep
import os

import rospy
from ros_myo.msg import EmgArray
from std_msgs.msg import UInt16MultiArray

# This function is just like the publisher
# it makes the EmgArray message, only used for
# tesing purposes
def Emgtest(): # define the function Emgtest that takes no arguments
    while True:
        emg_arr = EmgArray() # Create an instance of the EmgArray message

        #Make an array and fill it with fake emg signals
        emg_values = []
        # Fill up the array
        for i in range(8):
            emg_values.append(randrange(0, 1024))
        # Put the array into the message instance
        emg_arr.data = emg_values

        # Wait, to avoid overwriting
        sleep(0.5)
        os.system('clear')

if __name__ == '__main__':
    # Emgtest()

    # Initialize the node
    rospy.init_node('emg_data')

    # Define the publisher
    emgPub = rospy.Publisher('~mat_emg', UInt16MultiArray, queue_size=1)

    # A callback function that takes the message from the subscriber as argument
    def emg_cb(ea):

        emg_multi = UInt16MultiArray()

        # Make an array to fill with the new message
        emg_dat = []

        # Iterate through the incoming data
        for idx, d in enumerate(ea.data):
            emg_dat.append(d) # Put the info in the emg_data array
                                    # emg_dat.append((idx, d))
        emg_multi.data = emg_dat

        # Then publish the data as an MultiArray
        emgPub.publish(emg_multi) # ( label, data )

        # Print the values
        for emg in ('EMG values:', emg_dat):
            print(emg)
            # rospy.loginfo("Printing EMG values...")
        print("\n\n\n\n\n\n\n\n\n\n\n\n")

    emg_sub = rospy.Subscriber('/myo_raw/myo_emg',
                               EmgArray,
                               emg_cb,
                               queue_size=1)

    rospy.loginfo("Awaiting publications...")
    rospy.spin()
