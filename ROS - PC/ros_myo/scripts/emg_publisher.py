#!/usr/bin/env python

"""
Script that publishes fake values of the EmgArray message type

Author: Jon David Ragnarsson <jragna16 at student.aau.dk>
"""

import rospy
from ros_myo.msg import EmgArray
from random import randrange

# define our publisher function
def emg_pub():
    # Initialize the node
    rospy.init_node('myo_raw')

    # Define the publisher
    emgPub = rospy.Publisher('~myo_emg', EmgArray, queue_size=1)

    # Rate used to slow the publishing rate
    rate = rospy.Rate(10) # 10hz

    # While ros is running, continue
    while not rospy.is_shutdown():
        # Create an instance of the message EmgArray
        emg_arr = EmgArray()
        #Make an array and fill it with fake emg signals
        emg_values = []
        for i in range(8):
            emg_values.append(randrange(0, 1024))
        emg_arr.data = emg_values
        # Publish the array
        emgPub.publish(emg_arr.data, 0)
        # Sleep for the rate time
        rate.sleep()

if __name__ == '__main__':
    # Print out that the publisher has started
    rospy.loginfo('Publisher started!')

    # Try to run the function, if not then pass
    try:
        emg_pub()
    except rospy.ROSInterruptException:
        pass
