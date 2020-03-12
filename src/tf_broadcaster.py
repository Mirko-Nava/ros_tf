#!/usr/bin/env python

import tf
import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped


class TfBroadcaster:

    def __init__(self):
        """Initialization."""

        # initialize the node

        # initialize the tf broadcaster

        # set node update frequency in Hz

    def broadcast(self):
        """Broadcasts pose until the node is terminated."""

        while not rospy.is_shutdown():

            # send tf messages

            # sleep until next step
            self.rate.sleep()


if __name__ == '__main__':
    broadcaster = TfBroadcaster()

    try:
        broadcaster.broadcast()
    except rospy.ROSInterruptException as e:
        pass
