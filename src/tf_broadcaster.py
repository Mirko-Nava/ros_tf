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
        rospy.init_node(
            'tf_broadcaster'  # name of the node
        )

        # initialize the tf broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)

        tf1 = {
            'name': 'tf1',
            'translation': np.array([0., 0., 0.]),
            'rotation': Rotation.from_euler('xyz', [0., 0., 0.], degrees=True),
            'parent': 'world'
        }

        tf2 = {
            'name': 'tf2',
            'translation': np.array([0., 0., 0.]),
            'rotation': Rotation.from_euler('xyz', [0., 0., 0.], degrees=True),
            'parent': 'tf1'
        }

    def broadcast(self):
        """Broadcasts pose until the node is terminated."""

        while not rospy.is_shutdown():

            # send tf messages for each link
            for link in self.links:
                self.tf_broadcaster.sendTransform(
                    link['translation'],
                    link['rotation'].as_quat(),
                    rospy.Time.now(),
                    link['name'],
                    link['parent']
                )

            # sleep until next step
            self.rate.sleep()


if __name__ == '__main__':
    broadcaster = TfBroadcaster()

    try:
        broadcaster.broadcast()
    except rospy.ROSInterruptException as e:
        pass
