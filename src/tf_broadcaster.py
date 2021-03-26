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

        # list of transformations
        self.tfs = [{
            'name': 'base',
            'translation': np.array([0., 0., .2]),
            'rotation': Rotation.from_euler('xyz', [0., 20., 0.], degrees=True),
            'parent': 'world'
        }, {
            'name': 'tf1',
            'translation': np.array([0., 0., 1.]),
            'rotation': Rotation.from_euler('xyz', [0., 20., 0.], degrees=True),
            'parent': 'base'
        }, {
            'name': 'tf2',
            'translation': np.array([1., 0., 0.]),
            'rotation': Rotation.from_euler('xyz', [0., 0., 0.], degrees=True),
            'parent': 'tf1'
        }]

    def broadcast(self):
        """Broadcasts pose until the node is terminated."""

        while not rospy.is_shutdown():

            # animate the robot by changing the rotation based on the time
            # self.tfs[1]['rotation'] = Rotation.from_euler(
            #     'xyz',
            #     [0.,  10 * (2 + np.sin(rospy.Time.now().to_nsec() / 1e9)), 0.],
            #     degrees=True
            # )

            # send tf messages for each transform
            for tf in self.tfs:
                self.tf_broadcaster.sendTransform(
                    tf['translation'],
                    tf['rotation'].as_quat(),
                    rospy.Time.now(),
                    tf['name'],
                    tf['parent']
                )

            # sleep until next step
            self.rate.sleep()


if __name__ == '__main__':
    broadcaster = TfBroadcaster()

    try:
        broadcaster.broadcast()
    except rospy.ROSInterruptException as e:
        pass
