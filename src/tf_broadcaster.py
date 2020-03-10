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

        # initialize list of links
        self.links = []

    def add_link(self, name, length):
        """Adds a link to the fictitious robot."""

        # generate random direction
        direction = np.random.rand(3) - .5
        direction[2] = np.abs(direction[2]) + .1
        direction /= np.linalg.norm(direction)

        # define link position and rotation
        link_position = direction * length
        link_rotation = Rotation.from_rotvec(direction).as_quat()

        # selects the parent as the last link added
        if len(self.links) == 0:
            parent = 'world'
        else:
            parent = self.links[-1]['name']

        # create link
        link = {
            'name': name,
            'parent': parent,
            'translation': link_position,
            'rotation': link_rotation
        }

        # append link to the list
        self.links.append(link)

        return 'link %s added' % name

    def broadcast(self):
        """Broadcasts pose until the node is terminated."""

        self.add_link('link1', 1)
        self.add_link('end_effector', .5)

        while not rospy.is_shutdown():

            # send tf messages for each link
            for link in self.links:
                self.tf_broadcaster.sendTransform(
                    link['translation'],
                    link['rotation'],
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
