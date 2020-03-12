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

    def add_link(self, name, length, direction):
        """Adds a link to the fictitious robot."""

        # define link position and rotation
        link_position = direction.as_quat()[:-1] * length
        link_rotation = Rotation.from_euler('xyz', [0, 0, 0])

        # select the parent as the last link added or world
        if len(self.links) == 0:
            parent_name = 'world'
        else:
            parent = self.links[-1]
            parent_name = parent['name']

        # create link
        link = {
            'name': name,
            'parent': parent_name,
            'translation': link_position,
            'rotation': link_rotation
        }

        # append link to the list
        self.links.append(link)

    def move_link(self, idx, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        link = self.links[idx]

        # create rotation matrix from euler angles
        rot_mat = np.eye(4)
        rot_mat[:3, :3] = Rotation.from_euler(
            'xyz', [roll, pitch, yaw]).as_dcm()

        # create translation matrix
        trans_mat = np.eye(4)
        trans_mat[:3, -1] = [x, y, z]

        # compose matrix into transform
        # note: np.matmul = @ = matrix multiplication
        transform = np.matmul(trans_mat, rot_mat)

        # create affine vector from translation
        loc = np.ones((4, 1))
        loc[:3, 0] = link['translation']

        # update translation and rotation accordingly
        link['translation'] = np.matmul(transform, loc)[:-1].squeeze()
        link['rotation'] *= Rotation.from_euler('xyz', [roll, pitch, yaw])

    def broadcast(self):
        """Broadcasts pose until the node is terminated."""

        # create link structure, a.k.a. the robot arm
        self.add_link('link1', 2, Rotation.from_rotvec([0, 0, 1]))
        self.add_link('link2', 1, Rotation.from_rotvec([1, 0, 0]))
        self.add_link('end_effector', 1, Rotation.from_rotvec([1, 0, 0]))

        # move link(based on index in the list)
        self.move_link(1, z=.2, pitch=.6)
        self.move_link(2, z=-.1, pitch=.2)

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

            self.move_link(2, roll=.1, yaw=-.1)

            # sleep until next step
            self.rate.sleep()


if __name__ == '__main__':
    broadcaster = TfBroadcaster()

    try:
        broadcaster.broadcast()
    except rospy.ROSInterruptException as e:
        pass
