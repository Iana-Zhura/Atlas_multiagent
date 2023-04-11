# coding=utf-8

import rospy


class RobotState(object):
    def __init__(self, transform_receiver_factory, base_frame="hermesbot", parent_frame="map"):
        self._transform_receiver = transform_receiver_factory.get_transform_receiver(base_frame, parent_frame)

    @property
    def position(self):
        return self._transform_receiver.get_transform()

    @property
    def current_time(self):
        return rospy.get_time()
