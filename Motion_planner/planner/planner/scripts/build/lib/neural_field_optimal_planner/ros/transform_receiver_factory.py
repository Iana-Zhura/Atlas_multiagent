import tf2_ros

from .transform_receiver import TransformReceiver


class TransformReceiverFactory(object):
    def __init__(self):
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)

    def get_transform_receiver(self, base_frame, parent_frame):
        return TransformReceiver(self._buffer, base_frame, parent_frame)
