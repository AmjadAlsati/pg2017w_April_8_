"""@package cooperative_driving_common
Utilities which extend ROS and the standard library.
"""

from rospy import Publisher, Subscriber
from rospy.msg import args_kwds_to_message


class Pubscriber:
    """! Wrapper for an advertisement and subscription to the same topic.

    Provides an interface to access the same topic via both subscription
    and advertisement. Publishing messages works similary to a Publisher.
    Messages received on the subscribed will trigger the callback. This
    however exludes messages published via this Pubscriber.
    """

    def __init__(self, topic, data_class, callback, input_queue_size=None, output_queue_size=None, latch=False):
        """! Initializes a `Pubscriber`.

        Note that the interface currently supports only a limited amount of
        options which can be passed to Publishers and Subscribers.

        Parameters
        ----------
        @param topic: The topic subscribed to/published on.
        @type topic: str
        @param input_queue_size: Subscriber queue size.
        @type input_queue_size: int
        @param output_queue_size: Publisher queue size.
        @type output_queue_size: int
        @param callbackFunction: to call with received messages.
        @type callback: function
        @param latch: If true, the last message published on this topic will be saved and sent to new subscribers
        when they: connect.
        @type latch: bool
        """
        ## Topic associated with the `Pubscriber`
        self.topic = topic
        ## Maximum number of messages currently in transmission. Used to limit `_sent_message_timestamp`'s length.
        self._maximum_messages_in_flight = input_queue_size + output_queue_size
        ## List containing timestamps of messages which are currently in flight
        self._sent_message_timestamps = []

        def internal_callback(msg):
            if msg.header.stamp not in self._sent_message_timestamps:
                callback(msg)

        ## Used internaly to publish on `_topic`
        self._publisher = Publisher(
            topic, data_class, queue_size=output_queue_size, latch=latch)
        ## Used internaly to subscribe to the `_topic`
        self._subscriber = Subscriber(
            topic, data_class, queue_size=input_queue_size, callback=internal_callback)

    def publish(self, *args, **kwds):
        """! Publish a message on the topic associated with this `Pubscriber`.
        Can be called either with a message or constructor arguments for a new message.

        @param args: L{Message} instance, message arguments, or no args if keyword arguments are used
        @param kwds: Message keyword arguments. If kwds are used, args must be unset
        """
        data = args_kwds_to_message(self._publisher.data_class, args, kwds)
        self._sent_message_timestamps = self._sent_message_timestamps[1 if len(
            self._sent_message_timestamps) > self._maximum_messages_in_flight else 0:] + [data.header.stamp]
        self._publisher.publish(data)
