#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publish_stream():
    # Initialize the ROS node
    rospy.init_node('stream_publisher_node', anonymous=True)

    # Create a publisher for the stream. 'my_stream_topic' is the topic name, and String is the message type.
    stream_publisher = rospy.Publisher('my_stream_topic', String, queue_size=10)

    # Set the publishing rate (e.g., 1 Hz)
    rate = rospy.Rate(1)

    # Main loop to publish the stream
    while not rospy.is_shutdown():
        # Your logic to generate the stream of information
        stream_data = "This is a stream message"

        # Publish the stream data
        stream_publisher.publish(stream_data)

        # Sleep to maintain the desired publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_stream()
    except rospy.ROSInterruptException:
        pass
