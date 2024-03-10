# your_package_name/subscriber_node.py

import rclpy
from your_package_name.msg import YourMessage

def message_callback(msg):
    print('Received: %d' % msg.your_data)

def main():
    rclpy.init()

    node = rclpy.create_node('your_subscriber_node')

    subscriber = node.create_subscription(
        YourMessage,
        'your_topic_name',
        message_callback,
        10)

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
