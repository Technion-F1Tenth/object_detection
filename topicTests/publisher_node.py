# your_package_name/publisher_node.py

import rclpy
from your_package_name.msg import YourMessage
from std_msgs.msg import String

def main():
    rclpy.init()

    node = rclpy.create_node('your_publisher_node')

    publisher = node.create_publisher(YourMessage, 'your_topic_name', 10)

    msg = YourMessage()
    msg.your_data = 42

    while rclpy.ok():
        node.get_logger().info('Publishing: %d' % msg.your_data)
        publisher.publish(msg)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
