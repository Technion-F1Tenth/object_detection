#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose

from .object_detection import mainObjectDetection


class VisionNode(Node):
    """ 
    The opponent car detection node
    """
    def __init__(self):
        super().__init__('vision_node')
        # Topics & Subs, Pubs

        vision_topic = '/vision'
        ### add more topics if needed

        self.declare_parameters(
            namespace='',
            parameters=[
                ('vision_topic', '/vision'),
            ]
        )

        self.detector = mainObjectDetection.ObjectDetection(debug = False)
        # TODO: consider using PoseStamped instead of Pose - contains some metadata like time and sequence.
        self.vision_pub = self.create_publisher(Pose, self.get_parameter('vision_topic').value, 10)
        ### publish or subscribe more topics if needed        

    def generate_pose(self, xyz, orientation = None):
        pose = Pose()
        # TODO: choose a coordinate system! right now it's relative to the center of the camera,
        #       where x is going downwards, y to the right and z is the depth.
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])

        ## these values SHOULD represent no rotation. 
        pose.orientation.x = float(0)
        pose.orientation.y = float(0)
        pose.orientation.z = float(0)
        pose.orientation.w = float(1)

        if orientation:
            # orientation: xyz is a vector in quaternion rotation axis, and w is the rotation angle.
            pass

        return pose

    def vision_main(self):
        ### TODO: stop on some event?
        while True:
            xyz = self.detector.runObjectDetection(ros = True)
            if xyz is None:
            	xyz = [-1,-1,-1]
            pose = self.generate_pose(xyz = xyz)
            self.vision_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    print("Vision Initialized")

    vision_node = VisionNode()
    vision_node.vision_main()

    rclpy.spin(vision_node)

    vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
