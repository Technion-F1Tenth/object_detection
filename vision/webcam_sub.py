
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import PoseStamped
import cv2 # OpenCV library
from cv2 import aruco
import numpy as np
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('vision')

# Load the camera matrix and distortion coefficients from YAML file
fs = cv2.FileStorage(package_share_directory+"/camera_calibration.yml", cv2.FILE_STORAGE_READ)
camera_matrix = fs.getNode("camera_matrix").mat()
dist_coeffs = fs.getNode("dist_coeff").mat()
fs.release()
print(type(camera_matrix))
print(dist_coeffs)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
# Define the Aruco dictionary
#aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
#parameters = aruco.DetectorParameters_create()
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/color/image_raw', 
      self.listener_callback, 
      10)
    self.publisher = self.create_publisher(PoseStamped, 'pose_msg', 10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
     # Detect the Aruco marker
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        # Estimate the pose of the Aruco marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        
        # Add the translation and rotation vectors to the video feed
        tvec_text = "Translation vector: ({:.2f}, {:.2f}, {:.2f})".format(tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2])
        rvec_text = "Rotation vector: ({:.2f}, {:.2f}, {:.2f})".format(rvecs[0][0][0], rvecs[0][0][1], rvecs[0][0][2])
        cv2.putText(frame, tvec_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, rvec_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Visualize the pose of the Aruco marker
        frame = cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1)
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'camera'
        pose_msg.pose.position.x = round(tvecs[0][0][0], 3)
        pose_msg.pose.position.y = round(tvecs[0][0][1], 3)
        pose_msg.pose.position.z = round(tvecs[0][0][2], 3)
        pose_msg.pose.orientation.x = round(rvecs[0][0][0], 3)
        pose_msg.pose.orientation.y = round(rvecs[0][0][1], 3)
        pose_msg.pose.orientation.z = round(rvecs[0][0][2], 3)
        pose_msg.pose.orientation.w = 1.0
        self.publisher.publish(pose_msg)

    
    # Display image
    cv2.imshow("camera", frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
