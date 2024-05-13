#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField

import tf

import sensor_msgs.point_cloud2 as pc2



class DepthToPointCloudNode:
    def __init__(self):
        rospy.init_node('depth_to_pointcloud_node', anonymous=True)
        self.last_map_time = rospy.Time.now()
        self.bridge = CvBridge()

        self.y_min  = -0.6
        self.y_max = 0.6


        self.depth_sub = rospy.Subscriber("/turtlebot/kobuki/realsense/depth/image_depth", Image, self.depth_callback)
        self.camera_params = rospy.Subscriber("/turtlebot/kobuki/realsense/depth/camera_info", CameraInfo, self.camera_info_callback)
        self.pointcloud_pub = rospy.Publisher("/registered_scan", PointCloud2, queue_size=1)

        self.camera_intrinsics = None

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            depth_image = depth_image.astype(np.float32)  # Convert to float32
            # depth_image = depth_image.astype(np.float32)/1000 # For realworld

        except Exception as e:
            rospy.logerr("Error converting depth image: %s", str(e))
            return
        
        if self.camera_intrinsics is None:
            rospy.logwarn("Camera intrinsics not available yet.")
            return

        self.last_map_time = msg.header.stamp
        msg.header.frame_id = "map"

        # Convert depth image to point cloud
        point_cloud = self.convert_depth_to_pointcloud(depth_image, msg.header)

        # Publish the point cloud
        self.publish_pointcloud(point_cloud)

    def camera_info_callback(self, msg):
        # Get camera intrinsic parameters
        fx = msg.K[0]
        fy = msg.K[4]
        cx = msg.K[2]
        cy = msg.K[5]
        self.camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(msg.width, msg.height, fx, fy, cx, cy)

    def convert_depth_to_pointcloud(self, depth_image, header):
        self.listener = tf.TransformListener()
        (trans,rot) = self.listener.lookupTransform("turtlebot/kobuki/base_footprint","camera_link", rospy.Time(0))
        # Create an Open3D depth image
        depth_o3d = o3d.geometry.Image(np.array(depth_image))

        # Convert depth image to point cloud
        point_cloud = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d, self.camera_intrinsics)
        
        # Convert rotation from quaternion to rotation matrix
        rotation_matrix = tf.transformations.quaternion_matrix(rot)

        # Set translation component of the transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix[:3, :3]  # Set rotation part
        transformation_matrix[:3, 3] = trans  # Set translation part
        
        # Apply the transformation to the point cloud
        point_cloud.transform(transformation_matrix)

        # temp = points[:, 0]
        # points[:, 2] = points[:, 1]
        # points[:, 0] = points[:, 2]
        # points[:, 1] = temp
        # point_cloud.points
        # Define the transformation matrix
        # transform_matrix = np.linalg.inv(np.array([[0, 1, 0, 0],
        #                                             [0, 0, 1, 0],
        #                                             [1, 0, 0, 0],
        #                                             [0, 0, 0, 1]]))
        

        # Apply transformation to the point cloud
        # transformed_point_cloud = point_cloud.transform(transform_matrix)

        points = np.asarray(point_cloud.points)

        # Extract y-coordinate
        y_coords = points[:, 1]

        # Keep points within y_min and y_max range
        mask = np.logical_and(y_coords >= self.y_min, y_coords <= self.y_max)

        points =  points[mask]

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
        
        pointcloud_msg = pc2.create_cloud(header, fields, points)

        return pointcloud_msg

    def publish_pointcloud(self, point_cloud):
        # Publish the point cloud
        self.pointcloud_pub.publish(point_cloud)
        pass


if __name__ == '__main__':

    try:
        depth_to_pointcloud_node = DepthToPointCloudNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass