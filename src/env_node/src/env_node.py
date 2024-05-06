#!/usr/bin/python
import tf
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


odom_freq   = 0.1
odom_window = 100000.0


class Env:
    def __init__(self, odom_topic) -> None:
        # PUBLISHERS      
     
        # SUBSCRIBERS
        
        # SERVICES
    

        # TIMERS
        # Timer for pub tf
        rospy.Timer(rospy.Duration(0.05), self.pub_tf)

    
    # Ground Truth callback: Gets current robot pose and stores it into self.current_pose. Besides, get heading as a measurement to update filter
    def get_ground_truth(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

        # Get heading as a measurement to update filter
        if self.mag.read_magnetometer(yaw-self.yawOffset) and self.ekf_filter is not None:
            self.ekf_filter.gotNewHeadingData()

    # Odometry callback: Gets encoder reading to compute displacement of the robot as input of the EKF Filter.
    # Run EKF Filter with frequency of odometry reading
    def get_odom(self, odom):
        # Read encoder
        if self.odom.read_encoder(odom) and self.ekf_filter is not None:
            self.ekf_filter.gotNewEncoderData()

        if self.ekf_filter is not None:
            # Run EKF Filter
            self.xk, self.Pk = self.ekf_filter.Localize(self.xk, self.Pk)

            self.x_map       = Pose3D.oplus(self.x_frame_k, self.xk)

            # Publish rviz
            self.sensor_pub()

            self.publish_tf_map()

    # Publish Filter results
    def sensor_pub(self):
        # Transform theta from euler to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, float((self.xk[2, 0])))  # Convert euler angles to quaternion

        # Publish predicted odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        # odom.child_frame_id = "turtlebot/kobuki/predicted_base_footprint"


        odom.pose.pose.position.x = self.xk[0]
        odom.pose.pose.position.y = self.xk[1]

        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.pose.covariance = list(np.array([[self.Pk[0, 0], self.Pk[0, 1], 0, 0, 0, self.Pk[0, 2]],
                                [self.Pk[1, 0], self.Pk[1,1], 0, 0, 0, self.Pk[1, 2]],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [self.Pk[2, 0], self.Pk[2, 1], 0, 0, 0, self.Pk[2, 2]]]).flatten())

        # odom.twist.twist.linear.x = self.v
        # odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

        # Transform theta from euler to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # Convert euler angles to quaternion
        translation = (0.0, 0.0, 0.0)
        frame_id = "turtlebot/kobuki/realsense_depth"
        child_frame_id = "sensor"

        tf.TransformBroadcaster().sendTransform(translation, quaternion, rospy.Time.now(), child_frame_id, frame_id)

    def publish_tf_map(self):
        x_map = self.x_map.copy()

        # Define the translation and rotation for the inverse TF (base_footprint to world)
        translation = (x_map[0], x_map[1], 0) # Set the x, y, z coordinates

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # Convert euler angles to quaternion
        rotation = (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        
        # Publish the inverse TF from world to base_footprint
        tf.TransformBroadcaster().sendTransform(
            translation,
            rotation,
            rospy.Time.now(),
            "turtlebot/kobuki/base_footprint",
            "map"
        )

    def spin(self):
        pass

if __name__ == '__main__':
    rospy.init_node('EKF_node')
    node = EKF('/turtlebot/joint_states')	
    
    rate = rospy.Rate(odom_freq)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()