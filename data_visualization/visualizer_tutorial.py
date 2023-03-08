#!/usr/bin/env python

import rospy
import time
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class Visualizer:
    def __init__(self):
        self.pub_test_path = rospy.Publisher('ship_a_path_marker', Path, queue_size=1)
        self.pub_test_marker = rospy.Publisher('test_marker', Marker, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        time.sleep(0.5)
        
    def publish_path(self):
        """
            Path Visualization Example
        """
        path = Path()
        path.header.frame_id = 'observer'
        float_x = 0.0
        for _ in range(10):
            
            pose = PoseStamped()
            
            pose.header.frame_id = 'observer'
            pose.pose.position.x = float_x
            
            path.poses.append(pose)
            float_x += 0.5
            
        self.pub_test_path.publish(path)
        self
    def publish_tf(self):
        """
            TF Visualization Example
        """
        
        for _ in range(5):
            print(1)
            br = tf.TransformBroadcaster()
            time.sleep(0.5)
            br.sendTransform( (3.0, 3.0, 0) ,
                            tf.transformations.quaternion_from_euler(0, 0, 0.0),
                            rospy.Time.now(),
                            'observer',
                            'ship_b'
                            )
        rospy.loginfo("publish tf")
    
    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "observer"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []
        # first point
        first_line_point = Point()
        first_line_point.x = -2.0
        first_line_point.y = -2.0
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        
        # second point
        second_line_point = Point()
        second_line_point.x = -2.0
        second_line_point.y = 2.0
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        # third point
        third_line_point = Point()
        third_line_point.x = 2.0
        third_line_point.y = 2.0
        third_line_point.z = 0.0
        marker.points.append(third_line_point)

        # fourth point
        fourth_line_point = Point()
        fourth_line_point.x = 2.0
        fourth_line_point.y = -2.0
        fourth_line_point.z = 0.0
        
        marker.points.append(fourth_line_point)
        
        marker.points.append(first_line_point)

        # Publish the Marker
        self.pub_test_marker.publish(marker)
        
        
    
if __name__ == '__main__':
    rospy.init_node('test_path_publisher_node')

    test = Visualizer()
    
    test.publish_path()
    
    test.publish_tf()
    
    test.publish_marker()
    
    rospy.spin()    
    

