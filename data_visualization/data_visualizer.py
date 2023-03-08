#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import tf
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from udp_col_msg.msg import path_output, group_vis_info
from udp_msgs.msg import frm_info, group_wpts_info

class Visualizer:
    def __init__(self):
        self.sub_path_output = rospy.Subscriber('/path_out_inha', path_output, self.inha_callback, queue_size=1)
        self.sub_frm = rospy.Subscriber('/frm_info', frm_info, self.frm_callback, queue_size=1) 
        self.sub_waypoint = rospy.Subscriber('/waypoint_info', group_wpts_info, self.waypoint_callback, queue_size=1)   
        self.sub_vis_info = rospy.Subscriber('/group_vis_info', group_vis_info, self.group_vis_callback, queue_size=1) 

        self.OS_ID = rospy.get_param("OS_info/ship_ID")
        self.TS1_ID = rospy.get_param("TS1_info/ship_ID")
        self.TS2_ID = rospy.get_param("TS2_info/ship_ID")
        self.TS3_ID = rospy.get_param("TS3_info/ship_ID")
        self.TS4_ID = rospy.get_param("TS4_info/ship_ID")

        self.pub_ship0_path = rospy.Publisher('~ship_0_path', Path, queue_size=1)
        self.pub_ship1_path = rospy.Publisher('~ship_1_path', Path, queue_size=1)
        self.pub_ship2_path = rospy.Publisher('~ship_2_path', Path, queue_size=1)
        self.pub_ship3_path = rospy.Publisher('~ship_3_path', Path, queue_size=1)
        self.pub_ship4_path = rospy.Publisher('~ship_4_path', Path, queue_size=1)

        self.pub_stadium_marker = rospy.Publisher('~stadium_marker', Marker, queue_size=1)

        self.pub_heading_OS1_marker = rospy.Publisher('~heading_OS1_marker', Marker, queue_size=1)
        self.pub_wp_OS1_marker = rospy.Publisher('~wp_OS1_marker', Marker, queue_size=1)
        self.pub_vector_OS1_marker = rospy.Publisher('~vector_OS1_marker', Marker, queue_size=1)     
        self.pub_cone_OS1 = rospy.Publisher('~collision_cone_OS1', Marker, queue_size=1) 


        self.tf_broadcaster = tf.TransformBroadcaster()
        self.history_pos = {
            'ship_0': [],
            'ship_1': [],
            'ship_2': [],
            'ship_3': [],
            'ship_4': []
        }
        self.ship_ID = []
        self.len_path_out_inha = 0
        time.sleep(0.5)


    def check_subscribe(self):
        connect1 = self.sub_path_output.get_num_connections()
        connect2 = self.sub_frm.get_num_connections()
        connect3 = self.sub_waypoint.get_num_connections()
        connect4 = self.sub_vis_info.get_num_connections()
        # return은 connect가 3개 다 되면 True, 아니면 False
        return (connect1 + connect2 + connect3 + connect4) == 4


    def inha_callback(self, inha):
        """`/path_out_inha`의 데이터를 모두 저장하고 있음
        
        Note :
            inha_dic[f'{shipID}'] = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desired_spd, desired_heading, isNeedCA, "status_btw_OS"]
        Example:
            `desired_spd = self.path_out_inha_dic[f'{ship_ID}'].desired_spd`
        """
        if self.check_subscribe():
            self.len_path_out_inha = len(inha.pathData)
            inha_dic = dict()
            for i in range(self.len_path_out_inha):
                shipID = inha.pathData[i].nship_ID
                inha_dic['%i' %shipID] = inha.pathData[i]
        else:
            print('========= Waiting for all topic subscription(visualizer) =========')

        self.path_out_inha_dic = inha_dic


    def group_vis_callback(self, vis):
        """`/path_out_inha`의 데이터를 모두 저장하고 있음
        
        Note :
            inha_dic[f'{shipID}'] = [m_nShipID, isUpdateWP, numWP, WP_x[], WP_y[], speedWP, ETA_WP, EDA_WP, RI, CRI, isError, errors, desired_spd, desired_heading, isNeedCA, "status_btw_OS"]
        Example:
            `desired_spd = self.path_out_inha_dic[f'{ship_ID}'].desired_spd`
        """
        if self.check_subscribe():
            self.len_group_vis = len(vis.visData)
            vis_dic = dict()
            for i in range(self.len_group_vis):
                shipID = vis.visData[i].nship_ID
                vis_dic['%i' %shipID] = vis.visData[i]
        else:
            print('========= Waiting for all topic subscription(visualizer) =========')

        self.group_vis_dic = vis_dic



    def frm_callback(self, operation):
        ''' subscribe `/frm_info` 
        
        params : 
            `frm_info` 변수명은 입출력관계도 KRISO 참조

        Note :
            `psi`값이 [-2pi, 2pi]값으로 들어오므로, 편의상 강제로 [0, 2pi]로 변경
        '''
        if self.check_subscribe():
            self.ship_ID = list(operation.m_nShipID)
            self.Pos_X  = operation.m_fltPos_X
            self.Pos_Y  = np.asanyarray(operation.m_fltPos_Y)
            # Why flip the y pos???????????? Anyway...
            self.Vel_U  = operation.m_fltVel_U
            # self.Heading = operation.m_fltFOGang_yawzG
            raw_psi = np.asanyarray(operation.m_fltHeading)
            self.Heading = np.deg2rad(raw_psi % 360)
            print('heading', self.Heading)
            for i in range(len(self.ship_ID)):
                quat = tf.transformations.quaternion_from_euler(0, 0, self.Heading[i])
                self.tf_broadcaster.sendTransform((self.Pos_X[i], self.Pos_Y[i], 0),
                                                quat,
                                                rospy.Time.now(),
                                                'ship_%i' %i,
                                                'world')
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'ship_%i' %i
                pose_stamped.pose.position.x = self.Pos_X[i]
                pose_stamped.pose.position.y = self.Pos_Y[i]
                pose_stamped.pose.orientation.x = quat[0]
                pose_stamped.pose.orientation.y = quat[1]
                pose_stamped.pose.orientation.z = quat[2]
                pose_stamped.pose.orientation.w = quat[3]

                self.history_pos['ship_%i' %i].append(pose_stamped)       
        else:
            print('========= Waiting for all topic subscription(visualizer) =========')

                
    def waypoint_callback(self, wp):
        if self.check_subscribe():
            self.len_waypoint_info = len(wp.group_wpts_info)
            wp_dic = dict()
            for i in range(self.len_waypoint_info):
                shipID = wp.group_wpts_info[i].shipID
                wp_dic['{}'.format(shipID)] = wp.group_wpts_info[i]
            self.waypoint_dict = wp_dic
        else:
            print('========= Waiting for all topic subscription(visualizer) =========')


    def pub_trajectory(self, path_params_dict):
        """
            Path Visualization Example
        """
        path0 = Path()
        path0.header.stamp = rospy.Time.now() 
        path0.header.frame_id = 'world'        
        path0.poses = path_params_dict['ship_0']

        path1 = Path()
        path1.header.stamp = rospy.Time.now() 
        path1.header.frame_id = 'world'        
        path1.poses = path_params_dict['ship_1']

        path2 = Path()
        path2.header.stamp = rospy.Time.now() 
        path2.header.frame_id = 'world'        
        path2.poses = path_params_dict['ship_2']

        path3 = Path()
        path3.header.stamp = rospy.Time.now() 
        path3.header.frame_id = 'world'        
        path3.poses = path_params_dict['ship_3']

        path4 = Path()
        path4.header.stamp = rospy.Time.now() 
        path4.header.frame_id = 'world'        
        path4.poses = path_params_dict['ship_4']

        self.pub_ship0_path.publish(path0) 
        self.pub_ship1_path.publish(path1) 
        self.pub_ship2_path.publish(path2) 
        self.pub_ship3_path.publish(path3) 
        self.pub_ship4_path.publish(path4) 


    def pub_heading_arrow_OS1(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now() 
        marker.header.frame_id = "world"
        marker.type = marker.ARROW
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.4
        marker.scale.z = 0.3

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientation
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
        first_line_point.x = self.Pos_X[0]
        first_line_point.y = self.Pos_Y[0]
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        
        # second point
        second_line_point = Point()
        second_line_point.x = self.group_vis_dic['%s' %self.OS_ID].local_goal[0]
        second_line_point.y = self.group_vis_dic['%s' %self.OS_ID].local_goal[1] # FIXME waypoint 바뀌게 넣어줘야함
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        self.pub_heading_OS1_marker.publish(marker)


        wp = Marker()
        wp.header.stamp = rospy.Time.now() 
        wp.header.frame_id = "world"
        wp.type = marker.POINTS
        wp.action = marker.ADD

        # marker scale
        wp.scale.x = 10   # diameter 
        wp.scale.y = 10   # diameter
        wp.scale.z = 0.5 # height

        # marker color
        wp.color.a = 0.2
        wp.color.r = 1.0
        wp.color.g = 1.0
        wp.color.b = 0.5

        # marker orientation
        wp.pose.orientation.x = 0.0
        wp.pose.orientation.y = 0.0
        wp.pose.orientation.z = 0.0
        wp.pose.orientation.w = 1.0

        for i in range(len(self.waypoint_dict['%s' %self.OS_ID].wpts_x)):
            wp.points.append(Point(self.waypoint_dict['%s' %self.OS_ID].wpts_x[i], self.waypoint_dict['%s' %self.OS_ID].wpts_y[i], 0.0))

        self.pub_wp_OS1_marker.publish(wp)



    def pub_heading_arrow_OS2(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now() 
        marker.header.frame_id = "world"
        marker.type = marker.ARROW
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.4
        marker.scale.z = 0.3

        # marker color
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientation
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
        first_line_point.x = self.Pos_X[1]
        first_line_point.y = self.Pos_Y[1]
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        
        # second point
        second_line_point = Point()
        second_line_point.x = self.group_vis_dic['%s' %self.TS1_ID].local_goal[0]
        second_line_point.y = self.group_vis_dic['%s' %self.TS1_ID].local_goal[1] # FIXME waypoint 바뀌게 넣어줘야함
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        self.pub_heading_OS2_marker.publish(marker)


        wp = Marker()
        wp.header.stamp = rospy.Time.now() 
        wp.header.frame_id = "world"
        wp.type = marker.POINTS
        wp.action = marker.ADD

        # marker scale
        wp.scale.x = 10   # diameter 
        wp.scale.y = 10   # diameter
        wp.scale.z = 0.5 # height

        # marker color
        wp.color.a = 0.2
        wp.color.r = 1.0
        wp.color.g = 0.0
        wp.color.b = 1.0

        # marker orientation
        wp.pose.orientation.x = 0.0
        wp.pose.orientation.y = 0.0
        wp.pose.orientation.z = 0.0
        wp.pose.orientation.w = 1.0

        for i in range(len(self.waypoint_dict['%s' %self.TS1_ID].wpts_x)):
            wp.points.append(Point(self.waypoint_dict['%s' %self.TS1_ID].wpts_x[i], self.waypoint_dict['%s' %self.TS1_ID].wpts_y[i], 0.0))

        self.pub_wp_OS2_marker.publish(wp)


    def pub_vector_OS1(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now() 
        marker.header.frame_id = "world"
        marker.type = marker.ARROW
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.3
        marker.scale.z = 0.3 # 0.3

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.3
        marker.scale.z = 0.3 # 0.3

        # marker color
        marker.color.a = 0.9    # hyeri: a는 alpha로 색의 불투명도를 나타냄. 0~1 사이의 수 넣으면 되고 작을수록 투명
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # marker orientation
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
        first_line_point.x = self.Pos_X[0]
        first_line_point.y = self.Pos_Y[0]
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        
        # second point # 여기를 v_opt로 바꿔주면 되지않을까?
        second_line_point = Point()
        second_line_point.x = self.group_vis_dic['%s' %self.OS_ID].v_opt[0] + self.Pos_X[0]
        second_line_point.y = self.group_vis_dic['%s' %self.OS_ID].v_opt[1] + self.Pos_Y[0]
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        self.pub_vector_OS1_marker.publish(marker)


    def pub_vector_OS2(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now() 
        marker.header.frame_id = "world"
        marker.type = marker.ARROW
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.3
        marker.scale.z = 0.3 # 0.3

        # marker color
        marker.color.a = 0.9    # hyeri: a는 alpha로 색의 불투명도를 나타냄. 0~1 사이의 수 넣으면 되고 작을수록 투명
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # marker orientation
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
        first_line_point.x = self.Pos_X[1]
        first_line_point.y = self.Pos_Y[1]
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        
        # second point # 여기를 v_opt로 바꿔주면 되지않을까?
        second_line_point = Point()
        second_line_point.x = self.group_vis_dic['%s' %self.TS1_ID].v_opt[0] + self.Pos_X[1]
        second_line_point.y = self.group_vis_dic['%s' %self.TS1_ID].v_opt[1] + self.Pos_Y[1]
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        self.pub_vector_OS2_marker.publish(marker)


    def pub_collision_cone_OS1(self):
        cone = Marker()
        cone.header.stamp = rospy.Time.now() 
        cone.header.frame_id = "world"
        cone.type = cone.TRIANGLE_LIST
        cone.action = cone.ADD

        # marker scale
        cone.scale.x = 1.0
        cone.scale.y = 1.0
        cone.scale.z = 1.0

        # marker color
        cone.color.a = 0.3
        cone.color.r = 1.0
        cone.color.g = 1.0
        cone.color.b = 0.0
        # cone.color = ColorRGBA(1, 1, 0, 0.3)

        # marker orientation
        cone.pose.orientation.x = 0.0
        cone.pose.orientation.y = 0.0
        cone.pose.orientation.z = 0.0
        cone.pose.orientation.w = 1.0

        collsion_cone_point = self.group_vis_dic['%s' %self.OS_ID].collision_cone
        if len(collsion_cone_point) == 0:
            cone.points.append(Point(self.Pos_X[0], self.Pos_Y[0], 0))
            cone.points.append(Point(self.Pos_X[0], self.Pos_Y[0], 0))
            cone.points.append(Point(self.Pos_X[0], self.Pos_Y[0], 0))

        elif len(collsion_cone_point) == 6:
            cone.points.append(Point(collsion_cone_point[0], collsion_cone_point[1], 0))
            cone.points.append(Point(collsion_cone_point[2], collsion_cone_point[3], 0))
            cone.points.append(Point(collsion_cone_point[4], collsion_cone_point[5], 0))

        elif len(collsion_cone_point) == 12:
            cone.points.append(Point(collsion_cone_point[0], collsion_cone_point[1], 0))
            cone.points.append(Point(collsion_cone_point[2], collsion_cone_point[3], 0))
            cone.points.append(Point(collsion_cone_point[4], collsion_cone_point[5], 0))

            cone.points.append(Point(collsion_cone_point[6], collsion_cone_point[7], 0))
            cone.points.append(Point(collsion_cone_point[8], collsion_cone_point[9], 0))
            cone.points.append(Point(collsion_cone_point[10], collsion_cone_point[11], 0))

        self.pub_cone_OS1.publish(cone)


    def pub_collision_cone_OS2(self):
        cone = Marker()
        cone.header.stamp = rospy.Time.now() 
        cone.header.frame_id = "world"
        cone.type = cone.TRIANGLE_LIST
        cone.action = cone.ADD

        # marker scale
        cone.scale.x = 1.0
        cone.scale.y = 1.0
        cone.scale.z = 1.0

        # marker color
        cone.color.a = 0.2
        cone.color.r = 1.0
        cone.color.g = 0.0
        cone.color.b = 1.0
        # cone.color = ColorRGBA(1, 1, 0, 0.3)

        # marker orientation
        cone.pose.orientation.x = 0.0
        cone.pose.orientation.y = 0.0
        cone.pose.orientation.z = 0.0
        cone.pose.orientation.w = 1.0

        collsion_cone_point = self.group_vis_dic['%s' %self.TS1_ID].collision_cone
        if len(collsion_cone_point) == 0:
            cone.points.append(Point(self.Pos_X[1], self.Pos_Y[1], 0))
            cone.points.append(Point(self.Pos_X[1], self.Pos_Y[1], 0))
            cone.points.append(Point(self.Pos_X[1], self.Pos_Y[1], 0))

        elif len(collsion_cone_point) == 6:
            cone.points.append(Point(collsion_cone_point[0], collsion_cone_point[1], 0))
            cone.points.append(Point(collsion_cone_point[2], collsion_cone_point[3], 0))
            cone.points.append(Point(collsion_cone_point[4], collsion_cone_point[5], 0))

        elif len(collsion_cone_point) == 12:
            cone.points.append(Point(collsion_cone_point[0], collsion_cone_point[1], 0))
            cone.points.append(Point(collsion_cone_point[2], collsion_cone_point[3], 0))
            cone.points.append(Point(collsion_cone_point[4], collsion_cone_point[5], 0))

            cone.points.append(Point(collsion_cone_point[6], collsion_cone_point[7], 0))
            cone.points.append(Point(collsion_cone_point[8], collsion_cone_point[9], 0))
            cone.points.append(Point(collsion_cone_point[10], collsion_cone_point[11], 0))

        self.pub_cone_OS2.publish(cone)


    def pub_stadium(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now() 
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

        # marker orientation
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
        first_line_point.x = -20.0
        first_line_point.y = -20.0
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        
        # second point
        second_line_point = Point()
        second_line_point.x = -20.0
        second_line_point.y = 20.0
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        # third point
        third_line_point = Point()
        third_line_point.x = 20.0
        third_line_point.y = 20.0
        third_line_point.z = 0.0
        marker.points.append(third_line_point)

        # fourth point
        fourth_line_point = Point()
        fourth_line_point.x = 20.0
        fourth_line_point.y = -20.0
        fourth_line_point.z = 00.0
        
        marker.points.append(fourth_line_point)
        
        marker.points.append(first_line_point)

        # Publish the Marker
        self.pub_stadium_marker.publish(marker)

def main():
    update_rate = rospy.get_param("update_rate")
    # update_rate = 10

    rospy.init_node('data_visualizer', anonymous=False)    
    rate = rospy.Rate(update_rate) # 10 Hz renew

    vis = Visualizer()
    
    while not rospy.is_shutdown():
        if len(vis.ship_ID) == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
            print("========= Waiting for topic subscription =========")
            rate.sleep()
            continue

        if  vis.len_path_out_inha == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
            print("========= Waiting for `/path_out_inha` topic subscription =========")
            rate.sleep()
            continue

        if  vis.len_waypoint_info == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
            print("========= Waiting for `/waypoint_info` topic subscription =========")
            rate.sleep()
            continue

        if  vis.len_group_vis == 0:
            ## 아직 초기값이 들어오지 않은 상태라면 return 시켜 버림 
            print("========= Waiting for `/group_vis_info` topic subscription =========")
            rate.sleep()
            continue


        vis.pub_stadium()
        vis.pub_trajectory(vis.history_pos)

        vis.pub_heading_arrow_OS1()
        vis.pub_vector_OS1()
        vis.pub_collision_cone_OS1()

        vis.pub_heading_arrow_OS2()
        vis.pub_vector_OS2()
        vis.pub_collision_cone_OS2()

        vis.pub_heading_arrow_OS3()
        vis.pub_vector_OS3()
        vis.pub_collision_cone_OS3()
        vis.pub_heading_arrow_TS1()
        vis.pub_next_waypoints_stamped_TS1()
        vis.pub_heading_arrow_TS2()
        vis.pub_next_waypoints_stamped_TS2()
        vis.pub_heading_arrow_TS3()
        vis.pub_next_waypoints_stamped_TS3()
        vis.pub_heading_arrow_TS4()
        vis.pub_next_waypoints_stamped_TS4()
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()