#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi, cos, sin, atan2, asin, sqrt
import numpy
import rospy
from math import pi as PI

class VO_module2:

    def __init__(self):
        self.min_targetSpeed = rospy.get_param('min_targetSpeed')
        self.max_targetSpeed = rospy.get_param('max_targetSpeed')
        self.num_targetSpeedCandidates = int(rospy.get_param('num_targetSpeedCandidates'))

        self.min_targetHeading_deg_local = rospy.get_param('min_targetHeading_deg_local')
        self.max_targetHeading_deg_local = rospy.get_param('max_targetHeading_deg_local')
        self.num_targetHeadingCandidates = int(rospy.get_param('num_targetHeadingCandidates'))

        self.weight_alpha = rospy.get_param('weight_focusObs')  
        self.weight_aggresiveness = rospy.get_param('weight_agressivness')
        self.cri_param = rospy.get_param('cri_param')
        

    def distance(self, pose1, pose2):
        """ compute Euclidean distance for 2D """
        return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001

    def RVO_update(self, OS, TS, V_des):
        """ compute best velocity given the desired velocity, current velocity and workspace model"""
        TS_ID = TS.keys()
        for ts_ID in TS_ID:
            SD = [ts_ID]['mapped_radius']
        V_current = OS[]
        
        ROB_RAD = SD
        V_opt = list(V_current)
        for i in range(len(X)):
            vA = [V_current[i][0], V_current[i][1]]
            pA = [X[i][0], X[i][1]]
            RVO_BA_all = []
            for j in range(len(X)):
                if i!=j:
                    vB = [V_current[j][0], V_current[j][1]]
                    pB = [X[j][0], X[j][1]]
                    # use RVO
                    # transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
                    # use VO
                    transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
                    dist_BA = self.distance(pA, pB)
                    theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
                    if 2*ROB_RAD > dist_BA:
                        dist_BA = 2*ROB_RAD
                    theta_BAort = asin(2*ROB_RAD/dist_BA)
                    theta_ort_left = theta_BA+theta_BAort
                    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                    theta_ort_right = theta_BA-theta_BAort
                    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                    # use HRVO
                    # dist_dif = distance([0.5*(vB[0]-vA[0]),0.5*(vB[1]-vA[1])],[0,0])
                    # transl_vB_vA = [pA[0]+vB[0]+cos(theta_ort_left)*dist_dif, pA[1]+vB[1]+sin(theta_ort_left)*dist_dif]
                    RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD]
                    RVO_BA_all.append(RVO_BA)                
            vA_post = self.intersect(pA, V_des[i], RVO_BA_all)
            V_opt[i] = vA_post[:]
        return V_opt


    def intersect(self, pA, vA, RVO_BA_all):
        # print '----------------------------------------'
        # print 'Start intersection test'
        norm_v = self.distance(vA, [0, 0])
        suitable_V = []
        unsuitable_V = []
        for theta in numpy.arange(0, 2*PI, 0.1):
            for rad in numpy.arange(0.02, norm_v+0.02, norm_v/5.0):
                new_v = [rad*cos(theta), rad*sin(theta)]
                suit = True
                for RVO_BA in RVO_BA_all:
                    p_0 = RVO_BA[0]
                    left = RVO_BA[1]
                    right = RVO_BA[2]
                    dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        suit = False
                        break
                if suit:
                    suitable_V.append(new_v)
                else:
                    unsuitable_V.append(new_v)                
        new_v = vA[:]
        suit = True
        for RVO_BA in RVO_BA_all:                
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
            if self.in_between(theta_right, theta_dif, theta_left):
                suit = False
                break
        if suit:
            suitable_V.append(new_v)
        else:
            unsuitable_V.append(new_v)
        #----------------------        
        if suitable_V:
            # print 'Suitable found'
            vA_post = min(suitable_V, key = lambda v: self.distance(v, vA))
            new_v = vA_post[:]
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
        else:
            # print 'Suitable not found'
            tc_V = dict()
            for unsuit_v in unsuitable_V:
                tc_V[tuple(unsuit_v)] = 0
                tc = []
                for RVO_BA in RVO_BA_all:
                    p_0 = RVO_BA[0]
                    left = RVO_BA[1]
                    right = RVO_BA[2]
                    dist = RVO_BA[3]
                    rad = RVO_BA[4]
                    dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                        if abs(dist*sin(small_theta)) >= rad:
                            rad = abs(dist*sin(small_theta))
                        big_theta = asin(abs(dist*sin(small_theta))/rad)
                        dist_tg = abs(dist*cos(small_theta))-abs(rad*cos(big_theta))
                        if dist_tg < 0:
                            dist_tg = 0                    
                        tc_v = dist_tg/self.distance(dif, [0,0])
                        tc.append(tc_v)
                tc_V[tuple(unsuit_v)] = min(tc)+0.001
            WT = 0.2
            vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+self.distance(v, vA)))
        return vA_post 

    def in_between(self, theta_right, theta_dif, theta_left):
        if abs(theta_right - theta_left) <= PI:
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        else:
            if (theta_left <0) and (theta_right >0):
                theta_left += 2*PI
                if theta_dif < 0:
                    theta_dif += 2*PI
                if theta_right <= theta_dif <= theta_left:
                    return True
                else:
                    return False
            if (theta_left >0) and (theta_right <0):
                theta_right += 2*PI
                if theta_dif < 0:
                    theta_dif += 2*PI
                if theta_left <= theta_dif <= theta_right:
                    return True
                else:
                    return False