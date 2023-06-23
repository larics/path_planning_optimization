#!/usr/bin/env python3

import rospy, math
import copy
import time
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Transform, Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import Bool
from mip import Model, BINARY, minimize, xsum
from math import sqrt
import numpy as np

class PathOptimization:

    def __init__(self):
        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate)
        self.num_of_batt = rospy.get_param('~num_of_batt')
        self.batt_time = rospy.get_param('~batt_time')
        self.l_x = rospy.get_param('~length_x')
        self.l_y = rospy.get_param('~length_y')
        self.r = rospy.get_param('~spacing')
        self.base_x = rospy.get_param('~base_x')
        self.base_y = rospy.get_param('~base_y')
        self.base_z = rospy.get_param('~base_z')
        self.altitude = rospy.get_param('~altitude')
        self.vel_xy = 1.0
        self.vel_z = 1.0
        self.vel_yaw = 1.0
        self.acc_xy = 0.5
        self.acc_z = 0.5
        self.acc_yaw = 0.5

        self.optimize_path_flag = True

        self.base_flag = False
        self.first_batt_flag = False

        self.active_batt = 0
        self.b1 = [[1,1,5,0,0,0,1],[1,2,5,0,0,0,1],[1,3,5,0,0,0,1],[1,4,5,0,0,0,1]]
        self.b2 = [[2,1,5,0,0,0,1],[2,1,5,0,0,0,1]]
        self.b = []
        self.b.append(self.b1)
        self.b.append(self.b2)
        print(self.b)
        # self.wp_list = [[[1,1,5,0,0,0,1],[1,2,5,0,0,0,1],[1,3,5,0,0,0,1],[1,4,5,0,0,0,1]],[[2,1,5,0,0,0,1],[2,2,5,0,0,0,1]]]
        self.wp_list = []
        self.base = [self.base_x,self.base_y,self.base_z,0,0,0,1]

        # rospy.Subscriber('mavros/global_position/local', Odometry, self.odometryCallback, queue_size=1)
        rospy.Subscriber('base_flag', Bool, self.baseFlagCallback, queue_size=1)

        self.flight_path_pub = rospy.Publisher('flight_path', PoseArray, queue_size=1)
        # self.mpc_tracker_pose_pub = rospy.Publisher('tracker/input_pose', PoseStamped, queue_size=1)

        self.current_uav_pose = Pose()
        self.path_recieved_flag = False
        self.waypoint_reached_flag = True
        self.calculate_distance_to_wp = False
        # self.flight_path = PoseArray()
        self.next_waypoint = PoseStamped()
        # self.flight_path_wp = PoseStamped()
        self.next_waypoint_num = 0
        self.number_of_waypoints = 0
        self.distance_to_waypoint = 0

        self.multi_dof_trajectory = MultiDOFJointTrajectory()

        print("PathOptimization initialized")
        print("Num of batteries: %d" % self.num_of_batt)

        time.sleep(0.5)

    def odometryCallback(self, msg):
        self.current_uav_pose.position = msg.pose.pose.position
        self.current_uav_pose.orientation = msg.pose.pose.orientation

    def baseFlagCallback(self, msg):
        self.base_flag = msg.data
        self.active_batt += 1

    def pathOptimization(self):
        print("Optimization function")

        # rectangular shape of the field with the equal spacing r

        n_x = int(self.l_x/self.r +1)
        n_y = int(self.l_y/self.r +1)

        n_wp = n_x * n_y

        z = self.altitude       # constant height [m]
        wp = [[0,0,z,0,0,0,1] for _ in range(n_wp)]

        for x in range(self.l_x+1):
            for y in range(self.l_y+1):
                wp[x*(n_y)+y][1] = y+1

        for x in range(self.l_x+1):
            for y in range(self.l_y+1):
                wp[y*(n_x)+x][0] = y+1   


        for n in range(self.l_y+1):
            if n %2:
                wp_tmp = wp[n*self.l_x+n:n*self.l_x+n+self.l_x+1]   
                wp_tmp.reverse()
                wp[n*self.l_x+n:n*self.l_x+n+self.l_x+1]=wp_tmp 

        dist_wp = []

        def distance(x,y):
            return sqrt((x[0]-y[0]) ** 2 + (x[1]-y[1]) ** 2 + (x[2]-y[2]) ** 2)

        for n in range(len(wp)-1):
            dist_wp.append(distance(wp[n],wp[n+1]))


        N = len(wp)                  # number of POIs  
        B = self.num_of_batt                    # no. of batteries
        K = set(range(N-1))           # number of segments = N-1
        J = set(range(B))            # no. of batteries
        # print(N, B, K, J)

        s_k = dist_wp*len(J)
        s_k_tmp = np.array(s_k)
        # print(s_k_tmp)
        s_k_tmp = np.reshape(s_k_tmp,(len(J),len(K)))
        s_k = s_k_tmp.tolist()

        v_c = 5        # constant velocity [m/s]

        m = Model()

        x = [[m.add_var(var_type=BINARY) for k in K] for j in J]

        t_fjk = []
        t_fj = []
        t_j = self.batt_time
        # print(t_j)

        for j in J:
            for k in K:
                t_fjk.append(s_k[j][k]/v_c * x[j][k])    

        tmp = np.array(t_fjk)    
        tmp_m = np.reshape(tmp,(len(J),len(K)))
        t_fjk = tmp_m.tolist()
        # t_j = [0.1, 0.2]

        for j in J:
            t_fj.append(xsum(t_fjk[j][k] for k in K) - t_j[j])
            


        m.objective = minimize(xsum(x[j][k] for k in K for j in J))  

        for j in J:
            m += xsum(t_fjk[j][k] for k in K) <= t_j[j] * v_c

        for j in J:
            for k in K:
                m += xsum(x[j][k] for j in J) == 1

        m.optimize()

        b_list = []
        b_list = [[] for _ in J]

        for j in J:
            for k in K:
                if x[j][k].x == 1:
                    b_list[j].append(wp[k])
            b_list[j].append(self.base)       

        self.wp_list = b_list
        print(self.wp_list[0])
        print("next battery")
        print(self.wp_list[1])
        self.optimize_path_flag = False

    # def distanceToWaypoint(self):
    #     self.distance_to_waypoint = math.sqrt((self.current_uav_pose.position.x - self.next_waypoint.pose.position.x)**2
    #     + (self.current_uav_pose.position.y - self.next_waypoint.pose.position.y)**2
    #     + (self.current_uav_pose.position.z - self.next_waypoint.pose.position.z)**2)
        
    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.optimize_path_flag == True:
                self.pathOptimization()
                self.first_batt_flag = True

            if ((self.first_batt_flag == True) or (self.base_flag == True)) and (self.active_batt < self.num_of_batt):
                self.flight_path = PoseArray()
                print("Active battery: %d " % self.active_batt)
                for i in range(len(self.wp_list[self.active_batt])):
                    self.flight_path_wp = Pose()
                    self.flight_path_wp.position.x = self.wp_list[self.active_batt][i][0]
                    self.flight_path_wp.position.y = self.wp_list[self.active_batt][i][1]
                    self.flight_path_wp.position.z = self.wp_list[self.active_batt][i][2]
                    self.flight_path_wp.orientation.x = self.wp_list[self.active_batt][i][3]
                    self.flight_path_wp.orientation.y = self.wp_list[self.active_batt][i][4]
                    self.flight_path_wp.orientation.z = self.wp_list[self.active_batt][i][5]
                    self.flight_path_wp.orientation.w = self.wp_list[self.active_batt][i][6]
                    self.flight_path.poses.append(self.flight_path_wp)
                self.flight_path_pub.publish(self.flight_path)
                # print(self.flight_path)
                self.first_batt_flag = False
                self.base_flag = False

if __name__ == '__main__':
    rospy.init_node('path_optimization')
    path_to_tracker = PathOptimization()
    path_to_tracker.run()