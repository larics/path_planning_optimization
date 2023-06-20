#!/usr/bin/env python

import rospy, math
import copy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Transform, Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import Bool

class PathOptimization:

    def __init__(self):
        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate)
        self.num_of_batt = rospy.get_param('~num_of_batt')
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
        self.wp_list = [[[1,1,5,0,0,0,1],[1,2,5,0,0,0,1],[1,3,5,0,0,0,1],[1,4,5,0,0,0,1]],[[2,1,5,0,0,0,1],[2,1,5,0,0,0,1]]]
    

        # rospy.Subscriber('mavros/global_position/local', Odometry, self.odometryCallback, queue_size=1)
        rospy.Subscriber('base_flag', Bool, self.baseFlagCallback, queue_size=1)

        # self.flight_path_pub = rospy.Publisher('flight_path', PoseStamped, queue_size=1)

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

    def odometryCallback(self, msg):
        self.current_uav_pose.position = msg.pose.pose.position
        self.current_uav_pose.orientation = msg.pose.pose.orientation

    def baseFlagCallback(self, msg):
        self.base_flag = msg.data
        self.active_batt += 1

    def pathOptimization(self):
        print("Optimization function")
        self.optimize_path_flag = False

    def distanceToWaypoint(self):
        self.distance_to_waypoint = math.sqrt((self.current_uav_pose.position.x - self.next_waypoint.pose.position.x)**2
        + (self.current_uav_pose.position.y - self.next_waypoint.pose.position.y)**2
        + (self.current_uav_pose.position.z - self.next_waypoint.pose.position.z)**2)
        
    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.optimize_path_flag == True:
                self.pathOptimization()
                self.first_batt_flag = True

            if ((self.first_batt_flag == True) or (self.base_flag == True)) and (self.active_batt < self.num_of_batt):
                self.flight_path = PoseArray()
                for i in range(len(self.wp_list[self.active_batt])):
                    self.flight_path_wp = PoseStamped()
                    self.flight_path_wp.pose.position.x = self.wp_list[self.active_batt][i][0]
                    # print(self.wp_list[self.active_batt][i][1])
                    self.flight_path_wp.pose.position.y = self.wp_list[self.active_batt][i][1]
                    self.flight_path_wp.pose.position.z = self.wp_list[self.active_batt][i][2]
                    self.flight_path_wp.pose.orientation.x = self.wp_list[self.active_batt][i][3]
                    self.flight_path_wp.pose.orientation.y = self.wp_list[self.active_batt][i][4]
                    self.flight_path_wp.pose.orientation.z = self.wp_list[self.active_batt][i][5]
                    self.flight_path_wp.pose.orientation.w = self.wp_list[self.active_batt][i][6]
                    self.flight_path.poses.append(self.flight_path_wp)
                self.flight_path_pub(self.flight_path)
                print(self.flight_path)
                self.first_batt_flag = False
                self.base_flag = False

if __name__ == '__main__':
    rospy.init_node('path_optimization')
    path_to_tracker = PathOptimization()
    path_to_tracker.run()