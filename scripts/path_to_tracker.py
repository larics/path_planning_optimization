#!/usr/bin/env python

import rospy, math
import copy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Transform, Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

class PathToTracker:

    def __init__(self):
        self.rate = rospy.get_param('~rate', 100)
        self.wp_radius = rospy.get_param('~wp_radius', 0.2)
        self.tracker_input_type = rospy.get_param('~tracker_input_type', 'pose')
        self.ros_rate = rospy.Rate(self.rate)
        self.vel_xy = 1.0
        self.vel_z = 1.0
        self.vel_yaw = 1.0
        self.acc_xy = 0.5
        self.acc_z = 0.5
        self.acc_yaw = 0.5

        rospy.Subscriber('mavros/global_position/local', Odometry, self.odometryCallback, queue_size=1)
        rospy.Subscriber('flight_path', PoseArray, self.flightPathCallback, queue_size=1)

        self.mpc_tracker_pose_pub = rospy.Publisher('tracker/input_pose', PoseStamped, queue_size=1)
        self.mpc_tracker_traj_pub = rospy.Publisher('tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1)

        self.current_uav_pose = Pose()
        self.path_recieved_flag = False
        self.waypoint_reached_flag = True
        self.calculate_distance_to_wp = False
        self.flight_path = PoseArray()
        self.next_waypoint = PoseStamped()
        self.next_waypoint_num = 0
        self.number_of_waypoints = 0
        self.distance_to_waypoint = 0

        self.multi_dof_trajectory = MultiDOFJointTrajectory()

        print("PathToTracker initialized")
        print("wp_radius: %.2f" % self.wp_radius)
        print("tracker_input_type: ", self.tracker_input_type)

    def odometryCallback(self, msg):
        self.current_uav_pose.position = msg.pose.pose.position
        self.current_uav_pose.orientation = msg.pose.pose.orientation

    def flightPathCallback(self, msg):
        if self.path_recieved_flag == False:
            self.flight_path = copy.deepcopy(msg)
            self.number_of_waypoints = len(self.flight_path.poses)
            self.path_recieved_flag = True
            print("Flight path received")

        self.multi_dof_trajectory = MultiDOFJointTrajectory()
        for i in range(0, self.number_of_waypoints):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_vel = Twist()
            temp_acc = Twist()
            temp_transform.translation.x = self.flight_path.poses[i].position.x
            temp_transform.translation.y = self.flight_path.poses[i].position.y
            temp_transform.translation.z = self.flight_path.poses[i].position.z
            temp_transform.rotation.x = self.flight_path.poses[i].orientation.x
            temp_transform.rotation.y = self.flight_path.poses[i].orientation.y
            temp_transform.rotation.z = self.flight_path.poses[i].orientation.z
            temp_transform.rotation.w = self.flight_path.poses[i].orientation.w
            temp_vel.linear.x = self.vel_xy
            temp_vel.linear.y = self.vel_xy
            temp_vel.linear.z = self.vel_z
            temp_acc.linear.x = self.acc_xy
            temp_acc.linear.y = self.acc_xy
            temp_acc.linear.z = self.acc_z

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)

            self.multi_dof_trajectory.points.append(temp_point)

    def distanceToWaypoint(self):
        self.distance_to_waypoint = math.sqrt((self.current_uav_pose.position.x - self.next_waypoint.pose.position.x)**2
        + (self.current_uav_pose.position.y - self.next_waypoint.pose.position.y)**2
        + (self.current_uav_pose.position.z - self.next_waypoint.pose.position.z)**2)
        
    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.tracker_input_type == "pose":
                if (self.path_recieved_flag == True) and (self.waypoint_reached_flag == True):
                    if self.next_waypoint_num < self.number_of_waypoints:
                        self.next_waypoint.pose = self.flight_path.poses[self.next_waypoint_num]
                        self.mpc_tracker_pose_pub.publish(self.next_waypoint)
                        print("Go to wp %d" % self.next_waypoint_num)
                        self.next_waypoint_num += 1
                        self.waypoint_reached_flag = False
                        self.calculate_distance_to_wp = True
                    else:
                        self.next_waypoint_num = 0
                        self.path_recieved_flag = False
                if self.calculate_distance_to_wp == True:
                    self.distanceToWaypoint()
                # print("Distance to waypoint %.2f" % self.distance_to_waypoint)
                if self.distance_to_waypoint < self.wp_radius:
                    self.waypoint_reached_flag = True
            elif self.tracker_input_type == "trajectory":
                if self.path_recieved_flag == True:
                    print("MPC trajectory started")
                    self.mpc_tracker_traj_pub.publish(self.multi_dof_trajectory)
                    self.path_recieved_flag = False
            else:
                rospy.logerr("Unknown tracker input type")
if __name__ == '__main__':

    rospy.init_node('path_to_tracker')
    path_to_tracker = PathToTracker()
    path_to_tracker.run()