#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math
from colocalization.srv import addBearingRangeNodes, optimizeFactorGraph

class Coordinate:
    def __init__(self):
        # Define paramrters for
        self.sensor_reading_intervals = 0
        self.optimize_pose_intervals = 0

        self.previous_pose_sensor_readings= [[0,0],[0,0]]
        self.previous_pose_optimize_graph = [[0,0],[0,0]]
        # Range and bearing calculation service
        self.addBearingRangeNodes = rospy.ServiceProxy('addBearingRangeNodes', addBearingRangeNodes)

        # optimize pose service
        self.optimizeFactorGraph = rospy.ServiceProxy('optimizeFactorGraph', optimizeFactorGraph)

        # Listening to odomtery of first rover
        rospy.Subscriber("/odom1",
                                    Odometry,
                                    self.updateOdometry,
                                    (0,))

        # Listening to odomtery of second rover
        rospy.Subscriber("/odom2",
                                    Odometry,
                                    self.updateOdometry,
                                    (1,))


    def get_distance(self, odom, previouspose, rover_n):
        current_pose_x = odom.pose.pose.position.x
        current_pose_y = odom.pose.pose.position.y
        previous_x = previouspose[rover_n][0]
        previous_y = previouspose[rover_n][1]
        distance = math.sqrt((current_pose_x - previous_x)**2 + (current_pose_y - previous_y)**2)
        return distance

    def updateOdometry(self, odom, args):
        current_pose_x = odom.pose.pose.position.x
        current_pose_y = odom.pose.pose.position.y
        rover_n = args[0]

        distance = self.get_distance(odom, self.previous_pose_sensor_readings, rover_n)
        print(distance)
        if distance > self.sensor_reading_intervals:
            print("I am in addBearingRangeNodes")
            self.addBearingRangeNodes()
            self.previous_pose_sensor_readings[rover_n][0] = current_pose_x
            self.previous_pose_sensor_readings[rover_n][1] = current_pose_y

        distance = self.get_distance(odom, self.previous_pose_optimize_graph, rover_n)
        print(distance)
        if distance > self.optimize_pose_intervals:
            print("I am in optimizefactor")
            self.optimizeFactorGraph()
            self.previous_pose_optimize_graph[rover_n][0] = current_pose_x
            self.previous_pose_optimize_graph[rover_n][1] = current_pose_y


if __name__ == "__main__":
    try:
        # Init ROS node
        rospy.init_node('coordinator')

        # Instatiated the class
        NC = Coordinate()

        # Wait
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
