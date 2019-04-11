#!/usr/bin/env python
import rospy
import time
import nav_msgs, geometry_msgs
import tf

class Coordinate:
    def __init__(self):
        # Define paramrters for 
        self.sensor_reading_intervals = rospy.get_param("")
        self.optimize_pose_intervals = rospy.get_param("")

        self.currentMovement= [[0,0],[0,0]]
        self.currentPosition= [[0,0],[0,0]]

        # Range and bearing calculation service
        self.calc_range_bearing_call = rospy.ServiceProxy('calc_range_bearing', Func1)

        # optimize pose service
        self.optimize_pose = rospy.ServiceProxy('optimize_pose', Func2)

        # Update pose estimate for both rovers
        self.update_rover_pose = rospy.ServiceProxy('update_rover_pose', Func3)

        # Listening to odomtery of first rover
        rospy.Subscriber("/odom1", 
                                    odom, 
                                    self.updateOdometery,
                                    (0,calc_range_bearing_call, optimize_pose, update_rover_pose))
        
        # Listening to odomtery of second rover
        rospy.Subscriber("/odom2", 
                                    odom, 
                                    self.updateOdometery,
                                    (1,calc_range_bearing_call, optimize_pose, update_rover_pose))
        


    def updateOdometery(self, odom, args):
        rover_number = args[0]
        # Update self.currentPosition[rover_number] and self.currentMovement[rover_number]

# def Func1():


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