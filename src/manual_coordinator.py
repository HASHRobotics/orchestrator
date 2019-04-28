#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
# from colocalization.srv import addBearingRangeNodes, optimizeFactorGraph
from bearing_estimator.srv import estimate_bearing,estimate_range,ground_truth_bearing,ground_truth_range
class ManualCoordinator:
    def __init__(self):
        # Range and bearing calculation service
        self.estimate_bearing = rospy.ServiceProxy('estimate_bearing', estimate_bearing)
        self.estimate_range = rospy.ServiceProxy('estimate_range', estimate_range)
        self.ground_truth_range = rospy.ServiceProxy('ground_truth_range', ground_truth_range)
        self.ground_truth_bearing = rospy.ServiceProxy('ground_truth_bearing')
        # self.addBearingRangeNodes = rospy.ServiceProxy('addBearingRangeNodes', addBearingRangeNodes)

        # # optimize pose service
        # self.optimizeFactorGraph = rospy.ServiceProxy('optimizeFactorGraph', optimizeFactorGraph)

        # Listening to Joystick to call services
        rospy.Subscriber("/ak1/joy", Joy,
                                    self.take_action)

    def take_action(self, msg):
        if int(msg.axes[1]) == 1:
            self.ground_truth_range()
            self.ground_truth_bearing()
            self.estimate_range()
            self.estimate_bearing()
        # elif int(msg.axes[4]) == 1:
        #     self.optimizeFactorGraph()

if __name__ == "__main__":
    try:
        # Init ROS node
        rospy.init_node('manual_coordinator')

        # Instatiated the class
        NC = ManualCoordinator()

        # Wait
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
