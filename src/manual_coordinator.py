#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
# from colocalization.srv import addBearingRangeNodes, optimizeFactorGraph
from bearing_estimator.srv import estimate_bearing,estimate_range,ground_truth_bearing,ground_truth_range
class ManualCoordinator:
    def __init__(self):
        # Range and bearing calculation service
        self.estimate_bearing = rospy.ServiceProxy('/ak1/estimate_bearing', estimate_bearing)
        self.estimate_range = rospy.ServiceProxy('estimate_range', estimate_range)
        self.ground_truth_range = rospy.ServiceProxy('ground_truth_range', ground_truth_range)
        self.ground_truth_bearing = rospy.ServiceProxy('/ak1/ground_truth_bearing', ground_truth_bearing)
        # self.addBearingRangeNodes = rospy.ServiceProxy('addBearingRangeNodes', addBearingRangeNodes)

        # # optimize pose service
        # self.optimizeFactorGraph = rospy.ServiceProxy('optimizeFactorGraph', optimizeFactorGraph)

    def take_action(self,msg):
        if int(msg.buttons[1]) == 1:
            rospy.logwarn("Button Pressed")
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
        
        while(not rospy.is_shutdown()):
            msg = rospy.wait_for_message("/ak2/joy", Joy)
            NC.take_action(msg)

        # Wait
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
