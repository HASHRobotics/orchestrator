#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from colocalization.srv import addBearingRangeNodes, optimizeFactorGraph

class ManualCoordinator:
    def __init__(self):
        # Range and bearing calculation service
        self.addBearingRangeNodes = rospy.ServiceProxy('addBearingRangeNodes', addBearingRangeNodes)

        # optimize pose service
        self.optimizeFactorGraph = rospy.ServiceProxy('optimizeFactorGraph', optimizeFactorGraph)

        # Listening to Joystick to call services
        rospy.Subscriber("ak1/joy", Joy,
                                    self.take_action)

    def take_action(self, msg):
        # TODO: Change joystick buttons
        if int(msg.axes[7]) == 1:
            self.addBearingRangeNodes()
        # TODO: Change joystick buttons
        elif int(msg.axes[8]) == 1:
            self.optimizeFactorGraph()

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
