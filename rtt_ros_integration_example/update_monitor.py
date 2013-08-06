#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

def updated(req):
    rospy.loginfo("Updated!")
    return EmptyResponse()

def main():
    rospy.init_node("update_monitor")

    server = rospy.Service('updated', Empty, updated)

    rospy.spin()

if __name__ == '__main__':
    main()
