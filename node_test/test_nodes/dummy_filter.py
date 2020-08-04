#!/usr/bin/env python
"""
@package node_test
@file dummy filter
@author Anthony Remazeilles <anthony.remazeilles@tecnalia.com>
@brief dummy prb to check filter testing

Copyright (C) 2020 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license.

"""


import rospy
from std_msgs.msg import Float32


class DummyFilterNode():
    def __init__(self):
        self.pub = None
        self.sub = None
        self.wait = None

        rospy.init_node('dummy_filter', anonymous=True)

        self.wait = rospy.get_param('~wait', self.wait)
        print "waiting time: {}".format(self.wait)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        data.data = data.data * 2.0
        print self.wait
        if self.wait is not None:
            print "Sleeping!"
            duration = rospy.Duration(self.wait)
            rospy.sleep(duration)
        self.pub.publish(data)

    def run(self):
        self.pub = rospy.Publisher('filter_out', Float32, queue_size=10)
        self.sub = rospy.Subscriber('filter_in', Float32, self.callback)

        rospy.spin()


if __name__ == '__main__':
    try:
        dummy = DummyFilterNode()
        dummy.run()
    except rospy.ROSInterruptException:
        pass
