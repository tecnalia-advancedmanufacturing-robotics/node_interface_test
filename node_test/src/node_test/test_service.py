#!/usr/bin/env python
"""
@package node_test
@file test_service
@author Anthony Remazeilles
@brief perform a unittest on a ROS service call

Copyright (C) 2020 Tecnalia Research and Innovation
Distributed under the Apache 2.0 license.

"""

import sys
import unittest
import rospy
import rosservice
import rostest
import genpy
from rospy_message_converter import message_converter

CLASSNAME = 'servicetest'


class ServiceTest(unittest.TestCase):
    def __init__(self, *args):
        super(ServiceTest, self).__init__(*args)

    def setUp(self):
        rospy.init_node(CLASSNAME)

    def test_service(self):
        self.calls = list()

        try:
            calls = rospy.get_param('~calls')

            for call in calls:

                keys = ['name', 'input', 'output']
                for item in keys:
                    if item not in call:
                        self.fail("{} field required, but not specified in {}".format(item, call))

                srv_data = dict()

                srv_data['name'] = call['name']
                srv_data['input'] = call['input']
                srv_data['output'] = call['output']

                if srv_data['input'] == 'None':
                    rospy.logwarn('None input converted to empty input')
                    srv_data['input'] = dict()
                if srv_data['output'] == 'None':
                    rospy.logwarn('None output converted to empty output')
                    srv_data['output'] = dict()
                self.calls.append(srv_data)
        except KeyError as err:
            msg_err = "service_test not initialized properly"
            msg_err += " Parameter [%s] not set." % (str(err))
            msg_err += " Caller ID: [%s] Resolved name: [%s]" % (
                rospy.get_caller_id(),
                rospy.resolve_name(err.args[0]))
            self.fail(msg_err)

        for item in self.calls:
            rospy.loginfo("Testing service {} with input parameters {}".format(
                item['name'],
                item['input']))
            self._test_service(item['name'], item['input'], item['output'])
            rospy.loginfo("So far so good")

    def _test_service(self, srv_name, srv_input, srv_output):
        self.assert_(srv_name)

        all_services = rosservice.get_service_list()
        self.assertIn(srv_name, all_services)

        srv_class = rosservice.get_service_class_by_name(srv_name)

        try:
            srv_proxy = rospy.ServiceProxy(srv_name, srv_class)
        except KeyError as err:
            msg_err = "Service proxy could not be created"
            self.fail(msg_err)

        try:
            if srv_input:
                srv_resp = srv_proxy(**srv_input)
            else:
                srv_resp = srv_proxy()

        except (genpy.SerializationError, rospy.ROSException), err:
            msg_err = "Service proxy error: {}".format(err.message)
            self.fail(msg_err)
        srv_dic = message_converter.convert_ros_message_to_dictionary(srv_resp)

        self.assertDictEqual(srv_dic, srv_output)


def main():
    try:
        rostest.run('rostest', CLASSNAME, ServiceTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("{} exiting".format(CLASSNAME))
