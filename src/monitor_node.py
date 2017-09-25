#! /usr/bin/env python
"""Monitor node.

__author__ = Denise Ratasich
__date__ = 2017-09-26

Simple watchdog for a topic.

Set parameter ~model (SHSA model containing properties 'need' and 'pubrate').

"""

import yaml

import roslib
import rospy
import rostopic
import actionlib
from actionlib_msgs.msg import GoalStatus

import shsa_ros.msg


_node_name = 'monitor_node'
"""Name of this node in the ROS system."""


class Monitor(object):
    """Watchdog for a topic."""

    def __init__(self, ac, topic, period):
        self.__ac = ac
        self.__topic = topic
        self.__period = period
        # subscribe topic
        ttype, tname, _ = rostopic.get_topic_type(topic)
        if ttype is None:
            raise RuntimeError("""Can not resolve topic type of
            {}.""".format(topic))
        tclass = roslib.message.get_message_class(ttype)
        self.__subscriber = rospy.Subscriber(tname, tclass,
                                             self.__msg_callback)
        rospy.loginfo("Monitor '{}': created.".format(topic))
        # monitoring starts when first message is received
        self.__watchdog = None

    def __exit__(self):
        self.__subscriber.unregister()

    def __msg_callback(self, msg):
        """Called when a message of the topic is received."""
        rospy.logdebug("Monitor '{}': Received message.".format(self.__topic))
        # watchdog restart (unfortunately there is no start/stop)
        if self.__watchdog is not None:
            self.__watchdog.shutdown()
        else:
            rospy.loginfo("""Monitor '{}': start
            monitoring.""".format(self.__topic))
        self.__watchdog = rospy.Timer(rospy.Duration(self.__period*2),
                                      self.__watchdog_callback)

    def __watchdog_callback(self, event):
        """Called when a watchdog overflows."""
        rospy.logwarn("Watchdog overflow. Trigger SHSA.")
        # disable watchdog until dmin receives messages again (SHSA successful)
        self.__watchdog.shutdown()
        self.__watchdog = None
        # start SHSA
        self.__trigger_shsa()

    def __trigger_shsa(self):
        # send goal to action server
        goal = shsa_ros.msg.SubstituteGoal(topic=self.__topic)
        self.__ac.send_goal(goal)
        # wait for the action to return (with timeout)
        finished_on_time = self.__ac.wait_for_result(timeout=rospy.Duration(5))
        result = self.__ac.get_result()
        state = self.__ac.get_state()
        # check result
        if finished_on_time:
            if self.__ac.get_state == GoalStatus.SUCCEEDED:
                rospy.logdebug("""Monitor '{}': SHSA
                successful.""".format(self.__topic))
            else:
                rospy.logdebug("""Monitor '{}': SHSA did not succeed:
                {}.""".format(self.__topic, state))
        else:
            rospy.logdebug("""Monitor '{}': SHSA did not finish before
            timeout.""".format(self.__topic))


# main entry point of this node
if __name__ == '__main__':
    try:
        # setup ROS node
        rospy.init_node(_node_name)
        # setup action client
        client = actionlib.SimpleActionClient('substitute',
                                              shsa_ros.msg.SubstituteAction)
        client.wait_for_server()  # wait on shsa_node
        # get topics to monitor and its publishing rate
        properties = {}  # all properties to provisions
        modelfile = rospy.get_param('~model')
        with open(modelfile, 'r') as f:
            try:
                data = yaml.load(f)
                properties = data['provision_properties']
            except Exception as e:
                raise RuntimeError("""Failed to extract properties of
                provisions from the configuration. {}""".format(e))
        # fill list of needed provisions
        needed = []  # provisions that are in use
        for p in properties:
            try:
                if properties[p]['need'] == True:
                    needed.append(p)
            except KeyError:
                # no need for this provision
                pass
        # create monitor for each needed topic
        monitors = {}
        for provision in needed:
            rate = properties[provision]['pubrate']
            monitors[provision] = Monitor(client, provision, 1.0/rate)
        rospy.loginfo("Monitor node: initialized.")
        # loop
        rospy.spin()
    except Exception as e:
        rospy.logerr('Monitor node failed. %s', e)
        raise
    except rospy.ROSInterruptException:
        pass
