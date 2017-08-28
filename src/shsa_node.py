#! /usr/bin/env python3
"""SHSA node.

__author__ = Denise Ratasich
__date__ = 2017-08-25

ROS wrapper for self-healing by structural adaptation.

Set parameters ~model (SHSA model) and ~map (variable to topic mapping).

"""

import sys
import argparse
import yaml

import roslib
import rospy
import actionlib
import shsa_ros.msg

# there should be a more appropriate way of including the shsa library
sys.path.insert(0, "/home/denise/repos/work/research/shsa/shsa/")
from engine.greedy import Greedy


_node_name = 'shsa_node'
"""Name of this node in the ROS system."""


class Map(object):
    """Translates topic to variable (ROS to model), and vice-versa."""
    def __init__(self, configfile):
        self.__configfile = configfile
        self.__data = None
        # open config and read yaml
        with open(configfile, 'r') as f:
            try:
                self.__data = yaml.load(f)
                datacpy = dict(self.__data)
            except yaml.YAMLError as e:
                raise RuntimeError("""Failed to read map config
                ({}). {}""".format(configfile, e))
        # make map bi-directional (reversed duplicate)
        for k,v in datacpy.items():
            self.__data[v] = k

    def topic(self, variable):
        return self.__data[variable]

    def variable(self, topic):
        return self.__data[topic]


class SubstituteServer(object):
    def __init__(self, modelfile, mapfile):
        self.__engine = Greedy(configfile=modelfile)
        """SHSA engine."""
        self.__map = Map(mapfile)
        """Mapping ROS topics to SHSA model variables."""
        self.__tnn = 0
        """ID of next transfer node, or number of transfer nodes created."""
        self.__server = actionlib.SimpleActionServer('substitute',
                                                     shsa_ros.msg.SubstituteAction,
                                                     self.execute, False)
        self.__server.start()
        # prepare result message
        self.__result = shsa_ros.msg.SubstituteResult()
        """Result message, if the substitution succeeds."""

    def __start_transfer_node(self, s):
        if s is None or len(s) == 0:
            raise RuntimeError("""Substitution result empty.""")
        rospy.logdebug("create transfer node for {}".format(s))
        # todo: gather input, code, output
        # cmd
        node_name = "transfer_node_{}".format(self.__tnn)
        cmd = "rosrun dynamic_reconfiguration transfer_function_node.py"
        # special keyword/argument for ros to set node name
        cmd += " __name:={}".format(node_name)
        # 3 positional arguments for x, code and y
	# cmd += " -i {}".format("x")
	# cmd += " -c {}".format("y=x")
	# cmd += " -o {}".format("y")
        # todo: start node
        self.__tnn += 1
        return node_name

    def execute(self, goal):
        rospy.loginfo("Substitution of '{}' requested.".format(goal.topic))
        try:
            variable = self.__map.variable(goal.topic)
            s = self.__engine.substitute(variable)
            self.__result.node = self.__start_transfer_node(s.best())
            rospy.loginfo("Substitution of '{}' succeeded.".format(goal.topic))
            self.__server.set_succeeded(self.__result)
        except Exception as e:
            rospy.logwarn("Substitution of '{}' failed. {}".format(goal.topic,
                                                                   e))
            self.__server.set_aborted()


# main entry point of this node
if __name__ == '__main__':
    try:
        # setup
        rospy.init_node(_node_name)
        model = rospy.get_param('~model')
        vmap = rospy.get_param('~map')
        aserver = SubstituteServer(model, vmap)
        # loop
        rospy.spin()
    except Exception as e:
        rospy.logerr('SHSA node failed. %s', e)
        raise
    except rospy.ROSInterruptException:
        pass
