#! /usr/bin/env python2.7
"""SHSA node.

__author__ = Denise Ratasich
__date__ = 2017-08-25

ROS wrapper for self-healing by structural adaptation.

Set parameters ~model (SHSA model).

"""

import sys
import yaml
import subprocess
import networkx as nx

import roslib
import rospy
import rostopic
import actionlib
import shsa_ros.msg

# don't forget to add the shsa library to your PYTHONPATH
from engine.shpgsa import SHPGSA


_node_name = 'shsa_node'
"""Name of this node in the ROS system."""

_goals = []
"""Save substituted goals to prevent from re-substitution -- though this should
not be necessary!"""

class Map(object):
    """Translates topic to variable (ROS to model), and vice-versa."""
    def __init__(self, model):
        self.__data = None
        # get mappings from model
        try:
            self.__data = nx.get_node_attributes(model, 'provision')
            self.__data.update(nx.get_node_attributes(model, 'need'))
            datacpy = dict(self.__data)
        except Exception as e:
            raise RuntimeError("""Failed to extract provisions from SHSA
            model. {}""".format(e))
        # make map bi-directional (reversed duplicate)
        for k, V in datacpy.items():
            # value=provisions, is a list of topics
            for v in V:
                self.__data[v] = k

    def topic(self, variable):
        provisions = self.__data[variable]
        if len(provisions) == 0:
            raise RuntimeError("""No provision available for variable
            {}.""".format(variable))
        # TODO: which provision to return
        return provisions[0]

    def variable(self, topic):
        if "/" not in topic:
            raise RuntimeError("""'{}' is not a topic.""".format(topic))
        return self.__data[topic]


class SubstituteServer(object):
    def __init__(self, modelfile):
        self.__engine = SHPGSA(configfile=modelfile)
        """SHSA engine."""
        self.__map = Map(self.__engine.model)
        """Mapping ROS topics to SHSA model variables."""
        self.__tnn = 0
        """ID of next transfer node, or number of transfer nodes created."""
        self.__server = actionlib.SimpleActionServer(
            'substitute', shsa_ros.msg.SubstituteAction, self.execute, False)
        self.__server.start()
        # prepare result message
        self.__result = shsa_ros.msg.SubstituteResult()
        """Result message, if the substitution succeeds."""

    def __gen_code(self, s):
        """Generates arguments for the transfer node.

        Returns the input topics and the python code to execute. The transfer
        node executes the code in the form of "y=f(x)". Whereas x is a vector
        of values received from the input topics incl. field, and y is the
        value of the output topic incl. field.

        """
        # dfs of substitution tree
        t, vin = s.tree(collapse_variables=False)
        postorder = nx.dfs_postorder_nodes(t)
        constants = nx.get_node_attributes(s.model, 'constant')
        inputs = []
        code = "\n"
        # read input
        for n in vin:
            if n in constants.keys():
                # handle constants (no topics, no subscription necessary)
                code += n + " = " + str(constants[n]) + "\n"
            else:
                # handle provisions
                code += n + " = x[" + str(len(inputs)) + "]\n"
                inputs.append(self.__map.topic(n))
        code += "\n"
        # execute functions
        for n in postorder:
            if s.model.is_relation(n):
                # I/O variables of relation
                iv = ",".join(t.predecessors(n))  # input
                ov = list(t.successors(n))[0]  # output
                # define relation as function
                code += "def " + n + "(" + iv + "):\n"
                code += "    return (" \
                        + s.model.property_value_of(n, 'fct')[ov] \
                        + ")\n\n"
                # execute relation
                code += ov + " = " + n + "(" + iv + ")\n\n"
        # set output
        code += "y = " + s.root  # assign output
        return ",".join(inputs), code

    def __start_transfer_node(self, o, s):
        """Creates and starts the transfer node."""
        if s is None or len(s) == 0:
            raise RuntimeError("""Substitution result empty.""")
        rospy.logdebug("create transfer node for {}".format(s))
        # cmd
        node_name = "transfer_node_{}".format(self.__tnn)
        # call node
        cmd = "rosrun shsa_ros transfer_function_node.py"
        # special keyword/argument for ros to set node name
        cmd += " __name:={}".format(node_name)
        # arguments for inputs, code, outputs, period
        i, c = self.__gen_code(s)
        # set period from shsa model
        p = s.model.property_value_of(s.root, 'period')
        cmd += " -p {}".format(p)
        cmd += " -i \"{}\"".format(i)
        cmd += " -c \"{}\"".format(c)
        cmd += " -o \"{}\"".format(o)
        rospy.logdebug("run transfer node:\n{}".format(cmd))
        # start node
        subprocess.Popen(cmd, shell=True)
        self.__tnn += 1
        return node_name

    def execute(self, goal):
        global _goals
        rospy.loginfo("Substitution of '{}' requested.".format(goal.topic))
        if goal.topic in _goals:
            self.__server.set_aborted()
            rospy.logwarn("Another substitution of '{}' aborted.".format(goal.topic))
            return
        try:
            variable = self.__map.variable(goal.topic)
            while(self.__engine.substitute(variable)):
                pass
            s = self.__engine.last_results()
            rospy.logdebug("Substitutions found:\n{}".format(s))
            self.__result.node = self.__start_transfer_node(goal.topic,
                                                            s.best())
            rospy.loginfo("Substitution of '{}' succeeded.".format(goal.topic))
            self.__server.set_succeeded(self.__result)
            _goals.append(goal.topic)
        except Exception as e:
            rospy.logwarn("Substitution of '{}' failed. {}".format(goal.topic,
                                                                   e))
            raise
            self.__server.set_aborted()


# main entry point of this node
if __name__ == '__main__':
    try:
        # setup
        rospy.init_node(_node_name)
        model = rospy.get_param('~model')
        aserver = SubstituteServer(model)
        # loop
        rospy.spin()
    except Exception as e:
        rospy.logerr('SHSA node failed. %s', e)
        raise
    except rospy.ROSInterruptException:
        pass
