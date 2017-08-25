#! /usr/bin/env python
"""SHSA node.

__author__ = Denise Ratasich
__date__ = 2017-08-25

ROS wrapper for self-healing by structural adaptation.

usage: shsa_node.py [-h]

optional arguments:
    -h, --help            show this help message and exit
"""

import roslib
import rospy
import rostopic
import sys
import argparse

_node_name = 'shsa_node'
"""Name of this node in the ROS system."""


def _extract_args():
    # get the arguments for this node (no ROS params or remappings)
    argv = rospy.myargv(argv=sys.argv)
    argv.pop(0)  # remove first element, not needed for args parsing
    rospy.logdebug("Arguments list (stripped): %s", argv)
    # parse args
    parser = argparse.ArgumentParser()
    # todo
    args = parser.parse_args(argv)
    return args

def _setup_action_server():
    # todo
    pass

# main entry point of this node
if __name__ == '__main__':
    try:
        # setup
        rospy.init_node(_node_name)
        args = _extract_args()
        _setup_action_server()
        # loop
        rospy.spin()
    except Exception as e:
        rospy.logerr('SHSA node failed. %s', e)
        raise
    except rospy.ROSInterruptException:
        pass
