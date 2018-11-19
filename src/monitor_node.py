#! /usr/bin/env python
"""Monitor node.

__author__ = Denise Ratasich
__date__ = 2017-11-12

SHSA monitor for a topic.

Major parts are adopted form transfer_function_node.py.

"""

import roslib
import rospy
import rostopic
import actionlib
from actionlib_msgs.msg import GoalStatus

import shsa_ros.msg

# don't forget to add the shsa problog library to your PYTHONPATH
from model.monitor import Monitor as SHSAMonitor
from model.itom import Itom, Itoms


_node_name = 'monitor_node'
"""Name of this node in the ROS system."""

_data_in = {}
"""Dictionary of input data."""

_monitor = None
"""SHSA Monitor."""

_ac = None
"""Action client to trigger substitution."""


class DataIn(Itom):
    """Captures everything to receive and evaluate a message."""

    def __init__(self, topic):
        """Base constructor.

        topic -- The topic including the field to receive.

        """
        rospy.logdebug('data-in: init %s.', topic)
        # save some topic information
        topic_class, self._topic, self._fields = self._get_topic_type(topic)
        self._recv = False
        """Flag indicating that at least one value has been received."""
        # subscribe and install callback for message
        self._subscriber = rospy.Subscriber(self._topic, topic_class,
                                            self._callback)
        """Subscriber to a topic - to receive messages."""
        # init underlying itom with name=topic and a default value
        super(DataIn, self).__init__(topic, 1.0)

    def _get_topic_type(self, topic):
        # get the topic type and function to evaluate the field
        topic_type, real_topic, _ = rostopic.get_topic_type(topic)
        if topic_type is None:
            raise Exception("Can not resolve topic type of {}.".format(topic))
        # nb: last argument field_eval does not work -> split on our own
        # (field eval does not work for arrays?)
        # -> split field(s) from topic
        fields = topic[len(real_topic)+1:]
        fields = fields.split("/") if len(fields) > 0 else []
        # generate class from topic_type
        data_class = roslib.message.get_message_class(topic_type)
        return data_class, real_topic, fields

    def __exit__(self):
        """Called when object is left (i.e., before deletion)."""
        self._subscriber.unregister()

    def _callback(self, msg):
        """Called when a message of the topic is received."""
        self._recv = True
        v = msg
        for field in self._fields:
            v = getattr(v, field)
        self.v = v
        try:
            self.t = msg.header.stamp.to_sec()
        except Exception as e:
            pass
        rospy.logdebug("Received [%s]: %f", self.name, self.v)


def trigger_shsa(topic):
    # send goal to action server
    goal = shsa_ros.msg.SubstituteGoal(topic=topic)
    _ac.send_goal(goal)
    # wait for the action to return (with timeout)
    finished_on_time = _ac.wait_for_result(timeout=rospy.Duration(5))
    result = _ac.get_result()
    state = _ac.get_state()
    # check result
    if finished_on_time:
        if _ac.get_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("""Monitor '{}': SHSA successful.""".format(topic))
        else:
            rospy.logwarn("""Monitor '{}': SHSA did not succeed ({}).""".format(topic, state))
    else:
        rospy.logwarn("""Monitor '{}': SHSA did not finish before timeout.""".format(topic))

def task(event):
    # monitor periodically
    # cast/copy data to itom
    # due to error "TypeError: can't pickle thread.lock objects"
    itoms = Itoms()
    for k, d in _data_in.items():
        itoms[k] = Itom(d.name, d.v, timestamp=d.t, variable=d.variable)
    # check
    failed = _monitor.monitor(itoms)
    if failed is not None:
        # stop monitoring
        _pub_timer.shutdown()
        # substitute output itom of the substitution
        # (assumption: variable monitored is a needed variable)
        failed_itom = list(failed.vin)[0].name
        rospy.logwarn("Monitor node: mismatch detected -> trigger recovery of {} ...".format(failed_itom))
        trigger_shsa(failed_itom)
        rospy.loginfo("Monitor node: stop.".format(failed_topic))


# main entry point of this node
if __name__ == '__main__':
    try:
        # setup ROS node
        rospy.init_node(_node_name)

        # setup monitor
        modelfile = rospy.get_param('~model')
        variable = rospy.get_param('~variable')
        problog_paths = rospy.get_param('~problogpaths')
        _monitor = SHSAMonitor(modelfile, variable, librarypaths=problog_paths)

        # collect necessary input topics (itoms from monitor substitutions)
        S = _monitor.substitutions
        rospy.loginfo("Monitor node: use %d substitutions", len(S))
        for s in S:
            for v in s.vin:
                _data_in[v.name] = DataIn(v.name)
        rospy.logdebug("data-in: %s", [str(x) for x in _data_in.keys()])

        # setup action client
        rospy.loginfo("Monitor node: wait for action server (shsa_node) ...")
        _ac = actionlib.SimpleActionClient('substitute',
                                           shsa_ros.msg.SubstituteAction)
        _ac.wait_for_server()  # wait on shsa_node

        rospy.loginfo("Monitor node: initialized.")

        # start when all inputs have been received at least once
        rospy.logwarn("Monitor node: Waiting on inputs ...")
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            missing = [d.name for d in _data_in.values() if not d._recv]
            if len(missing) > 0:
                rospy.logwarn("Monitor node: Still waiting on first value of: %s.", ", ".join(missing))
                r.sleep()
                continue
            break

        rospy.loginfo("Monitor node: Initialization done. Start monitoring.")

        _pub_timer = rospy.Timer(rospy.Duration(0.1), task)
        """Timer for checking the inputs periodically."""
        # timer starts immediately

        # loop
        rospy.spin()

        r.sleep()
        _pub_timer.shutdown()
    except Exception as e:
        rospy.logerr('Monitor node failed. %s', e)
        raise
    except rospy.ROSInterruptException:
        pass
