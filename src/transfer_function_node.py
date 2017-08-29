#! /usr/bin/env python
"""Transfer function node.

__author__ Denise Ratasich
__date__ 2015-03-25

Subscribes to the topics given in the parameter 'inputs'. Executes the code
passed through parameter 'code'. Finally publishes the result to the topic
given with parameter 'output'. Evaluation and publishing is performed
periodically. The period in seconds can be optionally passed via int-parameter
'period' (100ms by default).

usage: transfer_function_node.py [-h] -i INPUT [INPUT ...] -c CODE -o OUTPUT
                                 [-p PERIOD] [-s START]

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT [INPUT ...], --inputs INPUT [INPUT ...]
                        Input topics including the field, where the input value
                        should be extracted from. Specify full path of topics
                        and the field, e.g., /car/linear_velocity/data.
  -c CODE, --code CODE  Code (Python) to execute. Use x like an array of input
                        values and y as a single output value. The time since
                        last evaluation of the code is captured in the variable
                        T, that you can use in your code too.
  -o OUTPUT, --output OUTPUT
                        Output topic including the field where the calculated
                        value should be published to.
  -p PERIOD, --period PERIOD
                        Evaluation and publishing period in seconds (if not
                        given, the parameter server is searched).
  -s                    start immediately to publish

"""

import roslib
import rospy
import rostopic
import sys
import argparse


_node_name = 'transfer_function'
"""Name of this node in the ROS system."""

_data_in = []
"""Array of input data."""
_data_out = None
"""Output data (a single object)."""
_period = None
"""Evaluation and publishing period

T in the code will be replaced by this value, can be set via the node arguments
(default: 0.1s).

"""


class DataIn:
    """Captures everything to receive and evaluate a message."""

    def __init__(self, topic):
        """Base constructor.

        topic -- The topic including the field to receive.

        """
        rospy.logdebug('data-in: init %s.', topic)

        # save the topic name and initialize the current value
        self._name = topic
        """Topic (full path, i.e., all namespaces included)."""
        self._value = 1.0
        """Value (float) of the input data."""
        self._recv = False
        """Flag indicating that at least one value has been received."""
        # get the topic type and function to evaluate the field
        topic_type, real_topic, field_eval = rostopic.get_topic_type(topic)
        if topic_type is None:
            raise Exception("Can not resolve topic type of {}.".format(topic))
        if field_eval is None:
            raise Exception("Can not resolve field of {}.".format(topic))
        # subscribe and save function for message evaluation
        data_class = roslib.message.get_message_class(topic_type)
        self._subscriber = rospy.Subscriber(real_topic, data_class,
                                            self._callback)
        """Subscriber to a topic - to receive messages."""
        self._field_eval = field_eval
        """Function returning the value of the specificed field in a
        message."""

    def __exit__(self):
        """Called when object is left (i.e., before deletion)."""
        self._subscriber.unregister()

    def _callback(self, msg):
        """Called when a message of the topic is received."""
        self._recv = True
        self._value = self._field_eval(msg)
        rospy.logdebug("Received [%s]: %f", self._name, self._value)

    @property
    def value(self):
        """Returns the latest value received."""
        return self._value


class DataOut:
    """Captures everything to send a message."""

    def __init__(self, topic):
        """Base constructor.

        topic -- The topic including the field to receive.
        """
        rospy.logdebug("data-out: init {}.".format(topic))
        # save the topic name and initialize the current value
        self._name = topic
        """Topic (full path, i.e., all namespaces included)."""
        self._value = 0
        """Value (float) of the output data."""
        # get the topic type
        topic_type, self._topic, _ = rostopic.get_topic_type(topic)
        if topic_type is None:
            raise Exception("""data-out: Can not resolve topic type of
            {}.""".format(topic))
        # get field to write
        self._field = topic[len(self._topic):len(topic)].replace('/', '.')
        """Field where the value shall be saved to."""
        if not self._field:
            raise Exception("""data-out: Can not resolve field of
            {}.""".format(topic))
        # save data class for message instantiation and register publisher
        self._data_class = roslib.message.get_message_class(topic_type)
        """Class of the topic type (needed for instantiation of message)."""
        self._publisher = rospy.Publisher(self._topic, self._data_class,
                                          queue_size=1)
        """Publisher of a topic - to send messages."""

    def __exit__(self):
        """Called when object is left (i.e., before deletion)."""
        self._publisher.unregister()

    def send(self, value):
        """Publishes a value."""
        msg = self._data_class()
        # if this message has a header - fill it
        try:
            msg.header.stamp = rospy.Time.now()
        except Exception as e:
            pass
        # set value and publish
        exec 'msg' + self._field + ' = value'
        self._publisher.publish(msg)
        exec 'value = msg' + self._field
        rospy.logdebug("Published [{}]: {}".format(self._name, value))


def evaluate(x, code_to_exec):
    """Evaluates the code.

    The code may use variable T that is the duration in seconds since last
    evaluation (period).

    """
    T = _period
    rospy.logdebug("Execute with x: %s, T: %.3f", x, T)
    exec code_to_exec
    return y


def task(event):
    """Evaluates code and publishes the output.

    This function is called every _period seconds.

    """
    value = evaluate([d.value for d in _data_in], code_to_exec)
    _data_out.send(value)


def cleanup():
    """Called before shutdown.

    Substitution, sending messages shall be stopped when node shuts
    down. Otherwise a publish during shutdown can happen. Although the message
    wouldn't be sent, it raises a ROSException ("publish on closed topic").

    """
    rospy.logdebug("Cleanup")
    pub_timer.shutdown()


# Main entry point of this node.
if __name__ == '__main__':
    try:
        rospy.init_node(_node_name)

        # get the arguments for this node (no ROS params or
        # remappings)
        argv = rospy.myargv(argv=sys.argv)
        argv.pop(0)  # remove first element, not needed for args parsing
        rospy.logdebug('Arguments list (stripped): %s', argv)
        # arg parsing
        parser = argparse.ArgumentParser()
        parser.add_argument('-i', '--inputs', nargs='+', required=True,
                            metavar="INPUT",
                            help="""Input topics including the field, where the
                            input value should be extracted from. Specify full
                            path of topics and the field, e.g.,
                            /car/linear_velocity/data.""")
        parser.add_argument('-c', '--code', nargs=1, required=True,
                            help="""Code (Python) to execute. Use x like an
                            array of input values and y as a single output
                            value. The time since last evaluation of the code
                            is captured in the variable T, that you can use in
                            your code too.""")
        parser.add_argument('-o', '--output', nargs=1, required=True,
                            help="""Output topic including the field where the
                            calculated value should be published to.""")
        parser.add_argument('-p', '--period', type=float, required=False,
                            help="""Evaluation and publishing period in seconds
                            (if not given, the parameter server is
                            searched).""")
        parser.add_argument('-s', '--start', action='store_true',
                            required=False, default=False,
                            help="""Start immediately to publish.""")
        args = parser.parse_args(argv)

        # transfer function
        topics_in = args.inputs
        """Topics (incl. field) serving as input for the evaluation."""
        rospy.logdebug("Input arg x: %s".format(topics_in))
        code_to_exec = args.code[0]
        """Python code using an input array x to calculate the output value
        y."""
        rospy.logdebug("Input arg code: {}".format(code_to_exec))
        topic_out = args.output[0]
        """Topic (incl. field) where to publish the evaluation result."""
        rospy.logdebug("Input arg y: %s".format(topic_out))

        # init interface to other nodes
        for topic in topics_in:
            _data_in.append(DataIn(topic))
        rospy.logdebug("data-in: %s", [x.__dict__ for x in _data_in])
        _data_out = DataOut(topic_out)
        rospy.logdebug("data-out: %s", _data_out.__dict__)

        # set evaluation and publishing period (given in ms)
        if args.period is not None:
            _period = args.period
            rospy.logdebug("Period (s) from arguments: %.3f", _period)

        # if period not set, there was no 'period' argument, hence check the
        # parameter server for a value
        if _period is None:
            param_name = '/spec/topic/' + _data_out._topic + '/period'
            if rospy.has_param(param_name):
                _period = rospy.get_param(param_name)
                rospy.logdebug("Period (s) from parameter server: %.3f",
                               _period)
            else:
                raise RuntimeError("""Period not specified through arguments
                and cannot be found on parameter server
                ({}).""".format(param_name))

        # register handler for shutdown
        rospy.on_shutdown(cleanup)

        # start when all inputs have been received at least once
        if args.start:
            rospy.logwarn("Publishing without waiting on inputs (x=1.0)")
        else:
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                num_i = 0
                missing = ""
                for i in _data_in:
                    if i._recv:
                        num_i = num_i + 1
                    else:
                        missing = missing + ', ' + i._name
                if num_i == len(_data_in):
                    break
                else:
                    rospy.logwarn("Still waiting on first value %s.", missing)
                r.sleep()

        rospy.loginfo("Initialization done. Start substitution.")

        pub_timer = rospy.Timer(rospy.Duration(_period), task)
        """Timer for evaluating and publishing the output periodically."""
        # timer starts immediately

        # loop over receive-evaluate-send-sleep
        rospy.spin()

        # avoid unresolved race condition during shutdown causing exception
        # https://github.com/ros/ros_comm/issues/527
        rospy.sleep(0.5)
    except Exception as e:
        rospy.logerr("Transfer function node failed. %s", e)
        raise
    except rospy.ROSInterruptException:
        pass
