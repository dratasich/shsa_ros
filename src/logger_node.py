#! /usr/bin/env python
"""Logger node.

__author__ = Denise Ratasich
__date__ = 2018-03-09

Temporally aligned CSV logger of topics.

Received information is always buffered. All buffers (containing the last
received values) are periodically logged to a CSV. Logging starts when at least
one message of each specified topic has been received.

Parameters:
- ~topics to define which topics shall be subscribed and which fields
  extracted.
- ~rate defines the logging rate, default: 10Hz.
- ~path to CSV file.

"""

import csv
import datetime
import os

import roslib
import rospy
import rostopic

_node_name = 'logger_node'
"""Name of this node in the ROS system."""


class DataIn(object):
    """Receiver and buffer of a topic."""

    def __init__(self, topic, name=None):
        """Constructor.

        topic -- topic including field to receive.
        name -- abbreviation for the topic used as column header in the CSV.

        """
        self.__topic = topic
        """Topic including field (full path, i.e., all namespaces included)."""
        if name is None:
            self.__name = topic
        else:
            self.__name = name
            """Abbreviation for the topic used as column header."""
        self.__value = None  # no message received yet
        """Value of the input data."""
        # get the topic type and function to evaluate the field
        # wait until topic gets available
        rospy.loginfo("DataIn '%s': await availability...", topic)
        topic_type, real_topic, field_eval = rostopic.get_topic_type(topic,
                                                                     blocking=True)
        if topic_type is None:
            raise Exception("Can not resolve topic type of {}.".format(topic))
        if field_eval is None:
            raise Exception("Can not resolve field of {}.".format(topic))
        # subscribe and save function for message evaluation
        data_class = roslib.message.get_message_class(topic_type)
        self.__subscriber = rospy.Subscriber(real_topic, data_class,
                                            self.__msg_callback)
        """Subscriber to a topic - to receive messages."""
        self.__field_eval = field_eval
        """Function returning the value of the specificed field in a
        message."""
        rospy.loginfo("DataIn '%s': created.", topic)

    def __exit__(self):
        self.__subscriber.unregister()

    def __msg_callback(self, msg):
        """Called when a message of the topic is received."""
        self.__value = self.__field_eval(msg)
        rospy.logdebug("DataIn '%s' received: %f", self.__topic, self.__value)

    def __get_name(self):
        """Returns the name of the input."""
        return self.__name

    name = property(__get_name)

    def __get_value(self):
        """Returns the latest value received."""
        return self.__value

    value = property(__get_value)


class Logger(object):
    """Logs inputs to a CSV."""

    def __init__(self, path, data, log_time=True):
        """Constructor.

        path -- path to CSV.
        data -- list of DataIn objects.
        log_time -- enables logging of timestamps.

        """
        self.__data = data  # DataIn objects
        self.__path = path  # path to csv to write logs to
        self.__writer = None  # csv writer
        self.__enabled = False  # enable write to csv
        self.__log_time = log_time
        # open csv file
        csvfile = open(path, 'w')
        self.__writer = csv.writer(csvfile)
        # write header
        fieldnames = [d.name for d in self.__data]
        if self.__log_time:
            fieldnames.insert(0, "time")
        rospy.logdebug("CSV fields: %s.", fieldnames)
        fieldnames[0] = '%' + fieldnames[0]  # header is a comment
        self.__writer.writerow(fieldnames)
        rospy.loginfo("Logger will write to %s.",
                      os.path.abspath(csvfile.name))

    def __exit__(self):
        print "close csv file"
        csvfile.close()

    def log(self, event):
        values = [d.value for d in self.__data]
        if self.__enabled:
            if self.__log_time:
                curtime = rospy.Time.now()
                values.insert(0, "{:d}.{:09d}".format(curtime.secs,
                                                      curtime.nsecs))
            self.__writer.writerow(values)
            rospy.logdebug("New row: %s.", values)
        else:
            # enable logger as soon as all inputs received at least once
            received = [1 for v in values if v is not None]
            if sum(received) == len(values):
                rospy.loginfo("Start logging (next cycle).")
                self.__enabled = True


# main entry point of this node
if __name__ == '__main__':
    try:
        # setup ROS node
        rospy.init_node(_node_name)

        # params
        topics = rospy.get_param('~topics', None)
        rate = rospy.get_param('~rate', 10)
        # path ... create default string from-ROS-time
        now = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        path = rospy.get_param('~path', now + ".csv")

        # create a port for each topic
        inputs = []
        try:
            for name, topic in topics.items():
                inputs.append(DataIn(topic, name=name))
        except Exception as e:
            raise RuntimeError("""Failed to subscribe to topics
            {}. {}""".format(topics, e))

        # initialize logger
        if len(inputs) == 0:
            raise RuntimeError("""No topics/inputs.""")
        logger = Logger(path, inputs)

        # create and start timer for logging
        pub_timer = rospy.Timer(rospy.Duration(1.0/rate), logger.log)
        """Timer for logging the received data periodically."""

        rospy.loginfo("Logger node initialized.")

        # loop over receive-log-sleep
        rospy.spin()

        # cleanup
        pub_timer.shutdown()
        rospy.sleep(0.5)
    except Exception as e:
        rospy.logerr('Logger node failed. %s', e)
        raise
    except rospy.ROSInterruptException:
        pass
