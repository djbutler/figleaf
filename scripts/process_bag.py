"""Processes the rosbag data from the study.
"""

from __future__ import division
from __future__ import print_function

import rosbag
import rospy
import sys

class TimeTaken(object):
  def __init__(self):
    self._first_message = None
    self._first_time = None
    self._first_actions = {}
    self._last_message = None
    self._last_time = None
    self._last_actions = {}

  def update(self, topic, message, time):
    if topic in self._first_actions:
      if self._first_actions[topic][0] == message:
        self._first_actions[topic] = (message, time)
    else:
      self._first_actions[topic] = (message, time)

    if topic in self._last_actions:
      if self._last_actions[topic][0] != message:
        self._last_actions[topic] = (message, time)
    else:
      self._last_actions[topic] = (message, time)

  def first_action(self):
    if self._first_message is None:
      for message, time in self._first_actions.values():
        if self._first_time is None or time < self._first_time:
          self._first_message = message
          self._first_time = time
      return self._first_message, self._first_time
    else:
      return self._first_message, self._first_time

  def last_action(self):
    if self._last_message is None:
      for message, time in self._last_actions.values():
        if self._last_time is None or time > self._last_time:
          self._last_message = message
          self._last_time = time
      return self._last_message, self._last_time
    else:
      return self._last_message, self._last_time

  def time_taken(self):
    first_message, first_time = self.first_action()
    last_message, last_time = self.last_action()
    time = last_time - first_time
    if time.to_sec() < 0:
      return rospy.Duration(0)
    else:
      return time

def main():
  filename = sys.argv[1]
  with rosbag.Bag(filename) as bag:
    time_taken_processor = TimeTaken()
    for topic, message, time in bag.read_messages():
      time_taken_processor.update(topic, message, time)
    print('Time taken: {} seconds'.format(
      time_taken_processor.time_taken().to_sec()))

if __name__ == '__main__':
  main()
