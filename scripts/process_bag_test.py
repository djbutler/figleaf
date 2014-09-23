import process_bag
import rospy
import unittest

class TimeTakenTest(unittest.TestCase):
  def setUp(self):
    self._processor = process_bag.TimeTaken()

  def test_single(self):
    self._processor.update('topic1', 'msg1', rospy.Time(1))
    self.assertEqual(self._processor.first_action(), ('msg1', rospy.Time(1)))
    self.assertEqual(self._processor.last_action(), ('msg1', rospy.Time(1)))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(0))

  def test_unique_start(self):
    self._processor.update('topic1', 'msg1', rospy.Time(1))
    self._processor.update('topic1', 'msg2', rospy.Time(2))
    self._processor.update('topic1', 'msg3', rospy.Time(3))
    self.assertEqual(self._processor.first_action(), ('msg1', rospy.Time(1)))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(2))

  def test_repeated_start(self):
    self._processor.update('topic1', 'msg1', rospy.Time(1))
    self._processor.update('topic1', 'msg1', rospy.Time(2))
    self._processor.update('topic1', 'msg2', rospy.Time(3))
    self.assertEqual(self._processor.first_action(), ('msg1', rospy.Time(2)))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(1))

  def test_unique_end(self):
    self._processor.update('topic1', 'msg1', rospy.Time(1))
    self._processor.update('topic1', 'msg2', rospy.Time(2))
    self._processor.update('topic1', 'msg3', rospy.Time(3))
    self.assertEqual(self._processor.last_action(), ('msg3', rospy.Time(3)))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(2))

  def test_repeated_end(self):
    self._processor.update('topic1', 'msg1', rospy.Time(1))
    self._processor.update('topic1', 'msg2', rospy.Time(2))
    self._processor.update('topic1', 'msg2', rospy.Time(3))
    self.assertEqual(self._processor.last_action(), ('msg2', rospy.Time(2)))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(1))

  def test_start_across_topics(self):
    self._processor.update('topic2', 'msg1', rospy.Time(1))
    self._processor.update('topic1', 'msg2', rospy.Time(2))
    self._processor.update('topic2', 'msg3', rospy.Time(3))
    self._processor.update('topic1', 'msg4', rospy.Time(4))
    self.assertEqual(self._processor.first_action(), ('msg1', rospy.Time(1)))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(3))

  def test_end_across_topics(self):
    self._processor.update('topic2', 'msg3', rospy.Time(1))
    self._processor.update('topic1', 'msg1', rospy.Time(2))
    self._processor.update('topic2', 'msg4', rospy.Time(3))
    self._processor.update('topic1', 'msg2', rospy.Time(4))
    self.assertEqual(self._processor.last_action(), ('msg2', rospy.Time(4)))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(3))

  def test_repeated_and_across_topics(self):
    self._processor.update('topic1', 'msg1', rospy.Time(1))
    self._processor.update('topic2', 'msg2', rospy.Time(2))
    self._processor.update('topic1', 'msg1', rospy.Time(3))
    self.assertEqual(self._processor.first_action(), ('msg2', rospy.Time(2)))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(0))
    self.assertEqual(self._processor.last_action(), ('msg2', rospy.Time(2)))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(0))

  def test_no_negative(self):
    self._processor.update('topic1', 'msg1', rospy.Time(1))
    self._processor.update('topic1', 'msg1', rospy.Time(2))
    self.assertEqual(self._processor.time_taken(), rospy.Duration(0))

if __name__ == '__main__':
  unittest.main()
