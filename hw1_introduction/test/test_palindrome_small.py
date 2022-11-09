#!/usr/bin/env python

import rospy
import rostest
import unittest

from std_msgs.msg import Bool


class TestpalindromeSmall(unittest.TestCase):
    def test_palindrome_small(self):
        message = rospy.wait_for_message("/introduction/palindrome_output", Bool, timeout=5)
        self.assertEqual(message.data, True)


if __name__ == "__main__":
    rospy.init_node("test_palindrome_small")
    rostest.rosrun("introduction", "test_palindrome_small", TestpalindromeSmall)
