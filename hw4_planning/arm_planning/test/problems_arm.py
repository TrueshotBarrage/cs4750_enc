#!/usr/bin/env python

import numpy as np
import rosunit
import unittest

from nav_msgs.msg import MapMetaData

from planning.problems import JointSpace

class TestJSProblem(unittest.TestCase):
    def setUp(self):
        self.problem = JointSpace()
        pass

    def test_joint_space_heuristic_2pi(self):
        # This sampler creates vertices for x and y = (1.67, 5.0, 8.33).
        # Only the state (5.0, 5.0) should be in collision.

        config = np.array([2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi]).reshape((1, 6))
        goal_config = np.array([0, 0, 0, 0, 0, 0]).reshape((1, 6))
        heuristics = self.problem.cost_to_go(config, goal_config)

        np.testing.assert_allclose(
            heuristics,
            0.0,
            err_msg="2*pi and 0 is the same configuration. There shouldn't be any cost for an arm to  move from 0 to 2*pi",
        )

    def test_joint_space_heuristic(self):
        # This sampler creates vertices for x and y = (1.67, 5.0, 8.33).
        # Only the state (5.0, 5.0) should be in collision.

        config = np.array([0.3, 0.4, 0.2, -0.2, 0.3, 0.1]).reshape((1, 6))
        goal_config = np.array([0.1, 0.3, 0.2, 0.0, 0.0, 0.0]).reshape((1, 6))
        heuristics = self.problem.cost_to_go(config, goal_config)

        np.testing.assert_allclose(
            heuristics,
            0.9,
            err_msg="heuristic incorrect",
        )


if __name__ == "__main__":
    rosunit.unitrun("planning", "test_joint_space_problem", TestJSProblem)
