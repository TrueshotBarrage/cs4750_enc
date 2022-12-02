#!/usr/bin/env python

import numpy as np
import os
import rosunit
import unittest

from planning import search
from planning.problems import R2Problem


class TestRRT(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        self.permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = R2Problem(self.permissible_region, check_resolution=0.5)

    def test_rrt_r2problem(self):

        rrt = search.RRTPlanner(self.problem, self.permissible_region)

        start = np.array([[1., 1.]])
        goal = np.array([[8., 8.]])
        correct_path = np.array([[1, 1],
                                [1, 8],
                                [7, 8],
                                [8, 8]])


        path = rrt.Plan(start, goal)

        self.assertTrue(
            np.allclose(path[0], start),
            msg="Path must begin at start",
        )

        self.assertTrue(
            np.allclose(path[-1], goal),
            msg="Path must end at goal",
        )

        cost_path = 0
        for i in range(path.shape[0] - 1):
            cost_path += np.linalg.norm(path[i+1] - path[i])

        cost_correct = 0
        for i in range(correct_path.shape[0] - 1):
            cost_correct += np.linalg.norm(correct_path[i+1] - correct_path[i])

        np.testing.assert_almost_equal(
            cost_path, cost_correct, 
            err_msg="RRT implementation is incorrect",
        )
        
        self.assertTrue(
            np.allclose(path, correct_path),
            msg="RRT implementation is incorrect",
        )

if __name__ == "__main__":
    np.random.seed(111)
    rosunit.unitrun("planning", "test_rrt_r2", TestRRT)