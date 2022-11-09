#!/usr/bin/env python3
from __future__ import division
import numpy as np
import rosunit
import unittest

from arm_kinematics.fk_broadcaster import Foward_Kinematics_Broadcaster
from tf.transformations import quaternion_from_matrix, translation_from_matrix


class TestForwardKinematics(unittest.TestCase):
    def testEEPos1(self):
        fkb= Foward_Kinematics_Broadcaster()
        q_waist = 2.19
        q_shoulder = -0.96
        q_elbow = 0.86
        computed_trans = fkb.compute_fk(q_waist, q_shoulder, q_elbow)
        computed_pos = np.squeeze(np.asarray(computed_trans[0:3, 3]))
        groud_truth = np.array([-0.134, 0.187, 0.334]).reshape((3,))
        np.testing.assert_allclose(computed_pos, groud_truth, rtol=0, atol=0.01,
        err_msg="test 1: Wrong position")

    def testEEPos2(self):
        fkb= Foward_Kinematics_Broadcaster()
        q_waist = -1.46
        q_shoulder = 1.74
        q_elbow = 0.01
        computed_trans = fkb.compute_fk(q_waist, q_shoulder, q_elbow)
        computed_pos = np.squeeze(np.asarray(computed_trans[0:3, 3]))
        groud_truth = np.array([0.018, -0.162, -0.383]).reshape((3,))
        np.testing.assert_allclose(computed_pos, groud_truth, rtol=0, atol=0.01,
        err_msg="test 2: Wrong position")

    def testEEOri1(self):
        fkb= Foward_Kinematics_Broadcaster()
        q_waist = -1.46
        q_shoulder = 1.74
        q_elbow = 0.01
        computed_trans = fkb.compute_fk(q_waist, q_shoulder, q_elbow)
        computed_quaternion = np.array(quaternion_from_matrix(computed_trans)).reshape((4,))
        groud_truth = np.array([0.513, 0.573, -0.427, 0.476]).reshape((4,))
        np.testing.assert_allclose(computed_quaternion, groud_truth, rtol=0, atol=0.01,
        err_msg="test 2: Wrong position")
        

    def testEEOri2(self):
        fkb= Foward_Kinematics_Broadcaster()
        q_waist = 1.28
        q_shoulder = -0.60
        q_elbow = 1.13
        computed_trans = fkb.compute_fk(q_waist, q_shoulder, q_elbow)
        computed_quaternion = np.array(quaternion_from_matrix(computed_trans)).reshape((4,))
        groud_truth = np.array([-0.155, 0.208, 0.576, 0.775]).reshape((4,))
        np.testing.assert_allclose(computed_quaternion, groud_truth, rtol=0, atol=0.01,
        err_msg="test 2: Wrong position")




if __name__ == "__main__":
    rosunit.unitrun("arm_kinematics", "test_arm_kinematics", TestForwardKinematics)