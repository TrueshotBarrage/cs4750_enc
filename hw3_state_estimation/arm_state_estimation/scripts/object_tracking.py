import random
from operator import pos

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from arm_particle_filter.tracker import CubeTracker

matplotlib.use('Agg')

random.seed(42)
np.random.seed(42)
plt.ylim(top=100)

if __name__ == '__main__':
    rospy.init_node('cube_tracking', disable_signals=True)
    detector = CubeTracker()
    rospy.spin()
    cv2.destroyAllWindows()
