import rospy
from arm_particle_filter.init_env import Env

if __name__ == "__main__":
    env = Env()
    try:
        env.publish_cube()
    except rospy.ROSInterruptException:
        pass
