import time

import numpy as np
import rospy
from iiwas_pybullet.srv import SetGains, SetGainsRequest


if __name__ == '__main__':
    namespace = "iiwa_front"
    rospy.init_node("set_gains")
    client = rospy.ServiceProxy(namespace + "/joint_feedforward_trajectory_controller/set_gains", SetGains)
    req = SetGainsRequest()
    req.p_gain.data = [2500., 3000., 800., 1000., 120., 120., 100]
    req.d_gain.data = [3.,    20.,   3.,   10.,    1.0,  0.5, 0.1]
    res = client.call(req)
    time.sleep(2)