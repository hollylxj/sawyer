# input desired position and orientation to ee_pos_des and ee_ori_des
#

import redis
import time
from math import pi

robot_name = "sawyer"
JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot::" + robot_name + "::actuators::fgc"
JOINT_ANGLES_KEY            = "cs225a::robot::" + robot_name + "::sensors::q"
JOINT_VELOCITIES_KEY        = "cs225a::robot::" + robot_name + "::sensors::dq"
PY_JOINT_TORQUES_COMMANDED_KEY = "py::robot::" + robot_name + "::actuators::fgc"
PY_JOINT_ANGLES_KEY         = "py::robot::" + robot_name + "::sensors::q"
PY_JOINT_VELOCITIES_KEY     = "py::robot::" + robot_name + "::sensors::dq"
PY_EE_POSITION_KEY          = "py::robot::" + robot_name + "::tasks::ee_pos"
PY_EE_DES_POSITION_KEY      = "py::robot::" + robot_name + "::tasks::ee_pos_des"
PY_EE_DES_ORIENTATION_KEY   = "py::robot::" + robot_name + "::tasks::ee_ori_des"

r=redis.Redis(host='localhost',port=6379,db=0)
while(1):
    q =r.get(JOINT_ANGLES_KEY) # TODO: change simulator
    dq =r.get(JOINT_VELOCITIES_KEY) # TODO: change simulator
    
    tau =r.get(PY_JOINT_TORQUES_COMMANDED_KEY)
    ee_pos_des = "0.5 0.5 0.5"
    #ee_pos_des = "-0.1 -0.1 -0.1"
    #ee_ori_des = "{} {} {}".format(0.0, pi/3, 0.0)
    ee_ori_des = "{} {} {}".format(pi/2, pi/6, -pi/4)
    
    r.set(PY_JOINT_ANGLES_KEY, q)
    r.set(PY_JOINT_VELOCITIES_KEY, dq)
    r.set(JOINT_TORQUES_COMMANDED_KEY, tau) # TODO: change simulator
    r.set(PY_EE_DES_POSITION_KEY, ee_pos_des)
    r.set(PY_EE_DES_ORIENTATION_KEY, ee_ori_des)

    #time.sleep(0.001)


