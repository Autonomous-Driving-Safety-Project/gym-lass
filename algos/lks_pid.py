# Lane Keeping System based on PID
# @author: Violin
# @Date: 2022-04-22
import math
import operator
from algos.algorithm import Algorithm
from utils.utils import Utils


class LKS_PID(Algorithm):

    def __init__(self, Kp, Ki = 0, Kd = 0):
        self.__p = Kp
        self.__i = Ki
        self.__d = Kd
        self.__u_1 = 0
        self.__err_1 = 0
        self.__err_2 = 0

    @property
    def longitudinal(self):
        return False

    @property
    def lateral(self):
        return True

    def step(self, ego_state, perception=None):
        err = - ego_state.lane_offset
        delta_u = self.__p * (err - self.__err_1) + self.__i * err + self.__d * (err - 2 * self.__err_1 + self.__err_2)
        self.__u_1 += delta_u
        self.__err_2 = self.__err_1
        self.__err_1 = err
        if self.__u_1 > 1:
            u = 1
        elif self.__u_1 < -1:
            u = -1
        else:
            u = self.__u_1
        return 0, u

