# Intelligent Driver Model Algorithm
# @author: Violin
# @Date: 2022-04-22
import math
import operator
from gym_lass.algos.algorithm import Algorithm
from gym_lass.utils.utils import Utils


class IDM(Algorithm):

    def __init__(self, desired_velocity, minimum_spacing, desired_time_headway, acceleration, comfortable_deceleration,
                 delta=4):
        self.__v0 = desired_velocity
        self.__s0 = minimum_spacing
        self.__T = desired_time_headway
        self.__a = acceleration
        self.__b = comfortable_deceleration
        self.__delta = delta

    @property
    def longitudinal(self):
        return True

    @property
    def lateral(self):
        return False

    def step(self, ego_state, perception=None):
        if perception is not None and len(perception) > 0:
            gaps = [(i, Utils.gap(ego_state, i)) for i in perception]
            closest_car = min(gaps, key=operator.itemgetter(1))
            throttle = self._get_accelerate(ego_state.speed, ego_state.h, closest_car[0].speed,
                                            closest_car[0].h, closest_car[1])
        else:
            throttle = self._get_accelerate(ego_state.speed, ego_state.h)

        return throttle, 0

    def _get_accelerate(self, self_speed, self_heading, prior_speed=None, prior_heading=None, space=None):
        """Calculate accelerate through IDM algorithm

        Parameters:
            self_speed (float): speed of ego in m/s
            self_heading (float): velocity direction of ego in rad
            prior_speed (float): speed of the vehicle in front of ego in m/s
            prior_heading (float): velocity direction of the vehicle in front of ego in rad
            space (float): the gap between ego and the vehicle in front of ego in m

        Returns:
            float: the accelerate of ego in m/s^2
        """
        if space == 0:
            return 0
        if prior_speed is not None and prior_heading is not None and space is not None:
            relative_speed = self_speed - prior_speed * math.cos(
                prior_heading - self_heading)
            s_star = self.__s0 + self_speed * self.__T + (self_speed * relative_speed) / (
                    2 * math.sqrt(self.__a * self.__b))
            return self.__a * (1 - math.pow(self_speed / self.__v0, self.__delta) - math.pow(s_star / space, 2))
        else:
            return self.__a * (1 - math.pow(self_speed / self.__v0, self.__delta))
