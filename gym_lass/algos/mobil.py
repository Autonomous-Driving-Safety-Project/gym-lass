# Lane-Changing Model MOBIL (Symmetric)
# @author: Violin
# @Date: 2022-05-09
import math
from .algorithm import Algorithm
from .lks_pid import LKS_PID
from .idm import IDM
from gym_lass.utils.utils import Utils


class MOBIL(Algorithm):

    def __init__(self, desired_speed, politeness_factor, changing_threshold=0.1, max_safe_deceleration=4, kp=0.2,
                 ki=0, kd=20, minimum_spacing=1.5, desired_time_headway=1.5, acceleration=1.2,
                 comfortable_deceleration=1.6, delta=4):
        # PID
        self.__p = kp
        self.__i = ki
        self.__d = kd
        self.__u_1 = 0
        self.__err_1 = 0
        self.__err_2 = 0
        # IDM
        self.__desired_speed = desired_speed
        self.__s0 = minimum_spacing
        self.__T = desired_time_headway
        self.__a = acceleration
        self.__b = comfortable_deceleration
        self.__delta = delta
        # MOBIL
        self.__p = politeness_factor
        self.__ath = changing_threshold
        self.__bsafe = max_safe_deceleration

    @property
    def longitudinal(self):
        return False

    @property
    def lateral(self):
        return True

    def _idm_acceleration(self, desired_speed, space, ego_speed, prior_speed):
        if space <= 0:
            return float('-inf')
        self_speed = ego_speed
        relative_speed = ego_speed - prior_speed
        s_star = self.__s0 + self_speed * self.__T + (self_speed * relative_speed) / (
                2 * math.sqrt(self.__a * self.__b))
        return self.__a * (1 - math.pow(self_speed / desired_speed, self.__delta) - math.pow(s_star / space, 2))

    def _idm_free_acceleration(self, desired_speed, ego_speed):
        return self.__a * (1 - math.pow(ego_speed / desired_speed, self.__delta))

    @staticmethod
    def _get_space(latter_state, prior_state):
        return max(prior_state.s - latter_state.s - prior_state.length / 2 - latter_state.length / 2, 0)

    def _get_lks(self, err):
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

    def step(self, ego_state, perception, full_state):
        """
        Get the action for the next time step
        :param ego_state: (Lass.ObjectState) current ego state
        :param perception: return tuple of RoadSensor
        :param full_state: all object states
        :return: 0, steer
        """
        ego_lane = ego_state.lane_id
        ego_lane_width = None
        left_lane = None
        left_lane_width = None
        right_lane = None
        right_lane_width = None
        # get left & right lane id
        for index in range(len(perception)):
            if perception[index]["id"] == ego_lane:
                ego_lane_width = perception[index]["width"]
                if index > 0:
                    left_lane = perception[index - 1]["id"]
                    left_lane_width = perception[index - 1]["width"]
                if index < len(perception) - 1:
                    right_lane = perception[index + 1]["id"]
                    right_lane_width = perception[index + 1]["width"]
                break
        # get left & right lane successor and prior states
        left_successor = None
        right_successor = None
        current_successor = None
        left_prior = None
        right_prior = None
        current_prior = None
        for obj in full_state:
            if obj.lane_id == ego_lane and obj.id != ego_state.id and obj.s >= ego_state.s:
                if current_prior is None or obj.s < current_prior.s:
                    current_prior = obj
            if obj.lane_id == ego_lane and obj.id != ego_state.id and obj.s < ego_state.s:
                if current_successor is None or obj.s > current_successor.s:
                    current_successor = obj
            if left_lane is not None:
                if obj.lane_id == left_lane and obj.s < ego_state.s:
                    # TODO: handle s decrease case
                    if left_successor is None or obj.s > left_successor.s:
                        left_successor = obj
                if obj.lane_id == left_lane and obj.s >= ego_state.s:
                    if left_prior is None or obj.s < left_prior.s:
                        left_prior = obj
            if right_lane is not None:
                if obj.lane_id == right_lane and obj.s < ego_state.s:
                    # TODO: handle s decrease case
                    if right_successor is None or obj.s > right_successor.s:
                        right_successor = obj
                if obj.lane_id == right_lane and obj.s >= ego_state.s:
                    if right_prior is None or obj.s < right_prior.s:
                        right_prior = obj

        # get acceleration
        left_successor_acceleration = 0
        right_successor_acceleration = 0
        current_successor_acceleration = 0
        left_successor_acceleration_new = 0
        right_successor_acceleration_new = 0
        current_successor_acceleration_new = 0
        ego_acceleration = 0
        ego_acceleration_left = 0
        ego_acceleration_right = 0
        desired_speed = self.__desired_speed
        if left_successor is not None:
            if left_prior is not None:
                left_successor_acceleration = self._idm_acceleration(desired_speed,
                                                                     self._get_space(left_successor, left_prior),
                                                                     left_successor.speed, left_prior.speed)
            else:
                left_successor_acceleration = self._idm_free_acceleration(desired_speed, left_successor.speed)
            left_successor_acceleration_new = self._idm_acceleration(desired_speed,
                                                                     self._get_space(left_successor, ego_state),
                                                                     left_successor.speed, ego_state.speed)
        if right_successor is not None:
            if right_prior is not None:
                right_successor_acceleration = self._idm_acceleration(desired_speed,
                                                                      self._get_space(right_successor, right_prior),
                                                                      right_successor.speed, right_prior.speed)
            else:
                right_successor_acceleration = self._idm_free_acceleration(desired_speed, right_successor.speed)
            right_successor_acceleration_new = self._idm_acceleration(desired_speed,
                                                                      self._get_space(right_successor, ego_state),
                                                                      right_successor.speed, ego_state.speed)
        if current_successor is not None:
            current_successor_acceleration = self._idm_acceleration(desired_speed,
                                                                    self._get_space(current_successor, ego_state),
                                                                    current_successor.speed, ego_state.speed)
            if current_prior is not None:
                current_successor_acceleration_new = self._idm_acceleration(desired_speed,
                                                                            self._get_space(current_successor,
                                                                                            current_prior),
                                                                            current_successor.speed,
                                                                            current_prior.speed)
            else:
                current_successor_acceleration_new = self._idm_free_acceleration(desired_speed, current_successor.speed)

        if current_prior is not None:
            ego_acceleration = self._idm_acceleration(desired_speed, self._get_space(ego_state, current_prior),
                                                      ego_state.speed, current_prior.speed)
        else:
            ego_acceleration = self._idm_free_acceleration(desired_speed, ego_state.speed)

        if left_prior is not None:
            ego_acceleration_left = self._idm_acceleration(desired_speed, self._get_space(ego_state, left_prior),
                                                           ego_state.speed, left_prior.speed)
        else:
            ego_acceleration_left = self._idm_free_acceleration(desired_speed, ego_state.speed)

        if right_prior is not None:
            ego_acceleration_right = self._idm_acceleration(desired_speed, self._get_space(ego_state, right_prior),
                                                            ego_state.speed, right_prior.speed)
        else:
            ego_acceleration_right = self._idm_free_acceleration(desired_speed, ego_state.speed)

        # whether ego can merge left:
        if left_lane is not None and left_successor_acceleration_new >= -self.__bsafe and (
                ego_acceleration_left - ego_acceleration) + self.__p * (
                left_successor_acceleration_new - left_successor_acceleration + current_successor_acceleration_new - current_successor_acceleration) > self.__ath:
            # merge left
            # print("merge left")
            return self._get_lks(- ego_state.lane_offset + left_lane_width / 2 + ego_lane_width / 2)
        # whether ego can merge right:
        if right_lane is not None and right_successor_acceleration_new >= -self.__bsafe and (
                ego_acceleration_right - ego_acceleration) + self.__p * (
                right_successor_acceleration_new - right_successor_acceleration + current_successor_acceleration_new - current_successor_acceleration) > self.__ath:
            # merge right
            # print("merge right")
            return self._get_lks(- ego_state.lane_offset - right_lane_width / 2 - ego_lane_width / 2)
        # keep lane
        return self._get_lks(- ego_state.lane_offset)
