import os
import numpy as np
import gym
from gym import spaces
import gym_lass.lass as Lass
from gym_lass.utils.utils import Utils
from gym_lass.algos import IDM, LKS_PID, MOBIL
from gym_lass.vehicles import CarMOBIL, CarNaive
from abc import abstractmethod

class HighwayFourCars_v2(gym.Env):
    """Highway Scenario, 3 rival cars"""
    metadata = {'render.modes': ['human']}

    def __init__(self, display=False, enable_random=False, record=False):
        super().__init__()
        # Define action and observation space
        self.__lass = None
        self.action_space = spaces.Box(low=np.array([-0.8, -1, -0.8, -1, -0.8, -1]),
                                       high=np.array([0.3, 1, 0.3, 1, 0.3, 1]),
                                       dtype=np.float32)
        self.observation_space = spaces.Box(
            low=np.array([-1000, -1000, -1000, -30, -30, -30, 0, 0, 0, 0, 0, 0, 0, 0, -20]),
            high=np.array(
                [1000, 1000, 1000, 30, 30, 30, 200, 200, 200, 200, 2 * np.pi, 2 * np.pi, 2 * np.pi, 2 * np.pi, 20]),
            dtype=np.float32)
        self.__display = display
        self.__record = record
        self.__random = enable_random
        self.__vdict = None
        self.__ego = None
        self.__leader = None
        self.__follow = None
        self.__target = None
    
    @property
    @abstractmethod
    def _xosc_path(self):
        pass

    @classmethod
    def __abstract_state(cls, ego_state, leader_state, follow_state, target_state):
        return [
            ego_state.s - leader_state.s,
            ego_state.s - follow_state.s,
            ego_state.s - target_state.s,
            ego_state.t - leader_state.t,
            ego_state.t - follow_state.t,
            ego_state.t - target_state.t,
            leader_state.speed,
            follow_state.speed,
            target_state.speed,
            ego_state.speed,
            ego_state.h,
            leader_state.h,
            follow_state.h,
            target_state.h,
            ego_state.t
        ]

    def reset(self):
        if self.__lass:
            del self.__lass
        if self.__display == True:
            display = 1
        elif self.__display == False:
            display = 0
        else:
            display = self.__display
        if self.__record == False:
            record = 0
        else:
            record = 1
        self.__lass = Lass.Lass(Utils.load_xosc(self._xosc_path, enable_random=self.__random), display, record)
        self.__vdict = self.__lass.vehicleDict()
        init_state = self.__lass.observe()

        ego_id = [k for k, v in self.__vdict.items() if v == "Ego"]
        leader_id = [k for k, v in self.__vdict.items() if v == "Car1"]
        follow_id = [k for k, v in self.__vdict.items() if v == "Car2"]
        target_id = [k for k, v in self.__vdict.items() if v == "Car3"]

        # self.__ego = CarIDM(ego_id[0], init_state[ego_id[0]], IDM(10, 1, 1.6, 1, 0.2), LKS_PID(0.2, 0, 20))
        self.__ego = CarMOBIL(ego_id[0], init_state[ego_id[0]], IDM(25, 1, 1.5, 3, 1.6), MOBIL(20, 0.2, 0.5))
        self.__leader = CarNaive(leader_id[0], init_state[leader_id[0]])
        self.__follow = CarNaive(follow_id[0], init_state[follow_id[0]])
        self.__target = CarNaive(target_id[0], init_state[target_id[0]])

        # s = np.array([self.__abstract_state(self.__ego.state), self.__abstract_state(self.__leader.state),
        #              self.__abstract_state(self.__follow.state), self.__abstract_state(self.__target.state)])
        s = np.array(
            self.__abstract_state(self.__ego.state, self.__leader.state, self.__follow.state, self.__target.state))
        return s.ravel()

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        perception = self.__lass.perceive()
        a_ego = self.__ego.get_action(perception[self.__ego.id])
        a_bumper = action.tolist()
        a_leader = a_bumper[0:2]
        a_follow = a_bumper[2:4]
        a_target = a_bumper[4:6]

        # reverse not allowed
        a_leader[0] = 0 if self.__leader.state.speed <= 0 and a_leader[0] < 0 else a_leader[0]
        a_follow[0] = 0 if self.__follow.state.speed <= 0 and a_follow[0] < 0 else a_follow[0]
        a_target[0] = 0 if self.__target.state.speed <= 0 and a_target[0] < 0 else a_target[0]

        s, collision, _, info = self.__lass.step(
            {self.__ego.id: a_ego, self.__leader.id: a_leader, self.__follow.id: a_follow,
             self.__target.id: a_target})
        self.__ego.update_state(s[self.__ego.id])
        self.__leader.update_state(s[self.__leader.id])
        self.__follow.update_state(s[self.__follow.id])
        self.__target.update_state(s[self.__target.id])
        t = info['t']
        success = False

        road_sensor_1 = [v_1 for v_1 in perception[self.__follow.id] if v_1[0] == 'RoadSensor']
        road_sensor_2 = [v_2 for v_2 in perception[self.__target.id] if v_2[0] == 'RoadSensor']
        road_sensor_3 = [v_3 for v_3 in perception[self.__leader.id] if v_3[0] == 'RoadSensor']

        # TODO: reward calculation and collision handle
        ended = False
        reward = 0
        # crash
        if len(collision[self.__ego.id]) != 0:
            ended = True
            success = True
            collider_id = collision[self.__ego.id][0]
            if collider_id == self.__leader.id:
                collider = self.__leader
            elif collider_id == self.__follow.id:
                collider = self.__follow
            else:
                collider = self.__target
            if collider.state.s > self.__ego.state.s and collider.state.speed >= 0:
                # collider in front
                reward += 50
        elif len(collision[self.__leader.id]) != 0 or len(collision[self.__follow.id]) != 0 or len(
                collision[self.__target.id]) != 0:
            ended = True
            reward -= 500
        # out of road
        elif road_sensor_1[0][1] == 0:
            ended = True
            reward -= 500
        elif road_sensor_2[0][1] == 0:
            ended = True
            reward -= 500
        elif road_sensor_3[0][1] == 0:
            ended = True
            reward -= 500
        elif t >= 10:
            ended = True
            reward -= 100
        else:
            reward -= self.__ego.state.speed * 0.1
            # reward shaping
            reward -= 0.001 * min(abs(self.__ego.state.s - self.__leader.state.s),
                                  abs(self.__ego.state.s - self.__follow.state.s),
                                  abs(self.__ego.state.s - self.__target.state.s)
                                  )

        # x_gap = self.__ego.state.s - self.__bumper.state.s
        # if x_gap > 0:
        #    reward -= x_gap / 5000

        # s = np.array([self.__abstract_state(self.__ego.state), self.__abstract_state(self.__leader.state),
        #               self.__abstract_state(self.__follow.state), self.__abstract_state(self.__target.state)])

        s = np.array(
            self.__abstract_state(self.__ego.state, self.__leader.state, self.__follow.state, self.__target.state))

        return s.ravel(), reward, ended, {'t': t, 'success': success}

    def render(self, mode='human', close=False):
        # Render the environment to the screen
        # assert self.__display
        # return
        pass

class HighwayFourCars_env1_v2(HighwayFourCars_v2):
    def __init__(self, display=False, enable_random=False, record=False):
        super().__init__(display, enable_random, record)
    
    @property
    def _xosc_path(self):
        return os.path.join(Utils.ROOT_PATH, 'resources/xosc/highway_four_cars_env1.xosc')

class HighwayFourCars_env2_v2(HighwayFourCars_v2):
    def __init__(self, display=False, enable_random=False, record=False):
        super().__init__(display, enable_random, record)
    
    @property
    def _xosc_path(self):
        return os.path.join(Utils.ROOT_PATH, 'resources/xosc/highway_four_cars_env2.xosc')

class HighwayFourCars_env3_v2(HighwayFourCars_v2):
    def __init__(self, display=False, enable_random=False, record=False):
        super().__init__(display, enable_random, record)
    
    @property
    def _xosc_path(self):
        return os.path.join(Utils.ROOT_PATH, 'resources/xosc/highway_four_cars_env3.xosc')
