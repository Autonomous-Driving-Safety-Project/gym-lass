import os
import numpy as np
import gym
from gym import spaces
import gym_lass.lass as Lass
from gym_lass.utils.utils import Utils
from gym_lass.algos import IDM, LKS_PID
from gym_lass.vehicles import CarIDM, CarLKS


class RampMerge_v1(gym.Env):
    """on-ramp merge scenario, with only longitudinal control enabled"""
    metadata = {'render.modes': ['human']}

    def __init__(self, display=False, enable_random=False, record=False):
        super(RampMerge_v1, self).__init__()
        # Define action and observation space
        self.__lass = None
        self.action_space = spaces.Box(low=np.array([-1]), high=np.array([1]), dtype=np.float32)
        self.observation_space = spaces.Box(low=np.array([0, -100, 0, 0, 0, -100, 0, 0]),
                                            high=np.array([5000, 100, 2 * np.pi, 200, 5000, 100, 2 * np.pi, 200]),
                                            dtype=np.float32)
        self.__xosc_path = os.path.join(Utils.ROOT_PATH, 'resources/xosc/merge.xosc')
        self.__display = display
        self.__record = record
        self.__random = enable_random
        self.__vdict = None
        self.__ego = None
        self.__bumper = None

    @classmethod
    def __abstract_state(cls, state):
        return [state.x, state.y, state.h, state.speed]

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
        self.__lass = Lass.Lass(Utils.load_xosc(self.__xosc_path, enable_random=self.__random), display, record)
        self.__vdict = self.__lass.vehicleDict()
        init_state = self.__lass.observe()

        ego_id = [k for k, v in self.__vdict.items() if v == "Ego"]
        bumper_id = [k for k, v in self.__vdict.items() if v == "Bumper"]

        self.__ego = CarIDM(ego_id[0], init_state[ego_id[0]], IDM(10, 1, 1.6, 1, 0.2), LKS_PID(0.2, 0, 20))
        self.__bumper = CarLKS(bumper_id[0], init_state[bumper_id[0]], LKS_PID(0.2, 0, 20))

        s = np.array([self.__abstract_state(self.__ego.state), self.__abstract_state(self.__bumper.state)])
        return s.ravel()

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        perception = self.__lass.perceive()
        a_ego = self.__ego.get_action(perception[self.__ego.id])
        a_bumper = self.__bumper.get_action(action[0].tolist())
        s, collision, _, info = self.__lass.step(
            {self.__ego.id: a_ego, self.__bumper.id: a_bumper})
        self.__ego.update_state(s[self.__ego.id])
        self.__bumper.update_state((s[self.__bumper.id]))
        t = info['t']

        # TODO: reward calculation and collision handle
        ended = False
        reward = 0

        # crash
        if len(collision[0]) != 0:
            ended = True
            reward += 10
        
        # out of road
        road_sensor = [v for v in perception[self.__bumper.id] if v[0] == 'RoadSensor']
        if road_sensor[0][1] == 0:
            ended = True
            reward -= 10

        # timeout
        if t >= 10:
            ended = True
            reward -= 10

        # time punishment
        reward -= 0.01
        
        # reward shaping
        # x_gap = self.__ego.state.s - self.__bumper.state.s
        # if x_gap > 0:
        #     reward -= x_gap / 10000

        s = np.array([self.__abstract_state(self.__ego.state), self.__abstract_state(self.__bumper.state)])

        return s.ravel(), reward, ended, {'t': t}

    def render(self, mode='human', close=False):
        # Render the environment to the screen
        assert self.__display
        return
