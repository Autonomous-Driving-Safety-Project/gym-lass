import os
import numpy as np
import gym
from gym import spaces
import gym_lass.lass as Lass
from gym_lass.utils.utils import Utils
from gym_lass.algos import IDM, LKS_PID
from gym_lass.vehicles import CarIDM, CarNaive


class HighwayFourCars_v0(gym.Env):
    """Highway Scenario, 3 rival cars, only longitudinal action"""
    metadata = {'render.modes': ['human']}

    def __init__(self, display=False, enable_random=False, record=False):
        super(HighwayFourCars_v0, self).__init__()
        # Define action and observation space
        self.__lass = None
        self.action_space = spaces.Box(low=np.array([-0.8, -0.8, -0.8]), high=np.array([0.3, 0.3, 0.3]),
                                       dtype=np.float32)
        self.observation_space = spaces.Box(low=np.array([-1000, -1000, -1000, 0, 0, 0, 0, 0, -20]),
                                            high=np.array([1000, 1000, 1000, 200, 200, 200, 200, 2 * np.pi, 20]),
                                            dtype=np.float32)
        self.__xosc_path = os.path.join(Utils.ROOT_PATH, 'resources/xosc/highway_four_cars.xosc')
        self.__display = display
        self.__record = record
        self.__random = enable_random
        self.__vdict = None
        self.__ego = None
        self.__leader = None
        self.__follow = None
        self.__target = None

    @classmethod
    def __abstract_state(cls, ego_state, leader_state, follow_state, target_state):
        return [
            ego_state.s - leader_state.s,
            ego_state.s - follow_state.s,
            ego_state.s - target_state.s,
            leader_state.speed,
            follow_state.speed,
            target_state.speed,
            ego_state.speed,
            ego_state.h,
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
        self.__lass = Lass.Lass(Utils.load_xosc(self.__xosc_path, enable_random=self.__random), display, record)
        self.__vdict = self.__lass.vehicleDict()
        init_state = self.__lass.observe()

        ego_id = [k for k, v in self.__vdict.items() if v == "Ego"]
        leader_id = [k for k, v in self.__vdict.items() if v == "Car1"]
        follow_id = [k for k, v in self.__vdict.items() if v == "Car2"]
        target_id = [k for k, v in self.__vdict.items() if v == "Car3"]

        self.__ego = CarIDM(ego_id[0], init_state[ego_id[0]], IDM(10, 1, 1.6, 1, 0.2), LKS_PID(0.2, 0, 20))
        self.__leader = CarNaive(leader_id[0], init_state[leader_id[0]])
        self.__follow = CarNaive(follow_id[0], init_state[follow_id[0]])
        self.__target = CarNaive(target_id[0], init_state[target_id[0]])

        #s = np.array([self.__abstract_state(self.__ego.state), self.__abstract_state(self.__leader.state),
        #              self.__abstract_state(self.__follow.state), self.__abstract_state(self.__target.state)])
        s = np.array(self.__abstract_state(self.__ego.state, self.__leader.state, self.__follow.state, self.__target.state))
        return s.ravel()

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        perception = self.__lass.perceive()
        a_ego = self.__ego.get_action(perception[self.__ego.id])
        a_bumper = action.tolist()
        a_leader = a_bumper[0]
        a_follow = a_bumper[1]
        a_target = a_bumper[2]

        # reverse not allowed
        a_leader = 0 if self.__leader.state.speed <= 0 and a_leader < 0 else a_leader
        a_follow = 0 if self.__follow.state.speed <= 0 and a_follow < 0 else a_follow
        a_target = 0 if self.__target.state.speed <= 0 and a_target < 0 else a_target

        s, collision, _, info = self.__lass.step(
            {self.__ego.id: a_ego, self.__leader.id: [a_leader, 0], self.__follow.id: [a_follow, 0],
             self.__target.id: [a_target, 0]})
        self.__ego.update_state(s[self.__ego.id])
        self.__leader.update_state(s[self.__leader.id])
        self.__follow.update_state(s[self.__follow.id])
        self.__target.update_state(s[self.__target.id])
        t = info['t']

        # TODO: reward calculation and collision handle
        ended = False
        reward = 0
        # crash
        if len(collision[self.__ego.id]) != 0:
            ended = True
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
            # if self.__ego.state.s >= self.__leader.state.s or self.__ego.state.s >= self.__leader.state.s:
            #     reward += 100
            # else:
            #     reward += 50
        # out of road
        # road_sensor_1 = [v_1 for v_1 in perception[self.__follow.id] if v_1[0] == 'RoadSensor']
        # road_sensor_2 = [v_2 for v_2 in perception[self.__target.id] if v_2[0] == 'RoadSensor']
        # road_sensor_3 = [v_3 for v_3 in perception[self.__leader.id] if v_3[0] == 'RoadSensor']
        # if road_sensor_1[0][1] == 0:
        #     ended = True
        #     reward -= 10
        # if road_sensor_2[0][1] == 0:
        #     ended = True
        #     reward -= 10
        # if road_sensor_3[0][1] == 0:
        #     ended = True
        #     reward -= 10
        elif t >= 10:
            ended = True
            reward -= 100
        else:
            reward -= self.__ego.state.speed * 0.1

        # x_gap = self.__ego.state.s - self.__bumper.state.s
        # if x_gap > 0:
        #    reward -= x_gap / 5000

        # s = np.array([self.__abstract_state(self.__ego.state), self.__abstract_state(self.__leader.state),
        #               self.__abstract_state(self.__follow.state), self.__abstract_state(self.__target.state)])

        s = np.array(self.__abstract_state(self.__ego.state, self.__leader.state, self.__follow.state, self.__target.state))

        return s.ravel(), reward, ended, {'t': t}

    def render(self, mode='human', close=False):
        # Render the environment to the screen
        assert self.__display
        return
