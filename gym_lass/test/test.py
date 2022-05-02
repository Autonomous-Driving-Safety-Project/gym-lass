from gym_lass.envs.RampMerge_v0 import RampMerge_v0
from gym_lass.envs.HighwayOvertake_v0 import HighwayOvertake_v0


if __name__ == '__main__':
    env = RampMerge_v0(display=True)
    # env = HighwayOvertake_v0(display=True, enable_random=True)
    env.reset()

    t = 1000
    while t:
        s, r, e, _ = env.step([0.2, 0])
        print("s: {}, r: {}, e: {}".format(s, r, e))
        if e:
            break
        t -= 1
