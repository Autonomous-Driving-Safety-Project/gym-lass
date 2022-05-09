# from gym_lass.envs.RampMerge_v1 import RampMerge_v1
from gym_lass.envs.HighwayOvertake_v0 import HighwayOvertake_v0

if __name__ == '__main__':
    env = HighwayOvertake_v0(display=True)
    env.reset()

    t = 5000
    while t:
        s, r, e, info = env.step([1, 0])
        print("s: {}, r: {}, e: {}, t: {}".format(s, r, e, info["t"]))
        if e:
            break
        t -= 1
