import os

import numpy
from RampMerge_v0 import RampMerge_v0
# from HighwayOvertake_v0 import HighwayOvertake_v0


if __name__ == '__main__':

    # env = HighwayOvertake_v0(display=True)
    env = RampMerge_v0(display=True)
    env.reset()

    t = 10
    while t:
        s, r, e, _ = env.step([0.01, 0])
        # print("s: {}, r: {}, e: {}".format(s, r, e))
        # if e:
        #     break
        print(t)
        t -= 1
