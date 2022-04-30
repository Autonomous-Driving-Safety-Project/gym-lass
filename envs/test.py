import os

from HighwayOvertake_v0 import HighwayOvertake_v0
from RampMerge_v0 import RampMerge_v0

if __name__ == '__main__':

    print(os.getpid())
    input()

    env = RampMerge_v0(display=True)
    env.reset()

    t = 1000
    while t:
        s, r, e, _ = env.step([1, 0])
        print("s: {}, r: {}, e: {}".format(s, r, e))
        if e:
            break
        t -= 1
