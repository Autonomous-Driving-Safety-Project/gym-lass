from gym.envs.registration import register

# My envs
# ----------------------------------------

register(
    id='HighwayOvertake-v0',
    entry_point='gym_lass.envs:HighwayOvertake_v0'
)

register(
    id='RampMerge-v0',
    entry_point='gym_lass.envs:RampMerge_v0'
)

register(
    id='RampMerge-v1',
    entry_point='gym_lass.envs:RampMerge_v1'
)
