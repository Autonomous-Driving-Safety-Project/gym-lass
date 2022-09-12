from gym.envs.registration import register

# My envs
# ----------------------------------------

register(
    id='HighwayOvertake-v0',
    entry_point='gym_lass.envs:HighwayOvertake_v0'
)

register(
    id='HighwayOvertake-v1',
    entry_point='gym_lass.envs:HighwayOvertake_v1'
)

register(
    id='RampMerge-v0',
    entry_point='gym_lass.envs:RampMerge_v0'
)

register(
    id='RampMerge-v1',
    entry_point='gym_lass.envs:RampMerge_v1'
)

register(
    id='HighwayFourCars-v0',
    entry_point='gym_lass.envs:HighwayFourCars_v0'
)

register(
    id='HighwayFourCars-v1',
    entry_point='gym_lass.envs:HighwayFourCars_v1'
)
