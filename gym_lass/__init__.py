from gym.envs.registration import register

# My envs
# ----------------------------------------

register(
    id='RampMerge-v0',
    entry_point='gym_lass.envs:RampMerge_v0'
)
