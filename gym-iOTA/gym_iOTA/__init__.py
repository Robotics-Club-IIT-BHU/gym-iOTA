from gym.envs.registration import register

register(
    id='iOTA-v0',
    entry_point='gym_iOTA.envs:IotaEnv',
)
