import os
import gym
from gym.envs.registration import register

__version__ = "0.1.0"


def envpath():
    resdir = os.path.join(os.path.dirname(__file__))
    return resdir


print("gym-ghumanoid: ")
print("|    gym version and path:", gym.__version__, gym.__path__)

print("|    REGISTERING GHumanoid-v0 from", envpath())
register(
    id="GHumanoid_PB-v0",
    entry_point="gym_ghumanoid_pybullet.envs:GHumanoid",
)

print("|    REGISTERING GHumanoidGoal-v0 from", envpath())
register(
    id="GHumanoidGoal_PB-v0",
    entry_point="gym_ghumanoid_pybullet.envs:GHumanoidGoal",
)

print("|    gym version and path:", gym.__version__, gym.__path__)

print("|    REGISTERING GHumanoid_BF-v0 from", envpath())
register(
    id="GHumanoid_PB_BF-v0",
    entry_point="gym_ghumanoid_pybullet.envs:GHumanoid_BF",
)

print("|    REGISTERING GHumanoidGoal_BF-v0 from", envpath())
register(
    id="GHumanoidGoal_PB_BF-v0",
    entry_point="gym_ghumanoid_pybullet.envs:GHumanoidGoal_BF",
)
# print("|    REGISTERING GFetchDCIL-v0 from", envpath())
# register(
#     id="GFetchDCIL-v0",
#     entry_point="gym_gfetch.envs:GFetchDCIL",
#     reward_threshold=1.,
# )
