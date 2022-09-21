import numpy as np
from gym.envs.mujoco import mujoco_env
from gym import utils
import os

import gym
from gym import spaces
import pybullet
import pybullet_data
# physicsClient = pybullet.connect(pybullet.GUI)
# pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

import pybullet_envs

from pybullet_envs.deep_mimic.gym_env.deep_mimic_env import HumanoidDeepMimicWalkBulletEnv

import mpi4py
import time
import ast
import math


# class HumanoidEnv_PB(HumanoidDeepMimicWalkBulletEnv):
#     def __init__(self, render=False):
#         self.render = render
#
#
#
#     def _get_obs(self):
#         state = self.env.unwrapped._internal_env._humanoid.getState()
#         absolute_chest_position, absolute_chest_orientation = self.env.unwrapped._internal_env._humanoid._pybullet_client.getBasePositionAndOrientation(1)
#         return np.concatenate(
#             [   absolute_chest_position,
#                 absolute_chest_orientation,
#                 state
#             ]
#         )
#
#     def _get_state(self):
#
#         return self._get_obs()
#
#
#     def reset_model(self):
#         return self.reset()
#
#
#     # get simulation state
#     def get_inner_state(self):
#         if self.same_instance:
#             inner_state = self.env.unwrapped._internal_env._humanoid._pybullet_client.saveState()
#         else:
#             self.env.unwrapped._internal_env._humanoid._pybullet_client.saveBullet(self.path_to_sim_state + "/sim_state.bullet")
#             inner_state = None
#         return inner_state
#
#     # set simulation state (more than just the position and velocity) to saved one
#     def set_inner_state(self, saved_state):
#         self.env.unwrapped._internal_env._humanoid._pybullet_client.restoreState(saved_state)
#         return self._get_state()


class HumanoidEnv_PB():
    def __init__(self, render=False):
        self.render = render
        self.env = gym.make("HumanoidDeepMimicWalkBulletEnv-v1", renders=self.render)
        self.same_instance = False

        observation_min = np.array([-500.0, -500.0, -500.0] + [-500.0, -500.0, -500.0, -500.0] + [0.0]+[-100.0]+[-4.0]*105+[-500.0]*90).astype(np.float64)
        observation_max = np.array([500.0, 500.0, 500.0]+ [500.0, 500.0, 500.0, 500.0] +[1.0]+[100.0]+[4.0]*105+[500.0]*90).astype(np.float64)
        state_size = 204
        self.observation_space = spaces.Box(observation_min, observation_min, dtype=np.float64)
        self.single_observation_space =  spaces.Box(observation_min, observation_min, dtype=np.float64)


    def _get_obs(self):
        state = self.env.env.unwrapped._internal_env._humanoid.getState()
        # absolute_chest_position, absolute_chest_orientation = self.env.env.unwrapped._internal_env._humanoid._pybullet_client.getBasePositionAndOrientation(1)
        absolute_chest_state = self.env.env.unwrapped._internal_env._humanoid._pybullet_client.getLinkState(self.env.env.unwrapped._internal_env._humanoid._sim_model, 1, computeForwardKinematics=True, computeLinkVelocity=True)
        return np.concatenate(
            [   absolute_chest_state[0],
                absolute_chest_state[1],
                state
            ]
        ).astype(np.float64)

    def _get_state(self):

        return self._get_obs()

    def step(self, a):
        return self.env.step(a)

    def reset_model(self):
        return self.env.reset()

    def reset(self):
        return self.reset_model()

    # get simulation state
    def get_inner_state(self, to_file=False):
        if to_file: ## TODO
            self.env.env.unwrapped._internal_env._humanoid._pybullet_client.saveBullet(self.path_to_sim_state + "/sim_state.bullet")
            inner_state = None
        else:
            inner_state = self.env.env.unwrapped._internal_env._humanoid._pybullet_client.saveState()

        return inner_state

    # set simulation state (more than just the position and velocity) to saved one
    def set_inner_state(self, saved_state, from_file=False):
        if from_file:
            self.env.env.unwrapped._internal_env._humanoid._pybullet_client.restoreState(fileName=saved_state)
            self.env.env.unwrapped._internal_env._humanoid._pybullet_client.stepSimulation()
        else:
            self.env.env.unwrapped._internal_env._humanoid._pybullet_client.restoreState(saved_state)
            self.env.env.unwrapped._internal_env._humanoid._pybullet_client.stepSimulation()
        return self._get_state()
