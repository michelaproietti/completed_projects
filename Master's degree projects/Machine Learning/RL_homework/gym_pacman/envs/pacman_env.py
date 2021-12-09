#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import os 
import numpy as np
from typing import Dict, Tuple, Optional, List
import dataclasses
import gym
from gym import error, spaces, utils
from gym.utils import seeding


# In[ ]:


@dataclasses.dataclass
class Transition:
    state:Tuple[int,int]
    action: str
    next_state: Tuple[int, int]
    reward: float
    termination: bool

class PacmanEnv(gym.Env):
    metadata = {'render.modes': ['human']}
        
    _states: np.array
    _rewards: np.array
    _action_semantics: List[str]
    _actions: Dict [str, np.array]
    _init_state: Tuple[int, int]
    _current_state: Tuple[int, int]
    _goal: bool
    _total_reward: float
    _ghosts: Optional[List[Tuple[int, int]]]
    _walls: Optional[List[Tuple[int, int]]]
    _transition_probabilities: np.array
    
    def __init__ (self,
               rows: int,
               cols: int,
               step_cost: float,
               goal: bool,
               walls: Optional[List[Tuple[int, int]]] = None,
               ghosts: Optional[List[Tuple[int, int]]] = None) -> None:
        self._states = np.zeros((rows, cols))
        
        walls = [] if walls is None else walls
        
        for r, c in walls:
            self._states[r, c] = 1
        
        self._rewards = step_cost * np.ones((rows, cols))
        
        for r,c in ghosts:
            self._rewards[r,c] = -100
        
        self._action_semantics = ['up', 'down', 'left', 'right']
        self._actions = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
        self._init_state = (6,5)
        
        # pacman does not eat anything in the initial state
        self._rewards[self._init_state] = 0
        
        self._current_state = self._init_state
        self._goal = goal
        self._total_reward = 0.
        self._ghosts = ghosts
        self._walls = walls
        
        self._transition_probabilities = np.array([0., 1., 0.])
    
    @property
    def actions(self) -> List[str]:
        return self._action_semantics

    @property
    def current_state(self) -> Tuple[int, int]:
        return self._current_state
    
    @property
    def reward(self) -> float:
        r, c = self._current_state
        return self._rewards[r, c]
    
    #compute the total reward after a transition
    @property
    def total_reward(self) -> float:
        return self._total_reward + self.reward

    @property
    def termination(self) -> bool:
        in_goal = self._total_reward == self._goal
        in_ghost = any ([self._current_state == p for p in self._ghosts])
        return in_goal or in_ghost
    
    def render(self, mode='human') -> None:
        grid = np.array(self._states, dtype=str)
        r,c = self._current_state
        grid[r, c] = ' X '
        
        for r, c in self._ghosts:
            if ' X ' not in grid[r, c]:
                grid[r, c] = ' G '

        for r, c in self._walls:
            if ' X ' not in grid[r, c]:
                grid[r, c] = '<=>'
        
        print(grid)
        
    def _transition(self, state: Tuple[int, int], a: np.array) -> Tuple[int, int]:
        n_actions = len(self._actions)
        a = self._actions[a + n_actions if a < 0 else a % n_actions]
        new_r = max(0, min(self._states.shape[0] - 1, state[0] + a[0]))
        new_c = max(0, min(self._states.shape[1] - 1, state[1] + a[1]))
        return (new_r, new_c) if self._states[new_r, new_c] == 0. else state
    
    def step(self, action:str) -> Transition:
        a_idx = self._action_semantics.index(action)
        
        rnd = np.random.rand()
        chosen_action = a_idx + np.random.choice([1, 0, -1], p = self._transition_probabilities)
        prev_state = self._current_state
        self._current_state = self._transition(self._current_state, chosen_action)

        # pacman already ate in the new cell, so the reward it will get
        # if it goes in that cell again is 0
        r,c = prev_state
        self._rewards[r,c] = 0

        # update the total reward
        self._total_reward = self.total_reward

        
        return Transition(state=prev_state,
                         action=action,
                         next_state = self._current_state,
                         reward=self.reward,
                         termination=self.termination)
    
    def reset(self, rows: int, cols: int, step_cost: float, ghosts: Optional[List[Tuple[int, int]]], goal:float) -> None:
        self._current_state = self._init_state
        self._rewards = step_cost * np.ones((rows, cols))
        for r,c in ghosts:
                self._rewards[r,c] = -100

        # pacman does not eat anything in the initial state
        self._rewards[self._init_state] = 0

        self._total_reward = 0.
        self._goal = goal

    @property
    def update_goal(self) -> float:
        return self._goal-100

    #this function is used when the agent hits a ghost but still has a life
    def cool_reset(self, ghosts: Optional[List[Tuple[int, int]]]) -> None:
        self._current_state = self._init_state
        for r,c in ghosts:
                self._rewards[r,c] = -100
        self._goal = self.update_goal
        
    def state_space_size(self) -> Tuple[int, int]:
        return self._states.shape
    
    def action_space_size(self) -> int:
        return len(self._actions)

