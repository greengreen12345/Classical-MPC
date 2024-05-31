from configuration import Configuration
from optimizer import CasadiOptimizer
import casadi as ca
import numpy as np
import gym
import mujoco_maze

# define the MuJoCo environment
env = gym.make('PointNMaze-v0')

observation = env.reset()
x0 = observation['observation'][:3].T
config = Configuration()

# set the goal
goal_states = [
    np.array([8., 8., 0.]),
    np.array([0., 16., np.pi/2]),
    np.array([8., 16., 0])
]
goal_state = goal_states[2]

num_steps = 1000
for i in range(num_steps):

    if np.all(goal_state == goal_states[2]) and np.linalg.norm(x0[:2] - goal_states[0][:2]) < 1.5:
        goal_state = goal_states[1]
    if np.all(goal_state == goal_states[1]) and np.linalg.norm(x0[:2] - goal_states[1][:2]) < 1.5:
        goal_state = goal_states[2]

    # Formulate MPC problem
    ca_optimizer = CasadiOptimizer(configuration=config, init_values=x0, predict_horizon=3, goal_state=goal_state)

    # Solve MPC problem
    optimal_U_opti, x17, xm = ca_optimizer.optimize()

    # Apply control input to the environment
    next_state, rewards, _, _ = env.step(optimal_U_opti[:, 0])

    # Update initial state for the next iteration
    x0 = next_state['observation'][:3].T

    env.render()

    


    

