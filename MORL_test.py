import argparse
import pandas as pd
import numpy as np
import math
import collections
from ns3gym import ns3env
from dqw_agent import DWL

import torch

K = 4
NET_STATE_SIZE = 32
NET_ACTION_SIZE = 21

parser = argparse.ArgumentParser(description='Start simulation script on/off')
parser.add_argument('--start',
                    type=int,
                    default=1,
                    help='Start ns-3 simulation script 0/1, Default: 1')
parser.add_argument('--iterations',
                    type=int,
                    default=1,
                    help='Number of iterations, Default: 5000')
args = parser.parse_args()
startSim = bool(args.start)
iterationNum = int(args.iterations)


port = 5555
stepTime = 0.5 # seconds
seed = 12
debug = False

env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, debug=debug)
env.reset();

ob_space = env.observation_space # TODO
state_shape = ob_space.shape # TODO
ac_space = env.action_space # TODO
action_num = ac_space.shape # TODO
num_objectives = env.num_of_objectives # TODO
print("Observation space: ", ob_space,  ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)
print("number of objective:", num_objectives)

currIt = 0

# The Q-learning agent parameters
BATCH_SIZE = 32
LR = .001                   # learning rate
EPSILON = .05  # .95               # starting epsilon for greedy policy
EPSILON_MIN = .1           # The minimal epsilon we want
EPSILON_DECAY = .99995      # The minimal epsilon we want
GAMMA = .9                # reward discount
MEMORY_SIZE = 2000        # size of the replay buffer

# The W-learning parameters
WEPSILON = 0.01  #.99
WEPSILON_DECAY = 0.9995
WEPSILON_MIN = 0.01

agent = DWL(state_shape, action_num, num_objectives, dnn_structure=True, epsilon=EPSILON, epsilon_min=EPSILON_MIN,
                epsilon_decay=EPSILON_DECAY, wepsilon=WEPSILON, wepsilon_decay=WEPSILON_DECAY,
                wepsilon_min=WEPSILON_MIN, memory_size=MEMORY_SIZE, learning_rate=LR, gamma=GAMMA)

# Init list for information we need to collect during simulation
num_of_steps = []
# agent.load(PATH)
# With 2 objectives
coll_reward1 = []
coll_reward2 = []
loss_q1_episode = []
loss_q2_episode = []
loss_w1_episode = []
loss_w2_episode = []
pol1_sel_episode = []
pol2_sel_episode = []

try:
    while True:
        print("Start iteration: ", currIt)

        done = False
        rewardsSum1 = 0
        rewardsSum2 = 0
        qSum = 0
        qActions = 1
        lossSum = 0
        obs = env.reset()

        reward = 0
        info = None

        num_steps = 0
        selected_policies = []

        while True:
            nom_action, sel_policy = agent.get_action_nomination(obs)
            num_steps += 1
            nextState, reward, done, info = env.step_all(nom_action)
            selected_policies.append(sel_policy)
            nextState = nextState

            agent.store_transition(obs, nom_action, reward, nextState, done, sel_policy)
            agent.learn()
            rewardsSum1 = np.add(rewardsSum1, atoi(info))
            rewardsSum2 = np.add(rewardsSum2, reward)
            obs = nextState
            if done:
                if currIt + 1 < iterationNum:
                    env.reset()
                break

        q_loss, w_loss = agent.get_loss_values()
        print("Episode", episode, "end_reward", reward, "Sum of the reward:", qSum, "Num steps:", num_steps,
              "Epsilon:", agent.epsilon, "Q loss:", q_loss, "W loss", w_loss)
        count_policies = collections.Counter(selected_policies)
        print("Policies selected in the episode:", count_policies, "Policy 1:", count_policies[0],
              "Policy 2:", count_policies[1], "Policy 3:", count_policies[2])
        count_policies = collections.Counter(selected_policies)
        # q_losses, w_losses = agent.get_loss_values()
        # Save the performance to lists
        num_of_steps.append(num_steps)
        coll_reward1.append(rewardsSum1)
        coll_reward2.append(rewardsSum2)
        pol1_sel_episode.append(count_policies[0])
        pol2_sel_episode.append(count_policies[1])
        loss_q1_episode.append(q_loss[0])
        loss_q2_episode.append(q_loss[1])
        loss_w1_episode.append(w_loss[0])
        loss_w2_episode.append(w_loss[1])

        agent.update_params()

        currIt += 1
        if currIt == iterationNum:
            break

    # Save the results
    df_results = pd.DataFrame()
    df_results['episodes'] = range(1, EPISODES + 1)
    df_results['num_steps'] = num_of_steps
    df_results['col_reward1'] = coll_reward1
    df_results['col_reward2'] = coll_reward2
    df_results['policy1'] = pol1_sel_episode
    df_results['policy2'] = pol2_sel_episode
    df_results['loss_q1'] = loss_q1_episode
    df_results['loss_q2'] = loss_q2_episode
    df_results['loss_w1'] = loss_w1_episode
    df_results['loss_w2'] = loss_w2_episode
    # df_results.to_csv('results/DWL_mountain_car.csv')
    # Save the trained ANN
    # agent.save(PATH)

except KeyboardInterrupt:
    print("Ctrl-C -> Exit")
finally:
    print("Done")






