# -*- coding: utf-8 -*-
# __author__ = 'siyuan'
import random

WORLD_SIZE = 5
discount = 0.9
# left, up, right, down
actions = ['L', 'U', 'R', 'D']


def construct_MDP(A_POS, A_TO_POS, A_REWARD, B_POS, B_TO_POS, B_REWARD):
    nextState = []
    actionReward = []
    for i in range(0, WORLD_SIZE):
        nextState.append([])
        actionReward.append([])
        for j in range(0, WORLD_SIZE):
            next = dict()
            reward = dict()
            if i == 0:
                next['U'] = [i, j]
                reward['U'] = -1.0
            else:
                next['U'] = [i - 1, j]
                reward['U'] = 0.0

            if i == WORLD_SIZE - 1:
                next['D'] = [i, j]
                reward['D'] = -1.0
            else:
                next['D'] = [i + 1, j]
                reward['D'] = 0.0

            if j == 0:
                next['L'] = [i, j]
                reward['L'] = -1.0
            else:
                next['L'] = [i, j - 1]
                reward['L'] = 0.0

            if j == WORLD_SIZE - 1:
                next['R'] = [i, j]
                reward['R'] = -1.0
            else:
                next['R'] = [i, j + 1]
                reward['R'] = 0.0

            if [i, j] == A_POS:
                next['L'] = next['R'] = next['D'] = next['U'] = A_TO_POS
                reward['L'] = reward['R'] = reward['D'] = reward['U'] = A_REWARD

            if [i, j] == B_POS:
                next['L'] = next['R'] = next['D'] = next['U'] = B_TO_POS
                reward['L'] = reward['R'] = reward['D'] = reward['U'] = B_REWARD

            nextState[i].append(next)
            actionReward[i].append(reward)

    return nextState, actionReward


# value iteration
def value_iteration(nextState, actionReward):
    world = [[0 for _ in range(WORLD_SIZE)] for _ in range(WORLD_SIZE)]
    while True:
        # keep iteration until convergence
        difference = 0
        ## Begin your code
        gamma = 0.9
        for i in range(WORLD_SIZE):
            for j in range(WORLD_SIZE):
                utility_old = world[i][j]
                utilities = []
                for a in actions:
                    x, y = nextState[i][j][a]
                    utilities.append(actionReward[i][j][a] + world[x][y] * gamma)
                world[i][j] = max(utilities)
                new_difference = abs(world[i][j] > utility_old)
                if new_difference > difference:
                    difference = new_difference
        ## End your code

        # keep iteration until convergence
        if difference < 1e-4:
            print('Value Iteration')
            for j in range(WORLD_SIZE):
                print([round(each_v, 1) for each_v in world[j]])
            break


def policy_evaluation(world, policy, nextState, actionReward):
    while True:
        difference = 0
        # Begin your code
        gamma = 0.9
        gamma = 0.9
        for i in range(WORLD_SIZE):
            for j in range(WORLD_SIZE):
                utility_old = world[i][j]
                utilities = []
                a = actions[policy[i][j]]
                x, y = nextState[i][j][a]
                utilities.append(actionReward[i][j][a] + world[x][y] * gamma)
                world[i][j] = max(utilities)
                new_difference = abs(world[i][j] > utility_old)
                if new_difference > difference:
                    difference = new_difference
        ## End your code

        if difference < 1e-4:
            break
    return world


# policy iteration
def policy_iteration(nextState, actionReward):
    # random initialize state value and policy
    world = [[0 for _ in range(WORLD_SIZE)] for _ in range(WORLD_SIZE)]
    policy = [[random.randint(0, len(actions) - 1) for _ in range(WORLD_SIZE)] for _ in range(WORLD_SIZE)]
    while True:
    ## Begin your code
        world = policy_evaluation(world, policy, nextState, actionReward)
        unchanged = True
        for i in range(WORLD_SIZE):
            for j in range(WORLD_SIZE):
                utilities = []
                for a in actions:
                    x, y = nextState[i][j][a]
                    utilities.append(world[x][y])
                a = actions[policy[i][j]]
                x, y = nextState[i][j][a]
                if max(utilities) > world[x][y]:
                    policy[i][j] = utilities.index(max(utilities))
                    unchanged = False
        if unchanged:
            break
    ## End your code

    print('Policy Iteration')
    for j in range(WORLD_SIZE):
        print([round(each_v, 1) for each_v in world[j]])


def process_read(x):
    from_state = [int(x[0][1]), int(x[0][-2])]
    to_state = [int(x[1][1]), int(x[1][-2])]
    reward = float(x[-1])
    return from_state, to_state, reward


while True:
    try:
        A_list = input().strip().split()
        B_list = input().strip().split()
        A_POS, A_TO_POS, A_REWARD = process_read(A_list)
        B_POS, B_TO_POS, B_REWARD = process_read(B_list)
        nextState, actionReward = construct_MDP(A_POS, A_TO_POS, A_REWARD, B_POS, B_TO_POS, B_REWARD)
        value_iteration(nextState, actionReward)
        policy_iteration(nextState, actionReward)
    except EOFError:
        break



