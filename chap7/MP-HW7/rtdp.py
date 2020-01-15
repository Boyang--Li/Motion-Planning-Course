import pickle
from racetracks import *
from graph_node import Node
import matplotlib.pyplot as plt

seed = np.random.seed(1234)
graph = {}

def Init_G_value(node):
    min_dis = 100000000.0
    for pnt in FINISH_LINE
        (target_px, target_py) = (pnt[0], pnt[1])
        dis = ((target_px - node.px)**2 + (target_py - node.py)**2)**0.5
        if dis < min_dis
            min_dis = dis
    return min_dis

for key in graph.keys():
    state = graph[key]
    if state.is_goal:
        state.g_value = 11 - state.py
    else:
        if state.px == 32 or state.px == 33 or state.px == 34:
            state.g_value = 11 - state.py
        else:
            state.g_value = (32 - state.px)+(11-state.py)

rand_start = np.random.randint(low = 0, high = 3, size = 1)[0]
greedy_plan = greedy_policy(idx = rand_start)

def greedy_policy(idx = 0, explore = True):
    start_node = Node(START_LINE[idx][0], START_LINE[idx][1],0,0)
    start_key = start_node.key
    start = graph[start_key]
    trajectory = [state.key]
    while not state.is_goal:
        value_uk = []
        for child_idx in range(len(ACTION_SPACE)):
            child_key_9 = state.next_prob_9[child_idx]
            child_9 = graph[child_key_9]
            value_uk.append(child_9.g_value)
        
        action_idx = np.argmin(value_uk)
        if explore:
            action_idx = explore_action(action_idx)
        child_key = state.next_prob_9[action_idx]
        trajectory.append(child_key)
        state = graph[child_key]

        print('finding feasible path:{},{}'.format(state.px,state.py))
    return trajectory

def explore_action(u_idx, epsilon=0.2):
    if np.random.uniform(0,1) < epsilon:
        return np.random.randint(0, len(ACTION_SPACE))
    else:
        return u_idx

for key in greedy_plan:
    state = graph[key]
    if state.is_goal:
        state.g_value = 0
    else:
        value_uk = []
        for child_idx in range(len(ACTION_SPACE)):
            child_key_9 = state.next_prob_9[child_idx]
            child_9 = graph[child_key_9]
            child_key_1 = state.next_prob_1[child_idx]
            child_1 = graph[child_key_1]
            expected_cost_uk = 0.9 * (1 + child_9.g_value) + 0.1 * (1 + child_1.g_value)
            value_uk.append(expected_cost_uk)
        current_value = min(value_uk)
        bellman_error += np.linalg.norm(state.g_value - current_value)
        state.g_value = min(value_uk)
    # end if
# end for
bellman_error_list.append(bellman_error)
print("{}th iteration: {}".format(itr_num, bellman_error))
# end while