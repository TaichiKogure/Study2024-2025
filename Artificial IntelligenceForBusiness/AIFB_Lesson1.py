
import numpy as np

# Setting the parameters gamma and alpha for the Q-learning
gamma = 0.75
alpha = 0.9

# Part 1 Defining the environment
# Defining the status

location_to_state = {'A': 0,
                     'B': 1,
                     'C': 2,
                     'D': 3,
                     'E': 4,
                     'F': 5,
                     'G': 6,
                     'H': 7,
                     'I': 8,
                     'J': 9,
                     'K': 10,
                     'L': 11}
# Dfining the actions
actions = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

# Defining the rewards　>＞ 最初の列はAが移動できる方向を示す。AからはBにのみ動けるので二つ目が１他は０

R = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],# A
             [1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0],# B
             [0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],# C
             [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],# D
             [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],# E
             [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],# F
             [0, 0, 1, 0, 0, 0, 1000, 1, 0, 0, 0, 0],# G
             [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1],# H
             [0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],# I
             [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0],# J
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1],# K
             [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0]],# L
             )

# Part 2 - Building the AI solution with Q-learning
# Initializing the Q-Values
Q = np.array(np.zeros([12,12]))

# Implementing the Q-Learning process
for i in range(1000):
    current_state = np.random.randint(0,12)#include 0 to exculde　12 <- !!
    playable_actions = []
    for j in range(12):
        if R[current_state ,j] > 0:     #Actionのjが現在の状態でプレイした場合の報酬がゼロ以上＝プレイできるアクション
            playable_actions.append(j)  #↑の時はjを追加する。

    #12個のプレイ可能なアクションからランダムに選ぶ = a
    next_state = np.random.choice(playable_actions)

    #時間変化を記載する。　→　TempiralDifference TDt (St,at) = R(St,at) + gamma max(a)(Q(st+1, a))-Q(St,at)
    TD = R[current_state, next_state] + gamma * Q[next_state, np.argmax(Q[next_state,])] - Q[current_state, next_state]

    # Q-value by applying the Bellman equation:
    Q[current_state,next_state] = Q[current_state, next_state] + alpha * TD

# Part 3 - Going into Production

# Mkaing a mapping from a states to the location
state_to_location = {state: location for location, state in location_to_state.items()}

# Making the final function that will return the optimal route
def route(stating_location, ending_location):
    route = [stating_location]
    next_location = stating_location
    while(next_location != ending_location):
        starting_state = location_to_state[stating_location]
        next_state = np.argmax(Q[starting_state])
        next_location = state_to_location[next_state]
        route.append(next_location)
        stating_location = next_location
    return route


print(route('E','G'))
