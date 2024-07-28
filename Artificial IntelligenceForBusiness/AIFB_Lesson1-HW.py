import numpy as np

# Setting the parameters gamma and alpha for the Q-learning
gamma = 0.75
alpha = 0.9

# Part 1 Defining the environment
# Defining the status　ロケーション：倉庫の配置はABC、ステートは番号振り分け、０１２

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
# Dfining the actions　全部の配置に移動するアクションは１２こ、この後で各ロケーションごとに取れるアクションを指定する。
actions = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

# Defining the rewards　>＞ 最初の列はAが移動できる方向を示す。AからはBにのみ動けるので二つ目が１他は０

R = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # A
              [1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0],  # B
              [0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],  # C
              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],  # D
              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],  # E
              [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],  # F
              [0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0],  # G
              [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1],  # H
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],  # I
              [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0],  # J
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1],  # K
              [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0]],  # L
             )

#############################
# Marge Part 2 and 3 module #
#############################


# Mkaing a mapping from a states to the location　ステート０１２から配置ABCを引っ張る辞書を作る。
state_to_location = {state: location for location, state in location_to_state.items()}


# Making the final function that will return the optimal route
def route(starting_location, ending_location):
    R_new = np.copy(R)  #COPY original R matrix
    ending_state = location_to_state[ending_location]  #ゴールの数字＝ステートは目指す配置のステートと定義

    R_new[ending_state, ending_state] = 1000

    Q = np.array(np.zeros([12, 12]))  # Initializing the Q-Values
    for i in range(1000):  # Implementing the Q-Learning process　ここでランダムに移動する仕組み＋報酬が少しでもある方＝通路を選ぶ仕組み。
        current_state = np.random.randint(0, 12)  # include 0 to exculde　12 <- !!
        playable_actions = []
        for j in range(12):
            if R_new[current_state, j] > 0:  # Actionのjが現在の状態でプレイした場合の報酬がゼロ以上＝プレイできるアクション
                playable_actions.append(j)  # ↑の時はjを追加する。

        # 12個のプレイ可能なアクションからランダムに選ぶ = a
        next_state = np.random.choice(playable_actions)
        # 時間変化を記載する。　→　TempiralDifference TDt (St,at) = R(St,at) + gamma max(a)(Q(st+1, a))-Q(St,at)
        TD = R_new[current_state, next_state] + gamma * Q[next_state, np.argmax(Q[next_state,])] - Q[
            current_state, next_state]

        # Q-value by applying the Bellman equation:
        Q[current_state, next_state] = Q[current_state, next_state] + alpha * TD

    route = [starting_location]  #ルートの起点はスタートの配置
    next_location = starting_location  #次の配置はまずはスタートの配置から
    while (next_location != ending_location):  #次の配置が最後の配置になるまで続ける
        starting_state = location_to_state[starting_location]  #開始時のロケーション＝文字をステート＝数字で定義
        next_state = np.argmax(Q[starting_state,])  #次のステートは初期ステートのQ最大値[,]はなくてもOK
        next_location = state_to_location[next_state]  #次のロケーションをステートから引用
        route.append(next_location)  #ルートリストに次のロケーションを追加
        starting_location = next_location  #初期ロケーションは次にロケーションに更新
    return route


# middle state

def best_route(starting_location, intermidiatery_location, ending_location):  #中間地点を設定
    return route(starting_location, intermidiatery_location) + route(intermidiatery_location, ending_location)[
                                                               1:]  #＜左一つ目をのぞいて二つ目から数える


print('Route:', best_route('E','D','A'))

#best_route('E', 'D', 'A') <- コンソール上から入力して応答を見る。か、↑のPrintの構文の中にbest_routeを追加して計算する。

