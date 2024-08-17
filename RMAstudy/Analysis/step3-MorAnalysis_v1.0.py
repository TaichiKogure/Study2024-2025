#全部まとめたネットワークずを作図。
import pandas as pd
import os
import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations

# フォルダ内のすべてのcsvファイルを取得
folder_path = '/Users/koguretaichi/PycharmProjects/RMA/StepByStep/outputdata'
csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]

# 共起ネットワークの初期化
G = nx.Graph()

# 各csvファイルに対して解析を実行
for csv_file in csv_files:
    csv_file_path = os.path.join(folder_path, csv_file)
    df = pd.read_csv(csv_file_path)

    # 名詞カラムのリストを取得
    nouns_columns = [col for col in df.columns if 'no' not in col and col.startswith('Nouns_')]

    # 各名詞カラムごとに共起関係を追加
    for col in nouns_columns:
        for nouns_list in df[col].dropna():
            # 名詞リストの空でない行に対する共起ペアを生成
            noun_pairs = combinations(eval(nouns_list), 2)
            for pair in noun_pairs:
                if G.has_edge(*pair):
                    G[pair[0]][pair[1]]['weight'] += 1
                else:
                    G.add_edge(pair[0], pair[1], weight=1)

# 共起ネットワークの可視化
pos = nx.spring_layout(G)
weights = nx.get_edge_attributes(G, 'weight').values()
nx.draw(G, pos, with_labels=True, node_size=500, node_color="skyblue", edge_color=weights, width=1.0,
        edge_cmap=plt.cm.Blues)
plt.title("Co-occurrence Network")
plt.show()
