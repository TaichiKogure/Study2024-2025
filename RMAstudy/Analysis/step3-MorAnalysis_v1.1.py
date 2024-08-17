import pandas as pd
import os
import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations
from collections import Counter
from matplotlib import font_manager, rc

# 使用する日本語フォントファイルのパス（適宜変更してください）
font_path = "/Library/Fonts/Arial Unicode.ttf"  # 例えば Arial Unicode フォント

# フォントプロパティを設定
font_prop = font_manager.FontProperties(fname=font_path)
rc('font', family=font_prop.get_name())

# フォルダ内のすべてのcsvファイルを取得
folder_path = '/Users/koguretaichi/PycharmProjects/RMA/StepByStep/outputdata'
csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]

# 各csvファイルごとに共起ネットワークを作成
for csv_file in csv_files:
    csv_file_path = os.path.join(folder_path, csv_file)
    df = pd.read_csv(csv_file_path)

    # 共起ネットワークの初期化
    G = nx.Graph()

    # 名詞カラムのリストを取得
    nouns_columns = [col for col in df.columns if 'no' not in col and col.startswith('Nouns_')]

    # 各名詞カラムごとに共起関係を追加
    for col in nouns_columns:
        for nouns_list in df[col].dropna():
            noun_list = eval(nouns_list)
            # 名詞の頻度をカウント
            noun_counter = Counter(noun_list)
            # 名詞リストの空でない行に対する共起ペアを生成
            noun_pairs = combinations(noun_list, 2)
            for pair in noun_pairs:
                if G.has_edge(*pair):
                    G[pair[0]][pair[1]]['weight'] += 1
                else:
                    G.add_edge(pair[0], pair[1], weight=1)

    # ノードのサイズを頻度に基づいて設定
    node_sizes = [20 * G.degree(node) for node in G.nodes()]

    # ノードの色を半透明の虹色に設定
    node_colors = plt.cm.rainbow([0.5] * len(G.nodes()))

    # 共起ネットワークの可視化
    pos = nx.spring_layout(G)
    weights = [G[u][v]['weight'] for u, v in G.edges()]
    plt.figure(figsize=(10, 12))
    nx.draw(G, pos, with_labels=True, font_size=14, node_size=node_sizes, node_color=node_colors, edge_color="grey",
            width=[weight / 2 for weight in weights],
            edge_cmap=plt.cm.Blues, font_family=font_prop.get_name())
    plt.title(f"Co-occurrence Network for {csv_file}", fontsize=20)  # タイトルのフォントサイズを大きく
    plt.tick_params(axis='both', which='major', labelsize=10)  # 軸の数値を大きく
    plt.show()
