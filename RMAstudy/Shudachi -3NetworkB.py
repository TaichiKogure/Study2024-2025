import csv
from collections import Counter, defaultdict
from itertools import combinations
import networkx as nx
import matplotlib.pyplot as plt
from sudachipy import tokenizer
from sudachipy import dictionary


# CSV ファイルの読み込み
def read_csv(file_path):
    with open(file_path, newline='', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        headers = next(reader)  # ヘッダー行を取得
        data = [row for row in reader]
    return headers, data


# CSV ファイルの書き出し
def write_csv(file_path, data):
    with open(file_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)


# 名詞の頻度ランキングをCSVファイルに保存する関数
def save_noun_ranking_to_csv(noun_counter, file_path):
    sorted_nouns = noun_counter.most_common()
    with open(file_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Noun', 'Frequency'])
        for noun, freq in sorted_nouns:
            writer.writerow([noun, freq])


# 形態素解析を実行する関数
def analyze_morphology(text, tok, mode=tokenizer.Tokenizer.SplitMode.C):
    tokens = tok.tokenize(text, mode)
    results = []
    for m in tokens:
        results.append({
            "surface": m.surface(),  # 表層系
            "part_of_speech": m.part_of_speech(),  # 品詞
            "dictionary_form": m.dictionary_form(),  # 辞書系
            "reading_form": m.reading_form(),  # 読み
            "normalized_form": m.normalized_form(),  # 正規化系
        })
    return results


# 各セルの内容を形態素解析し、名詞を抽出する
def extract_nouns_counter(csv_data, tok, category_column=1):
    global_noun_counter = Counter()
    for row in csv_data:
        for cell in row[2:]:  # 2列目の信頼性、安全性などの項目ごと
            analyzed_tokens = analyze_morphology(cell, tok)
            nouns = [token["surface"] for token in analyzed_tokens if token["part_of_speech"][0] == '名詞']
            global_noun_counter.update(nouns)
    return global_noun_counter


# 共起ネットワークを構築する関数
def build_cooccurrence_network(csv_data, tok, category_column=1):
    co_occurrence_dict = defaultdict(Counter)
    for row in csv_data:
        for cell in row[2:]:
            analyzed_tokens = analyze_morphology(cell, tok)
            nouns = [token["surface"] for token in analyzed_tokens if token["part_of_speech"][0] == '名詞']
            for w1, w2 in combinations(nouns, 2):
                co_occurrence_dict[w1][w2] += 1
                co_occurrence_dict[w2][w1] += 1
    return co_occurrence_dict


# ネットワークを可視化する関数
def visualize_network(co_occurrence_dict, top_n=30):
    G = nx.Graph()
    for word, neighbors in co_occurrence_dict.items():
        for neighbor, weight in neighbors.items():
            if weight > 1:  # 共起回数が1回以上のエッジのみを追加
                G.add_edge(word, neighbor, weight=weight)

    pos = nx.spring_layout(G, k=1.5)

    # ノードの色とサイズの設定
    node_color = [G.degree(v) for v in G]
    node_size = [G.degree(v) * 100 for v in G]

    plt.figure(figsize=(15, 10))
    nx.draw_networkx_nodes(G, pos, node_color=node_color, cmap=plt.cm.viridis, node_size=node_size)
    nx.draw_networkx_edges(G, pos, alpha=0.3)
    nx.draw_networkx_labels(G, pos, font_size=8, font_family='sans-serif')

    plt.title("Co-occurrence Network of Top Words", size=15)
    plt.show()


# メイン処理
if __name__ == "__main__":
    file_paths = ['LM_S.csv', 'LM_R.csv', 'LM_HO.csv', 'LM_HC.csv']

    # Sudachiの辞書をロード
    tok_obj = dictionary.Dictionary().create()

    for input_file_path in file_paths:
        output_file_path = f'{input_file_path}_NounRanking.csv'
        morphology_output_path = f'{input_file_path}_MorphologyResults.csv'

        # 解析処理
        headers, csv_data = read_csv(input_file_path)

        # 名詞の計数
        noun_counter = extract_nouns_counter(csv_data, tok_obj)
        save_noun_ranking_to_csv(noun_counter, output_file_path)
        print(f"名詞の頻出度ランキングを '{output_file_path}' に保存しました。")

        # 共起ネットワークの構築
        co_occurrence_dict = build_cooccurrence_network(csv_data, tok_obj)

        # ネットワークの可視化
        visualize_network(co_occurrence_dict)
