import pandas as pd
import re
import os
from sudachipy import dictionary, Tokenizer


# テキストの正規化関数
def normalize_text(text):
    text = text.lower()  # 小文字に変換
    text = re.sub(r'[０-９]', lambda x: chr(ord(x.group(0)) - 0xFEE0), text)  # 全角数字を半角に変換
    text = re.sub(r'[Ａ-Ｚａ-ｚ]', lambda x: chr(ord(x.group(0)) - 0xFEE0), text)  # 全角英字を半角に変換
    text = re.sub(r'[\u3000]', ' ', text)  # 全角空白を半角に変換
    text = re.sub(r'[^\w\s]', '', text)  # 記号の削除
    return text


# 不要な文字の削除、記号・数字の削除、ストップワードの除去などを含む前処理関数
def preprocess_text(text):
    text = normalize_text(text)
    text = re.sub(r'[0-9]', '', text)  # 数字の削除
    text = re.sub(r'\s+', ' ', text)  # 複数の空白を一つにまとめる

    stopwords = ['ため', 'こと', 'とき', '等']
    pattern = re.compile(r'\b(?:' + '|'.join(stopwords) + r')\b')
    text = pattern.sub('', text)

    return text


# 形態素解析を行い名詞を抽出する関数
def extract_nouns(text):
    # 辞書の初期化 (SudachiDict-fullを使用)
    tokenizer_obj = dictionary.Dictionary(dict_type="full").create()
    mode = Tokenizer.SplitMode.C  # SplitModeを選択 (A, B, Cの中から)
    tokens = tokenizer_obj.tokenize(text, mode)

    nouns = [token.surface() for token in tokens if token.part_of_speech()[0] == "名詞"]
    return nouns


# CSVファイルのパスを設定
csv_file_path = '/Users/koguretaichi/PycharmProjects/RMA/LM_HC.csv'

# CSVファイルの存在チェック
if os.path.exists(csv_file_path):
    # CSVファイルの読み込み
    df = pd.read_csv(csv_file_path)

    # 前処理と形態素解析
    df['ProcessedText'] = df['Text'].apply(preprocess_text)
    df['Nouns'] = df['ProcessedText'].apply(extract_nouns)

    # ファイルに書き出し
    df.to_csv('output_with_nouns.csv', index=False, encoding='utf-8-sig')

    # 結果の表示
    print(df[['Text', 'ProcessedText', 'Nouns']])
else:
    print(f"Error: The file '{csv_file_path}' does not exist. Please provide the correct file path.")

#%%
import pandas as pd
import re
import os
from sudachipy import dictionary, Tokenizer


# テキストの正規化関数
def normalize_text(text):
    if not isinstance(text, str):
        text = str(text)  # 非文字列の値を文字列に変換
    text = text.lower()  # 小文字に変換

    text = re.sub(r'[０-９]', lambda x: chr(ord(x.group(0)) - 0xFEE0), text)  # 全角数字を半角に変換
    text = re.sub(r'[Ａ-Ｚａ-ｚ]', lambda x: chr(ord(x.group(0)) - 0xFEE0), text)  # 全角英字を半角に変換
    text = re.sub(r'[\u3000]', ' ', text)  # 全角空白を半角に変換
    text = re.sub(r'[^\w\s]', '', text)  # 記号の削除
    return text


# 不要な文字の削除、記号・数字の削除、ストップワードの除去などを含む前処理関数
def preprocess_text(text):
    text = normalize_text(text)
    text = re.sub(r'[0-9]', '', text)  # 数字の削除
    text = re.sub(r'\s+', ' ', text)  # 複数の空白を一つにまとめる
    return text


# 形態素解析を行い名詞を抽出する関数
def extract_nouns(text):
    # 辞書の初期化 (SudachiDict-fullを使用)
    tokenizer_obj = dictionary.Dictionary(dict="full").create()
    mode = Tokenizer.SplitMode.C  # SplitModeを選択 (A, B, Cの中から)
    tokens = tokenizer_obj.tokenize(text, mode)

    nouns = [token.surface() for token in tokens if token.part_of_speech()[0] == "名詞"]
    return nouns


# CSVファイルのパスを設定
csv_file_path = '/Users/koguretaichi/PycharmProjects/RMA/LM_HC.csv'

# CSVファイルの存在チェック
if os.path.exists(csv_file_path):
    # CSVファイルの読み込み
    df = pd.read_csv(csv_file_path)

    # デバッグ用: カラムの確認
    print("CSV columns:", df.columns)

    target_columns = ['category', 'tech4need', 'issuesandmechanism', 'teardownissues', 'actionitems_planandtarget',
                      'risks', 'coutermeasure4issues']

    # 全ての対象カラムに対して前処理と名詞抽出を行う
    for column in target_columns:
        if column in df.columns:
            processed_col_name = f'Processed_{column}'
            nouns_col_name = f'Nouns_{column}'

            df[processed_col_name] = df[column].apply(preprocess_text)
            df[nouns_col_name] = df[processed_col_name].apply(extract_nouns)
        else:
            print(f"Error: The column '{column}' does not exist in the CSV file.")

    # 全ての結果をファイルに書き出し
    df.to_csv('output_with_nouns.csv', index=False, encoding='utf-8-sig')

    # 結果の表示
    print(df.head())
else:
    print(f"Error: The file '{csv_file_path}' does not exist. Please provide the correct file path.")

#%%
import pandas as pd
import os
import networkx as nx
import matplotlib.pyplot as plt
from itertools import combinations

# 前処理されたデータの読み込み
csv_file_path = 'output_with_nouns.csv'
df = pd.read_csv(csv_file_path)

# 共起ネットワークの初期化
G = nx.Graph()

# 名詞カラムのリストを取得
nouns_columns = [col for col in df.columns if col.startswith('Nouns_')]

# 各名詞カラムごとに共起関係を追加
for col in nouns_columns:
    for nouns_list in df[col].dropna():
        # 評価名詞リストの空でない行に対する共起ペア
        noun_pairs = combinations(nouns_list.strip('][').replace("'", "").split(', '), 2)
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

#%%
import pandas as pd
import os
from itertools import combinations
from collections import Counter

# 前処理されたデータの読み込み
csv_file_path = 'output_with_nouns.csv'
df = pd.read_csv(csv_file_path)

# 名詞カラムのリストを取得
nouns_columns = [col for col in df.columns if col.startswith('Nouns_')]

# 共起関係の収集
cooccurrences = Counter()
for col in nouns_columns:
    for nouns_list in df[col].dropna():
        # 評価名詞リストの空でない行に対する共起ペア
        noun_pairs = combinations(nouns_list.strip('][').replace("'", "").split(', '), 2)
        cooccurrences.update(noun_pairs)

# 共起関係をデータフレームに変換
cooccurrence_df = pd.DataFrame(cooccurrences.items(), columns=['Pair', 'Frequency'])
cooccurrence_df[['Word1', 'Word2']] = pd.DataFrame(cooccurrence_df['Pair'].tolist(), index=cooccurrence_df.index)
cooccurrence_df.drop(columns=['Pair'], inplace=True)

# 出力ファイルとして保存
output_csv_file_path = 'cooccurrence_network.csv'
cooccurrence_df.to_csv(output_csv_file_path, index=False,encoding='utf-8-sig')

# 表の表示
print(cooccurrence_df)
