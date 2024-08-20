import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from sklearn.feature_extraction.text import TfidfVectorizer
from itertools import combinations

# フォルダ内の全てのCSVファイルを取得
folder_path = '/Users/koguretaichi/PycharmProjects/RMA/StepByStep/outputdata'
csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]


def preprocess_documents(df):
    """ DataFrameを単語リストのリストに変換 """
    documents = []
    for col in df.columns:
        if col.startswith('Nouns_') and df[col].dropna().any():
            documents.extend(df[col].dropna().apply(lambda x: eval(x)))
    return documents


def jaccard_similarity(doc1, doc2):
    """ 2つの文書のJaccard Similarityを計算 """
    set1, set2 = set(doc1), set(doc2)
    intersection = len(set1.intersection(set2))
    union = len(set1.union(set2))
    return intersection / union if union != 0 else 0.0


# 全CSVファイルの単語リストを取得
all_documents = []
for csv_file in csv_files:
    csv_file_path = os.path.join(folder_path, csv_file)
    df = pd.read_csv(csv_file_path)
    documents = preprocess_documents(df)
    all_documents.append((csv_file, documents))

# Jaccard Similarityの計算
similarity_matrix = pd.DataFrame(index=csv_files, columns=csv_files)
high_similarity_pairs = []

for (file1, docs1), (file2, docs2) in combinations(all_documents, 2):
    similarities = []
    for doc1 in docs1:
        doc_similarities = [jaccard_similarity(doc1, doc2) for doc2 in docs2 if doc2]
        if doc_similarities:  # Ensure doc_similarities is not empty
            similarities.append(max(doc_similarities))
    average_similarity = sum(similarities) / len(similarities) if similarities else 0.0
    similarity_matrix.loc[file1, file2] = average_similarity
    similarity_matrix.loc[file2, file1] = average_similarity
    if average_similarity >= 0.3:
        high_similarity_pairs.append((file1, file2, average_similarity))

similarity_matrix = similarity_matrix.astype(float)

# 高い類似度のペアをCSVに保存
high_similarity_df = pd.DataFrame(high_similarity_pairs, columns=['File1', 'File2', 'Similarity'])
high_similarity_df.to_csv('HighSimilarityPairs.csv', index=False)

# 数値ラベルを設定
file_number_map = {i: csv_file for i, csv_file in enumerate(csv_files)}
number_labels = list(file_number_map.keys())

# 数値ラベルの対応関係を表示するための情報を生成
footer_info = '\n'.join([f"{num}: {name}" for num, name in file_number_map.items()])

# 図表のサイズ変更 (縦横ともに1.5倍にする)
plt.figure(figsize=(15, 12))  # 元が10, 8 -> 1.5倍で15, 12

# ヒートマップの描画
ax = sns.heatmap(similarity_matrix.values, annot=True, fmt=".2f", cmap="YlGnBu", square=True,
                 vmin=0, vmax=0.4, xticklabels=number_labels, yticklabels=number_labels,
                 annot_kws={"size": 7})  # フォントサイズ調整
ax.set_xticklabels(ax.get_xticklabels(), fontsize=7)  # x軸のラベルフォントを半分に
ax.set_yticklabels(ax.get_yticklabels(), fontsize=7)  # y軸のラベルフォントを半分に
plt.title('Jaccard Similarity Heatmap', fontsize=12)

# ヒートマップの脚注に数値ラベルとファイル名の対応関係を追加
plt.figtext(0.85, 0.5, footer_info, wrap=True, verticalalignment='center', fontsize=7)

plt.show()
