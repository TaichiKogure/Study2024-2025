import os
import pandas as pd
from sklearn.feature_extraction.text import TfidfVectorizer
from itertools import combinations  # 組み合わせの生成に使用

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

# 各CSV間のJaccard Similarityを計算
similarity_results = []
for (file1, docs1), (file2, docs2) in combinations(all_documents, 2):
    similarities = []
    for doc1 in docs1:
        doc_similarities = [jaccard_similarity(doc1, doc2) for doc2 in docs2]
        similarities.append(max(doc_similarities))  # 最大値を取る（平均値や他の集約も考えられる）
    average_similarity = sum(similarities) / len(similarities) if similarities else 0.0
    similarity_results.append({'file1': file1, 'file2': file2, 'similarity': average_similarity})

# 結果をデータフレームに格納し出力
similarity_df = pd.DataFrame(similarity_results)
output_file_path = os.path.join(folder_path, 'Jaccard_similarity_results.csv')
similarity_df.to_csv(output_file_path, index=False)
print(f"Jaccard similarity results saved to {output_file_path}")
