import os
import pandas as pd
from sklearn.feature_extraction.text import TfidfVectorizer

# フォルダ内の全てのcsvファイルを取得
folder_path = '/Users/koguretaichi/PycharmProjects/RMA/StepByStep/outputdata'
csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]

# 各CSVファイルに対してTF-IDFを計算
for csv_file in csv_files:
    csv_file_path = os.path.join(folder_path, csv_file)
    df = pd.read_csv(csv_file_path)

    # `Nouns_`で始まるすべてのカラムの内容を取り出し一つの文書に統合
    documents = []
    for col in df.columns:
        if col.startswith('Nouns_') and df[col].dropna().any():
            documents.extend(df[col].dropna().apply(lambda x: ' '.join(eval(x))))

    # TF-IDFを計算
    vectorizer = TfidfVectorizer()
    tfidf_matrix = vectorizer.fit_transform(documents)
    feature_names = vectorizer.get_feature_names_out()
    dense = tfidf_matrix.todense()
    denselist = dense.tolist()

    # 結果としてTF-IDFスコアをデータフレームに格納
    tfidf_results = pd.DataFrame(denselist, columns=feature_names)

    # ファイル名の変更と保存
    output_file_name = f"TFIDF_{csv_file}"
    output_file_path = os.path.join(folder_path, output_file_name)
    tfidf_results.to_csv(output_file_path, index=False)
    print(f"TF-IDF results saved to {output_file_path}")
