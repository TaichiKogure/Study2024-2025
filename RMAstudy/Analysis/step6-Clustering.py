import os
import pandas as pd
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA

# 名詞データのcsvファイルの読み込み
directory = "/Users/koguretaichi/PycharmProjects/RMA/StepByStep/outputdata"
documents = []
filenames = []

for filename in os.listdir(directory):
    if filename.endswith(".csv"):
        file_path = os.path.join(directory, filename)
        df = pd.read_csv(file_path)

        # Assuming there's only one column in the CSV
        first_column = df.columns[0]
        text = ' '.join(df[first_column].tolist())
        documents.append(text)
        filenames.append(filename)

# Continue only if documents are found
if documents:
    # TF-IDFによるベクトル化
    vectorizer = TfidfVectorizer()
    X = vectorizer.fit_transform(documents)

    # KMeansクラスタリング
    num_clusters = 2
    km = KMeans(n_clusters=num_clusters)
    km.fit(X)

    # 各文書のクラスタラベル
    labels = km.labels_

    # 結果の表示
    for i in range(num_clusters):
        print(f'Cluster {i}:')
        for j in range(len(documents)):
            if labels[j] == i:
                print(f'   {filenames[j]}')

    # クラスタリング結果の可視化（2D）
    pca = PCA(n_components=2)
    scatter_plot_points = pca.fit_transform(X.toarray())

    colors = ["r", "b"]
    x_axis = [o[0] for o in scatter_plot_points]
    y_axis = [o[1] for o in scatter_plot_points]
    fig, ax = plt.subplots()
    scatter = ax.scatter(x_axis, y_axis, c=[colors[d] for d in labels])

    # 引用先のcsvファイルの名称を表示
    for i, filename in enumerate(filenames):
        ax.annotate(filename, (x_axis[i], y_axis[i]), fontsize='xx-small')

    plt.show()
else:
    print("No valid documents found. Ensure CSV files contain data.")
