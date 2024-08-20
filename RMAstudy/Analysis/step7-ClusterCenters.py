import os
import pandas as pd
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d

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
    num_clusters = 3
    km = KMeans(n_clusters=num_clusters, random_state=42)
    km.fit(X)

    # 各文書のクラスタラベル
    labels = km.labels_

    # クラスタセンターを取得
    cluster_centers = km.cluster_centers_

    # 結果の表示
    for i in range(num_clusters):
        print(f'Cluster {i}:')
        for j in range(len(documents)):
            if labels[j] == i:
                print(f'   {filenames[j]}')

    # クラスタリング結果の可視化（2D）
    pca = PCA(n_components=2)
    scatter_plot_points = pca.fit_transform(X.toarray())
    pca_centers = pca.transform(cluster_centers)

    colors = plt.cm.tab10(np.linspace(0, 1, num_clusters))
    x_axis = [o[0] for o in scatter_plot_points]
    y_axis = [o[1] for o in scatter_plot_points]
    fig, ax = plt.subplots()

    scatter = ax.scatter(x_axis, y_axis, c=[colors[d] for d in labels])

    # クラスタセンターをプロット
    center_x = [o[0] for o in pca_centers]
    center_y = [o[1] for o in pca_centers]
    ax.scatter(center_x, center_y, c='black', marker='x', s=100)

    # 引用先のcsvファイルの名称を表示
    for i, filename in enumerate(filenames):
        ax.annotate(filename, (x_axis[i], y_axis[i]), fontsize='xx-small')

    plt.title('Clustering Visualization with Cluster Centers and Voronoi Diagram')

    # Voronoi図のプロット
    points = np.vstack([scatter_plot_points, pca_centers])
    vor = Voronoi(points)

    # Voronoi diagram plot
    voronoi_plot_2d(vor, ax=ax, show_vertices=False, line_colors='orange',
                    line_width=2, line_alpha=0.6, point_size=2)

    plt.show()

    # 各クラスターの特徴的な単語を表示
    terms = vectorizer.get_feature_names_out()
    for i in range(num_clusters):
        print(f"Cluster {i} top terms:")
        centroid = cluster_centers[i]
        if isinstance(centroid, np.ndarray):  # PCAの出力がnumpy配列であることを確認
            sorted_indices = centroid.argsort()[:-11:-1]
            top_terms = [terms[index] for index in sorted_indices]
            print(top_terms)
        else:
            print("Cluster center is not a numpy array.")
else:
    print("No valid documents found. Ensure CSV files contain data.")
