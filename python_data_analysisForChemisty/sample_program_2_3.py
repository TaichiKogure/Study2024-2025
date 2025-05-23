# -*- coding: utf-8 -*-
"""
@author: hkaneko
"""

import matplotlib.pyplot as plt
import pandas as pd

variable_number_1 = 0  # 散布図における横軸の特徴量の番号 (0 から始まるため注意)
variable_number_2 = 1  # 散布図における縦軸の特徴量の番号

dataset = pd.read_csv('iris_without_species.csv', index_col=0)

# 以下で散布図を描画します
plt.rcParams['font.size'] = 18  # 横軸や縦軸の名前の文字などのフォントのサイズ
plt.scatter(dataset.iloc[:, variable_number_1], dataset.iloc[:, variable_number_2])  # 散布図の作成
plt.xlabel(dataset.columns[variable_number_1])  # 横軸の名前。ここでは、variable_number_1 番目の列の名前
plt.ylabel(dataset.columns[variable_number_2])  # 縦軸の名前。ここでは、variable_number_2 番目の列の名前
plt.show()  # 以上の設定において、グラフを描画
