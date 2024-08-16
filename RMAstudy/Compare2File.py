import pandas as pd

# CSVファイルの読み込み
file1 = 'LM_R.csv_NounRanking.csv'  # 第一のキーワードリスト
file2 = 'LM_S.csv_NounRanking.csv'  # 第二のキーワードリスト

df1 = pd.read_csv(file1)
df2 = pd.read_csv(file2)

# データフレームに変換 (必要ない場合はこの処理をスキップ)
# df1 = pd.read_csv(file1)
# df2 = pd.read_csv(file2)

# 共通のキーワード、共通でないキーワードの特定
common_nouns = pd.merge(df1, df2, on='Noun', how='inner', suffixes=('_1', '_2'))
unique_to_df1 = df1[~df1['Noun'].isin(common_nouns['Noun'])]
unique_to_df2 = df2[~df2['Noun'].isin(common_nouns['Noun'])]

# 結果の表示
print("共通のキーワード:")
print(common_nouns)

print("\n共通ではないキーワード（第一のキーワードリストにのみ存在）:")
print(unique_to_df1)

print("\n共通ではないキーワード（第二のキーワードリストにのみ存在）:")
print(unique_to_df2)

#%%
import pandas as pd
import itertools
import matplotlib.pyplot as plt

# ファイルパスとファイル名
files = {
    'LM_R': 'LM_R.csv_NounRanking.csv',
    'LM_S': 'LM_S.csv_NounRanking.csv',
    'LM_HO': 'LM_HO.csv_NounRanking.csv',
    'LM_HC': 'LM_HC.csv_NounRanking.csv'
}

# データフレームを格納するリスト
dfs = {}
for name, file in files.items():
    df = pd.read_csv(file)
    df.columns = ['Noun', f'{name}_Frequency']
    dfs[name] = df

# 総当たりで共通・非共通のキーワードを特定
result_common = []
result_unique = []

file_combinations = list(itertools.combinations(dfs.keys(), 2))

for name1, name2 in file_combinations:
    df1 = dfs[name1]
    df2 = dfs[name2]

    common = pd.merge(df1, df2, on='Noun', how='inner')
    unique_df1 = df1[~df1['Noun'].isin(common['Noun'])]
    unique_df2 = df2[~df2['Noun'].isin(common['Noun'])]

    # 共通キーワードを結果に追加
    result_common.append(common)

    # 非共通キーワードを結果に追加
    result_unique.append(pd.concat([unique_df1, unique_df2]))

# 全てのファイルに共通するキーワードを特定
common_all = list(dfs.values())[0]
for df in list(dfs.values())[1:]:
    common_all = pd.merge(common_all, df, on='Noun', how='inner')

# 結果をCSVに出力（エンコーディングを指定）
common_all.to_csv('FileB_common_all.csv', index=False, encoding='utf-8-sig')  # 全てのファイルに共通するキーワード
pd.concat(result_common).to_csv('FileA_common_pairs.csv', index=False, encoding='utf-8-sig')  # ペアごとの共通キーワード
pd.concat(result_unique).to_csv('FileA_unique_pairs.csv', index=False, encoding='utf-8-sig')  # ペアごとの非共通キーワード

# 可視化
fig, ax = plt.subplots()
common_all.plot(kind='bar', x='Noun', ax=ax)
plt.title('Common Keywords across all Files')
plt.xlabel('Nouns')
plt.ylabel('Frequency')
plt.xticks(rotation=45)
plt.tight_layout()
plt.savefig('common_keywords_all_files.png')  # 図を保存
plt.show()
