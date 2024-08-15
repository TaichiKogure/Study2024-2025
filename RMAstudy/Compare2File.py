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
