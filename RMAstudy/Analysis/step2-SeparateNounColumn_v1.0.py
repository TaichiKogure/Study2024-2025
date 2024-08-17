import pandas as pd

# 読み込むCSVファイル名のリスト
input_csv_files = ['Routput_with_nouns.csv', 'Soutput_with_nouns.csv','HCoutput_with_nouns.csv','HOoutput_with_nouns.csv']

for input_csv_file in input_csv_files:
    # CSVファイルを読み込む
    df = pd.read_csv(input_csv_file)

    # カラム名をリストとして取得
    columns_to_extract = ['no'] + [col for col in df.columns if col.startswith('Nouns')]

    # データフレームから指定されたカラムを抽出
    df_extracted = df[columns_to_extract]

    # 各カラムを個別にCSVファイルとして保存
    for column in columns_to_extract:
        output_csv_file = f"{input_csv_file[:5]}-{column.replace('Nouns_', '')}.csv"
        df[[column]].to_csv(output_csv_file, index=False)
    print(f"{output_csv_file}に保存されました。")
