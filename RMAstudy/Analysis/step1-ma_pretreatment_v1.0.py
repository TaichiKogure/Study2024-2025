#

import pandas as pd
import re
import os
from sudachipy import dictionary, Tokenizer


# テキストの正規化関数
def normalize_text(text):
    if not isinstance(text, str):
        text = str(text)
    text = text.lower()  # 小文字に変換
    text = re.sub(r'[０-９]', lambda x: chr(ord(x.group(0)) - 0xFEE0), text)  # 全角数字を半角に変換
    text = re.sub(r'[Ａ-Ｚａ-ｚ]', lambda x: chr(ord(x.group(0)) - 0xFEE0), text)  # 全角英字を半角に変換
    text = re.sub(r'\u3000', ' ', text)  # 全角空白を半角に変換
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
csv_file_path = '/LM_S.csv'

# CSVファイルの存在チェック
if os.path.exists(csv_file_path):
    # CSVファイルの読み込み
    df = pd.read_csv(csv_file_path)

    # 前処理を行う列のリスト
    text_columns = [
        'tech4need',
        'issuesandmechanism',
        'teardownissues',
        'actionitems_planandtarget',
        'risks',
        'coutermeasure4issues'
    ]

    # 各列に対して前処理と形態素解析を実行
    for col in text_columns:
        if col in df.columns:
            processed_col = f'Processed_{col}'
            nouns_col = f'Nouns_{col}'

            df[processed_col] = df[col].apply(preprocess_text)
            df[nouns_col] = df[processed_col].apply(extract_nouns)
        else:
            print(f"Warning: The CSV file does not contain a '{col}' column.")

    # ファイルに書き出し
    df.to_csv('Soutput_with_nouns.csv', index=False, encoding='utf-8-sig')

    # 結果の表示（表示する列を必要に応じて追加または削除してください）
    print(df.head())
else:
    print(f"Error: The file '{csv_file_path}' does not exist. Please provide the correct file path.")