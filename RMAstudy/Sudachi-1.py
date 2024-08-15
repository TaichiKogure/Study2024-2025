import csv
from collections import Counter
from sudachipy import tokenizer
from sudachipy import dictionary


# CSV ファイルの読み込み
def read_csv(file_path):
    with open(file_path, newline='', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        data = [row for row in reader]
    return data


# CSV ファイルの書き出し
def write_csv(file_path, data):
    with open(file_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)


# 形態素解析を実行する関数
def analyze_morphology(text, tok):
    mode = tokenizer.Tokenizer.SplitMode.C
    tokens = tok.tokenize(text, mode)
    return [(m.surface(), m.part_of_speech()[0]) for m in tokens]


# 各セルの内容を形態素解析し、名詞を抽出する
def extract_nouns_counter(csv_data, tok):
    global_noun_counter = Counter()
    for row in csv_data:
        for cell in row:
            analyzed_tokens = analyze_morphology(cell, tok)
            nouns = [token[0] for token in analyzed_tokens if token[1] == '名詞']
            global_noun_counter.update(nouns)
    return global_noun_counter


# 結果をCSV形式で保存
def save_noun_ranking_to_csv(noun_counter, output_path):
    with open(output_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Word', 'Frequency'])  # ヘッダー行
        for word, count in noun_counter.most_common():
            writer.writerow([word, count])


# メイン処理
if __name__ == "__main__":
    input_file_path = 'LessonMaterial.csv'
    output_file_path = 'NounRanking-Sudachi.csv'

    # Sudachiの辞書をロード
    tok_obj = dictionary.Dictionary().create()

    # 解析処理
    csv_data = read_csv(input_file_path)
    noun_counter = extract_nouns_counter(csv_data, tok_obj)
    save_noun_ranking_to_csv(noun_counter, output_file_path)
    print(f"名詞の頻出度ランキングを '{output_file_path}' に保存しました。")
