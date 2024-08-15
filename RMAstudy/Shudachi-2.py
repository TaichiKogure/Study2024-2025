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
    with open(file_path, 'w', newline='', encoding='utf-8-sig') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)


# 形態素解析を実行する関数
def analyze_morphology(text, tok, mode=tokenizer.Tokenizer.SplitMode.C):
    tokens = tok.tokenize(text, mode)
    results = []
    for m in tokens:
        results.append({
            "surface": m.surface(),  # 表層系
            "part_of_speech": m.part_of_speech(),  # 品詞
            "dictionary_form": m.dictionary_form(),  # 辞書系
            "reading_form": m.reading_form(),  # 読み
            "normalized_form": m.normalized_form(),  # 正規化系
        })
    return results


# 各セルの内容を形態素解析し、名詞を抽出する
def extract_nouns_counter(csv_data, tok):
    global_noun_counter = Counter()
    for row in csv_data:
        for cell in row:
            analyzed_tokens = analyze_morphology(cell, tok)
            nouns = [token["surface"] for token in analyzed_tokens if token["part_of_speech"][0] == '名詞']
            global_noun_counter.update(nouns)
    return global_noun_counter


# 結果をCSV形式で保存
def save_noun_ranking_to_csv(noun_counter, output_path):
    with open(output_path, 'w', newline='', encoding='utf-8-sig') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Word', 'Frequency'])  # ヘッダー行
        for word, count in noun_counter.most_common():
            writer.writerow([word, count])


# 形態素解析結果をファイルに保存
def save_morphology_results(data, output_path):
    with open(output_path, 'w', newline='', encoding='utf-8-sig') as csvfile:
        writer = csv.DictWriter(csvfile,
                                fieldnames=["text", "surface", "part_of_speech", "dictionary_form", "reading_form",
                                            "normalized_form"])
        writer.writeheader()
        for cell in data:
            writer.writerow(cell)


# メイン処理
if __name__ == "__main__":
    input_file_path = 'LessonMaterial.csv'
    output_file_path = 'Old Files/NounRanking.csv'
    morphology_output_path = 'Old Files/MorphologyResults.csv'

    # Sudachiの辞書をロード
    tok_obj = dictionary.Dictionary().create()

    # 解析処理
    csv_data = read_csv(input_file_path)

    # 名詞の計数
    noun_counter = extract_nouns_counter(csv_data, tok_obj)
    save_noun_ranking_to_csv(noun_counter, output_file_path)
    print(f"名詞の頻出度ランキングを '{output_file_path}' に保存しました。")

    # 形態素解析の結果保存
    morphology_results = [
        {
            "text": cell,
            "surface": token["surface"],
            "part_of_speech": token["part_of_speech"],
            "dictionary_form": token["dictionary_form"],
            "reading_form": token["reading_form"],
            "normalized_form": token["normalized_form"]
        }
        for row in csv_data for cell in row for token in analyze_morphology(cell, tok_obj)
    ]
    save_morphology_results(morphology_results, morphology_output_path)
    print(f"形態素解析の結果を '{morphology_output_path}' に保存しました。")

# %%

import csv
from collections import Counter
from sudachipy import tokenizer
from sudachipy import dictionary


# CSV ファイルの読み込み
def read_csv(file_path):
    with open(file_path, newline='', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        headers = next(reader)  # ヘッダー行を取得
        data = [row for row in reader]
    return headers, data


# CSV ファイルの書き出し
def write_csv(file_path, data):
    with open(file_path, 'w', newline='', encoding='utf-8-sig') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(data)


# 形態素解析を実行する関数
def analyze_morphology(text, tok, mode=tokenizer.Tokenizer.SplitMode.C):
    tokens = tok.tokenize(text, mode)
    results = []
    for m in tokens:
        results.append({
            "surface": m.surface(),  # 表層系
            "part_of_speech": m.part_of_speech(),  # 品詞
            "dictionary_form": m.dictionary_form(),  # 辞書系
            "reading_form": m.reading_form(),  # 読み
            "normalized_form": m.normalized_form(),  # 正規化系
        })
    return results


# 各セルの内容を形態素解析し、名詞を抽出する
def extract_nouns_counter(csv_data, tok):
    global_noun_counter = Counter()
    for row in csv_data:
        for cell in row:
            analyzed_tokens = analyze_morphology(cell, tok)
            nouns = [token["surface"] for token in analyzed_tokens if token["part_of_speech"][0] == '名詞']
            global_noun_counter.update(nouns)
    return global_noun_counter


# 結果をCSV形式で保存
def save_noun_ranking_to_csv(noun_counter, output_path):
    with open(output_path, 'w', newline='', encoding='utf-8-sig') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Word', 'Frequency'])  # ヘッダー行
        for word, count in noun_counter.most_common():
            writer.writerow([word, count])


# 形態素解析結果をファイルに保存
def save_morphology_results(data, output_path):
    with open(output_path, 'w', newline='', encoding='utf-8-sig') as csvfile:
        writer = csv.DictWriter(csvfile,
                                fieldnames=["text", "surface", "part_of_speech", "dictionary_form", "reading_form",
                                            "normalized_form"])
        writer.writeheader()
        for cell in data:
            writer.writerow(cell)


# メイン処理
if __name__ == "__main__":
    input_file_path = 'LessonMaterial.csv'
    output_file_path = 'Old Files/NounRanking.csv'
    morphology_output_path = 'Old Files/MorphologyResults.csv'

    # Sudachiの辞書をロード
    tok_obj = dictionary.Dictionary().create()

    # 解析処理
    headers, csv_data = read_csv(input_file_path)

    # データをカテゴリ別に分割
    data_by_category = {}
    for row in csv_data:
        category = row[1]  # 'category' 列に基づいて分割
        if category not in data_by_category:
            data_by_category[category] = []
        data_by_category[category].append(row)

    # グローバルな名詞カウンター
    overall_noun_counter = Counter()

    for category, data in data_by_category.items():
        # 名詞の計数
        noun_counter = extract_nouns_counter(data, tok_obj)
        overall_noun_counter.update(noun_counter)

        # 形態素解析の結果保存
        morphology_results = [
            {
                "text": cell,
                "surface": token["surface"],
                "part_of_speech": token["part_of_speech"],
                "dictionary_form": token["dictionary_form"],
                "reading_form": token["reading_form"],
                "normalized_form": token["normalized_form"]
            }
            for row in data for cell in row[2:] for token in analyze_morphology(cell, tok_obj)
        ]
        save_morphology_results(morphology_results, f'MorphologyResults_{category}.csv')
        print(f"カテゴリー '{category}' の形態素解析の結果を 'MorphologyResults_{category}.csv' に保存しました。")

    # 名詞頻出度ランキングの保存
    save_noun_ranking_to_csv(overall_noun_counter, output_file_path)
    print(f"全体の名詞の頻出度ランキングを '{output_file_path}' に保存しました。")
