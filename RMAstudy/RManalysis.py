import csv
from janome.tokenizer import Tokenizer
from collections import Counter, defaultdict


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
def analyze_morphology(text):
    t = Tokenizer()
    tokens = t.tokenize(text)
    return [(token.surface, token.part_of_speech.split(',')[0]) for token in tokens]


# 各セルの内容を形態素解析し、名詞を抽出する
def extract_nouns_and_frequency(csv_data):
    noun_freq = []
    for row in csv_data:
        noun_counter = Counter()
        for cell in row:
            analyzed_tokens = analyze_morphology(cell)
            nouns = [token[0] for token in analyzed_tokens if token[1] == '名詞']
            noun_counter.update(nouns)
        noun_freq.append(noun_counter)
    return noun_freq


# 結果をCSV形式で保存
def save_noun_frequency_to_csv(noun_freq, output_path):
    with open(output_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        for row_counter in noun_freq:
            writer.writerow([f"{word}:{count}" for word, count in row_counter.items()])


# メイン処理
if __name__ == "__main__":
    input_file_path = 'LessonMaterial.csv'
    output_file_path = 'Old Files/NounFrequency-Janome.csv'

    csv_data = read_csv(input_file_path)
    noun_freq = extract_nouns_and_frequency(csv_data)
    save_noun_frequency_to_csv(noun_freq, output_file_path)
    print(f"名詞の頻度分析結果を '{output_file_path}' に保存しました。")
