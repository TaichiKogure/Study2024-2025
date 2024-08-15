import csv
from collections import defaultdict


# CSV ファイルの読み込み
def read_csv(file_path):
    with open(file_path, newline='', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        headers = next(reader)  # ヘッダー行を取得
        data = [row for row in reader]
    return headers, data


# CSV ファイルの書き出し
def write_csv(file_path, headers, data):
    with open(file_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)  # ヘッダー行を書き込む
        writer.writerows(data)


# データをカテゴリごとに分割する関数
def split_data_by_category(headers, data):
    categories = ["信頼性", "安全性", "高出力", "高容量"]
    grouped_data = defaultdict(list)

    for row in data:
        category = row[1]  # 左から二列目の'category'列
        if category in categories:
            grouped_data[category].append(row)

    return grouped_data


# メイン処理
if __name__ == "__main__":
    input_file_path = 'LessonMaterial.csv'

    # CSVの読み込み
    headers, csv_data = read_csv(input_file_path)

    # データの分割
    grouped_data = split_data_by_category(headers, csv_data)

    # カテゴリごとのCSVファイルを書き出し
    for category, data in grouped_data.items():
        if category == "信頼性":
            output_file_path = 'LM_R.csv'
        elif category == "安全性":
            output_file_path = 'LM_S.csv'
        elif category == "高出力":
            output_file_path = 'LM_HO.csv'
        elif category == "高容量":
            output_file_path = 'LM_HC.csv'
        else:
            continue

        write_csv(output_file_path, headers, data)
        print(f"カテゴリ '{category}' のデータを '{output_file_path}' に保存しました。")
