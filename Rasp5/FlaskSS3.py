@app.route('/data', methods=['POST'])
def handle_data():
    data = request.json
    print(data)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    data_1 = str(data.get('pressure')[:-3])
    data_2 = str(data.get('temperature')[:-1])
    row_data = {"current_time": timestamp, "Pressure": data_1, "Tempereture": data_2}

    with open('PicodataX.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["current_time", "Pressure", "Tempereture"])
        if f.tell() == 0:
            writer.writeheader()
        writer.writerow(row_data)

    return 'OK', 200
