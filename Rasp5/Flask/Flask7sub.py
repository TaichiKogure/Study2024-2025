from flask import Flask, request, jsonify

app = Flask(__name__)


@app.route('/data', methods=['POST'])s
def handle_data():
    try:
        data = request.get_json()
        if not data:
            return "No data received", 400

        print("Received BME280 Data:", data)
        if 'pressure' in data:
            pressure = str(data['pressure'][:-3])  # 仮にスライス操作が必要な場合
            print(f"Pressure: {pressure}")
        else:
            return jsonify({"error": "pressure key not found"}), 400

        return jsonify({"status": "success", "data": data}), 200
    except Exception as e:
        print(f"Error: {e}")
        return "Internal Server Error", 500


@app.route('/data3', methods=['POST'])
def handle_data3():
    try:
        data = request.get_json()
        if not data:
            return "No data received", 400

        print("Received BME680 Data:", data)
        return jsonify({"status": "success", "data": data}), 200
    except Exception as e:
        print(f"Error: {e}")
        return "Internal Server Error", 500


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8888, debug=True)
