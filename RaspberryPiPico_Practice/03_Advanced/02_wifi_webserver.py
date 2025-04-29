"""
Raspberry Pi Pico - Advanced Program 2: WiFi Web Server
======================================================

This program demonstrates how to create a web server on the Pico W
that allows remote monitoring of sensor data and control of outputs.

Note: This program requires a Raspberry Pi Pico W (with WiFi capability).
It will not work on a standard Pico without WiFi.

Hardware required:
- Raspberry Pi Pico W
- DHT22 or DHT11 temperature and humidity sensor
- LED
- Servo motor (optional)
- 10K ohm resistor (for DHT sensor)
- 220 ohm resistor (for LED)
- Breadboard and jumper wires

Connections:
- Connect DHT sensor DATA to GPIO pin 15
- Connect DHT sensor VCC to 3.3V
- Connect DHT sensor GND to GND
- Connect a 10K resistor between DATA and VCC (pull-up)
- Connect LED anode to GPIO pin 16 through a 220 ohm resistor
- Connect LED cathode to GND
- Optional: Connect servo signal wire to GPIO pin 17
- Optional: Connect servo power to 5V (or external power supply)
- Optional: Connect servo ground to GND

Note: You'll need to install the following libraries:
- dht.py for the DHT sensor
- network.py should be included with MicroPython for Pico W

Instructions:
1. Set up the circuit as described above
2. Update the WiFi credentials in the code with your network details
3. Upload this program to the Pico W
4. The Pico will connect to WiFi and display its IP address
5. Open a web browser and navigate to the IP address
6. Use the web interface to monitor sensor data and control the LED/servo

Concepts covered:
- WiFi connectivity
- Web server implementation
- HTML and CSS for web interface
- AJAX for dynamic updates
- Remote sensor monitoring
- Remote device control
- JSON data formatting
"""

import machine
import network
import socket
import time
import json
import gc

# Import required libraries
try:
    import dht
except ImportError:
    print("Error: DHT library not found")
    print("Please upload dht.py to your Pico")
    raise

# WiFi credentials - REPLACE WITH YOUR OWN
WIFI_SSID = "YourWiFiNetwork"
WIFI_PASSWORD = "YourWiFiPassword"

# Set up the DHT sensor
dht_pin = machine.Pin(15)
dht_sensor = dht.DHT22(dht_pin)  # Change to DHT11 if using that sensor

# Set up the LED
led = machine.Pin(16, machine.Pin.OUT)

# Set up the servo (optional)
try:
    servo_pin = machine.Pin(17)
    servo_pwm = machine.PWM(servo_pin)
    servo_pwm.freq(50)  # Standard servo frequency is 50Hz
    servo_enabled = True
except:
    servo_enabled = False
    print("Servo setup failed, continuing without servo support")

# Function to set servo position (0-180 degrees)
def set_servo_position(degrees):
    if not servo_enabled:
        return
    
    # Convert degrees to duty cycle (typically 1ms to 2ms pulse)
    min_duty = 1000  # 1ms pulse (0 degrees)
    max_duty = 9000  # 2ms pulse (180 degrees)
    duty = min_duty + (degrees / 180) * (max_duty - min_duty)
    servo_pwm.duty_u16(int(duty))

# Function to read sensor data
def read_sensor():
    try:
        dht_sensor.measure()
        temperature = dht_sensor.temperature()
        humidity = dht_sensor.humidity()
        return temperature, humidity
    except Exception as e:
        print(f"Error reading sensor: {e}")
        return None, None

# Function to connect to WiFi
def connect_to_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    print(f"Connecting to WiFi network: {WIFI_SSID}")
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    
    # Wait for connection with timeout
    max_wait = 10
    while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        max_wait -= 1
        print("Waiting for connection...")
        time.sleep(1)
    
    # Check connection status
    if wlan.status() != 3:
        print("WiFi connection failed")
        return None
    
    ip_address = wlan.ifconfig()[0]
    print(f"Connected to WiFi. IP address: {ip_address}")
    return ip_address

# HTML template for the web page
def get_html():
    html = """<!DOCTYPE html>
<html>
<head>
    <title>Raspberry Pi Pico W Web Server</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #0066cc;
            text-align: center;
        }
        .card {
            background-color: #f9f9f9;
            border-radius: 5px;
            padding: 15px;
            margin-bottom: 15px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .sensor-value {
            font-size: 24px;
            font-weight: bold;
            color: #333;
        }
        .controls {
            display: flex;
            flex-wrap: wrap;
            gap: 15px;
        }
        button {
            background-color: #0066cc;
            color: white;
            border: none;
            padding: 10px 15px;
            border-radius: 5px;
            cursor: pointer;
        }
        button:hover {
            background-color: #0055aa;
        }
        .slider-container {
            width: 100%;
            margin-top: 15px;
        }
        .slider {
            width: 100%;
        }
        .status {
            margin-top: 5px;
            font-style: italic;
            color: #666;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Raspberry Pi Pico W Web Server</h1>
        
        <div class="card">
            <h2>Sensor Readings</h2>
            <p>Temperature: <span id="temperature" class="sensor-value">--</span> °C</p>
            <p>Humidity: <span id="humidity" class="sensor-value">--</span> %</p>
            <p class="status">Last updated: <span id="last-update">--</span></p>
            <button onclick="updateSensorData()">Refresh Data</button>
        </div>
        
        <div class="card">
            <h2>LED Control</h2>
            <div class="controls">
                <button onclick="controlLED(1)">Turn On</button>
                <button onclick="controlLED(0)">Turn Off</button>
            </div>
            <p class="status">LED Status: <span id="led-status">--</span></p>
        </div>
        
        <div class="card" id="servo-card" style="display: %s;">
            <h2>Servo Control</h2>
            <div class="slider-container">
                <input type="range" min="0" max="180" value="90" class="slider" id="servo-slider" oninput="updateServoPosition(this.value)">
                <p>Position: <span id="servo-position">90</span>°</p>
            </div>
            <p class="status">Servo Status: <span id="servo-status">--</span></p>
        </div>
    </div>

    <script>
        // Function to update sensor data
        function updateSensorData() {
            fetch('/api/sensor')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('temperature').textContent = data.temperature.toFixed(1);
                    document.getElementById('humidity').textContent = data.humidity.toFixed(1);
                    document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
                })
                .catch(error => console.error('Error fetching sensor data:', error));
        }
        
        // Function to control the LED
        function controlLED(state) {
            fetch('/api/led?state=' + state)
                .then(response => response.json())
                .then(data => {
                    document.getElementById('led-status').textContent = data.state ? 'ON' : 'OFF';
                })
                .catch(error => console.error('Error controlling LED:', error));
        }
        
        // Function to control the servo
        function updateServoPosition(position) {
            document.getElementById('servo-position').textContent = position;
            fetch('/api/servo?position=' + position)
                .then(response => response.json())
                .then(data => {
                    document.getElementById('servo-status').textContent = 'Position set to ' + data.position + '°';
                })
                .catch(error => console.error('Error controlling servo:', error));
        }
        
        // Update sensor data on page load
        updateSensorData();
        
        // Check LED status on page load
        fetch('/api/led')
            .then(response => response.json())
            .then(data => {
                document.getElementById('led-status').textContent = data.state ? 'ON' : 'OFF';
            })
            .catch(error => console.error('Error fetching LED status:', error));
            
        // Update sensor data every 10 seconds
        setInterval(updateSensorData, 10000);
    </script>
</body>
</html>
""" % ("block" if servo_enabled else "none")
    return html

# Function to parse request parameters
def parse_request_params(request):
    params = {}
    if '?' in request:
        query_string = request.split('?')[1]
        if ' ' in query_string:
            query_string = query_string.split(' ')[0]
        
        for param_pair in query_string.split('&'):
            if '=' in param_pair:
                key, value = param_pair.split('=')
                params[key] = value
    
    return params

# Function to handle web requests
def handle_request(client_socket, request):
    if request.startswith('GET /api/sensor'):
        # Handle sensor data API request
        temperature, humidity = read_sensor()
        if temperature is None:
            temperature = 0
        if humidity is None:
            humidity = 0
        
        response = {
            'temperature': temperature,
            'humidity': humidity
        }
        
        client_socket.send('HTTP/1.1 200 OK\r\n')
        client_socket.send('Content-Type: application/json\r\n')
        client_socket.send('Access-Control-Allow-Origin: *\r\n')
        client_socket.send('\r\n')
        client_socket.send(json.dumps(response))
        
    elif request.startswith('GET /api/led'):
        # Handle LED control API request
        params = parse_request_params(request)
        
        if 'state' in params:
            state = int(params['state'])
            led.value(state)
        
        response = {
            'state': led.value()
        }
        
        client_socket.send('HTTP/1.1 200 OK\r\n')
        client_socket.send('Content-Type: application/json\r\n')
        client_socket.send('Access-Control-Allow-Origin: *\r\n')
        client_socket.send('\r\n')
        client_socket.send(json.dumps(response))
        
    elif request.startswith('GET /api/servo') and servo_enabled:
        # Handle servo control API request
        params = parse_request_params(request)
        
        if 'position' in params:
            position = int(params['position'])
            position = max(0, min(180, position))  # Clamp to 0-180
            set_servo_position(position)
        else:
            position = 90  # Default position
        
        response = {
            'position': position
        }
        
        client_socket.send('HTTP/1.1 200 OK\r\n')
        client_socket.send('Content-Type: application/json\r\n')
        client_socket.send('Access-Control-Allow-Origin: *\r\n')
        client_socket.send('\r\n')
        client_socket.send(json.dumps(response))
        
    else:
        # Serve the main HTML page
        html = get_html()
        
        client_socket.send('HTTP/1.1 200 OK\r\n')
        client_socket.send('Content-Type: text/html\r\n')
        client_socket.send('\r\n')
        client_socket.send(html)

# Main function
def main():
    # Connect to WiFi
    ip_address = connect_to_wifi()
    if not ip_address:
        print("Could not connect to WiFi. Check credentials and try again.")
        return
    
    # Set up the web server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 80))
    server_socket.listen(5)
    
    print(f"Web server started on http://{ip_address}")
    print("Press Ctrl+C to stop the server")
    
    try:
        while True:
            # Wait for a client to connect
            client_socket, addr = server_socket.accept()
            print(f"Client connected from {addr[0]}")
            
            # Receive the request
            request = client_socket.recv(1024).decode('utf-8')
            
            # Handle the request
            if request:
                handle_request(client_socket, request)
            
            # Close the connection
            client_socket.close()
            
            # Free up memory
            gc.collect()
            
    except KeyboardInterrupt:
        print("\nWeb server stopped by user")
    finally:
        # Clean up
        server_socket.close()
        if servo_enabled:
            servo_pwm.duty_u16(0)  # Turn off the servo
        led.value(0)  # Turn off the LED
        print("Web server shutdown complete")

# Run the main function
if __name__ == "__main__":
    main()