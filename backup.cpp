
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ElegantOTA.h>
#include <Servo.h>
#include <Wire.h>

// WiFi credentials
const char *ssid = "RobotControl";
const char *password = "12345678";

// WiFi AP Static IP configuration
IPAddress apIP(192, 168, 4, 1);     // AP IP
IPAddress gateway(192, 168, 4, 1);  // Gateway
IPAddress subnet(255, 255, 255, 0); // Subnet mask

// Pin definitions
#define RIGHT_SERVO_PIN D5 // GPIO14
#define LEFT_SERVO_PIN D6  // GPIO12
// Encoder 1 (RIGHT)
#define ENCODER1_SDA D1 // GPIO5
#define ENCODER1_SCL D2 // GPIO4
// Encoder 2 (LEFT)
#define ENCODER2_SDA D7 // GPIO13
#define ENCODER2_SCL D0 // GPIO16

// AS5600 I2C addresses
#define AS5600_ADDR 0x36 // Both encoders use same address (on different buses)

// AS5600 Register addresses
#define RAW_ANGLE_REG 0x0C
#define ANGLE_REG 0x0E

// Servo objects
Servo rightServo;
Servo leftServo;

// Web server
ESP8266WebServer server(80);

// Robot state
volatile bool isMoving = false;
volatile int robotState = 0; // 0=idle, 1=forward, 2=backward, 3=turnRight, 4=turnLeft, 5=back180, 6=back360

// Servo positions (Microseconds)
int stopL = 1470;           // Left servo stop
int stopR = 1470;           // Right servo stop
#define SERVO_FORWARD 2000  // Full speed forward (~500us from stop)
#define SERVO_BACKWARD 1000 // Full speed backward (~500us from stop)

// ============= ENCODER FUNCTIONS (AS5600 I2C) =============
// Current I2C bus pins
int currentSDA = -1;
int currentSCL = -1;

void switchI2CBus(int sda, int scl)
{
    if (currentSDA != sda || currentSCL != scl)
    {
        Wire.begin(sda, scl);
        Wire.setClock(100000);
        currentSDA = sda;
        currentSCL = scl;
        delay(10);
    }
}

uint16_t readRawAngle(int sda, int scl, uint8_t addr)
{
    switchI2CBus(sda, scl);

    Wire.beginTransmission(addr);
    Wire.write(RAW_ANGLE_REG);
    uint8_t error = Wire.endTransmission();

    if (error != 0)
    {
        Serial.print("I2C Error on address 0x");
        Serial.print(addr, HEX);
        Serial.print(": ");
        Serial.println(error);
        return 0;
    }

    Wire.requestFrom(addr, 2);
    uint16_t rawAngle = 0;
    if (Wire.available() >= 2)
    {
        rawAngle = (Wire.read() << 8) | Wire.read();
        rawAngle = rawAngle >> 6; // Only 10 bits are used
    }
    else
    {
        Serial.print("I2C No data from address 0x");
        Serial.println(addr, HEX);
    }
    return rawAngle;
}

float rawAngleToDegrees(uint16_t rawAngle)
{
    return (rawAngle / 1023.0) * 360.0;
}

float getRightEncoderAngle()
{
    // Encoder 1 (RIGHT): SDA=D1, SCL=D2
    uint16_t raw = readRawAngle(ENCODER1_SDA, ENCODER1_SCL, AS5600_ADDR);
    return rawAngleToDegrees(raw);
}

float getLeftEncoderAngle()
{
    // Encoder 2 (LEFT): SDA=D7, SCL=D0
    uint16_t raw = readRawAngle(ENCODER2_SDA, ENCODER2_SCL, AS5600_ADDR);
    return rawAngleToDegrees(raw);
}

void initEncoders()
{
    Serial.println("\n========== I2C ENCODER DEBUG ==========");

    // Initialize Encoder 1 on D1 (SDA), D2 (SCL) - RIGHT
    Serial.print("Initializing ENCODER 1 (RIGHT) on D1 (SDA), D2 (SCL)...");
    switchI2CBus(ENCODER1_SDA, ENCODER1_SCL);
    Serial.println(" Done");

    // Initialize Encoder 2 on D7 (SDA), D0 (SCL) - LEFT
    Serial.print("Initializing ENCODER 2 (LEFT) on D7 (SDA), D0 (SCL)...");
    switchI2CBus(ENCODER2_SDA, ENCODER2_SCL);
    Serial.println(" Done");

    // Scan Encoder 1 bus
    Serial.println("\nScanning ENCODER 1 I2C bus (D1/D2)...");
    switchI2CBus(ENCODER1_SDA, ENCODER1_SCL);
    byte error, address;
    int nDevices = 0;

    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("✓ Device found at 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        }
    }

    if (nDevices == 0)
        Serial.println("✗ No devices on ENCODER 1 bus!");
    else
    {
        Serial.print("✓ Found ");
        Serial.print(nDevices);
        Serial.println(" device(s) on ENCODER 1 bus");
    }

    // Scan Encoder 2 bus
    Serial.println("\nScanning ENCODER 2 I2C bus (D7/D0)...");
    Serial.println("DEBUG: About to switch to ENCODER 2 pins...");
    switchI2CBus(ENCODER2_SDA, ENCODER2_SCL);
    delay(100); // Wait longer for bus stabilization

    Serial.println("DEBUG: Switched to ENCODER 2 bus, checking connectivity...");

    // Try direct read first
    Wire.beginTransmission(AS5600_ADDR);
    error = Wire.endTransmission();
    Serial.print("DEBUG: Direct transmission to 0x36 error code: ");
    Serial.println(error);

    nDevices = 0;
    int errorCount = 0;

    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("✓ Device found at 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        }
        else if (error != 4) // error 4 = no device responded
        {
            errorCount++;
        }
    }

    if (nDevices == 0)
    {
        Serial.println("✗ No devices on ENCODER 2 bus!");
        Serial.print("DEBUG: I2C errors detected: ");
        Serial.println(errorCount);
        Serial.println("TROUBLESHOOTING ENCODER 2:");
        Serial.println("1. Check D7 (GPIO13) and D0 (GPIO15) connections");
        Serial.println("2. Verify 4.7kΩ pull-up resistors on SDA and SCL");
        Serial.println("3. Check encoder power supply (3.3V)");
        Serial.println("4. Try swapping SDA/SCL cables");
    }
    else
    {
        Serial.print("✓ Found ");
        Serial.print(nDevices);
        Serial.println(" device(s) on ENCODER 2 bus");
    }

    Serial.println("=======================================\n");
}

// ============= SERVO CONTROL FUNCTIONS =============
void setSpeed(int left, int right)
{
    // left: negative = backward, 0 = stop, positive = forward
    // right: negative = backward, 0 = stop, positive = forward
    leftServo.writeMicroseconds(stopL + left);
    rightServo.writeMicroseconds(stopR - right);
}

void stopMotors()
{
    setSpeed(0, 0);
    isMoving = false;
    robotState = 0;
    Serial.println("Motors stopped");
}

void moveForward()
{
    setSpeed(500, 500); // Full speed forward
    isMoving = true;
    robotState = 1;
    Serial.println("Moving forward");
}

void moveBackward()
{
    setSpeed(-500, -500); // Full speed backward
    isMoving = true;
    robotState = 2;
    Serial.println("Moving backward");
}

void turnRight()
{
    setSpeed(500, 0); // Left wheel forward, right wheel stop
    isMoving = true;
    robotState = 3;
    Serial.println("Turning right");
}

void turnLeft()
{
    setSpeed(0, 500); // Right wheel forward, left wheel stop
    isMoving = true;
    robotState = 4;
    Serial.println("Turning left");
}

void backwardTurn180()
{
    // Move backward and turn 180 degrees
    moveBackward();
    robotState = 5;
    Serial.println("Backward 180 degrees");
}

void backwardTurn360()
{
    // Move backward and turn 360 degrees
    moveBackward();
    robotState = 6;
    Serial.println("Backward 360 degrees");
}

// ============= WEB SERVER HANDLERS =============
const char INDEX_HTML[] = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }
        .container {
            background: white;
            border-radius: 20px;
            box-shadow: 0 10px 40px rgba(0,0,0,0.3);
            padding: 30px;
            max-width: 500px;
            width: 100%;
        }
        h1 {
            text-align: center;
            color: #333;
            margin-bottom: 30px;
            font-size: 28px;
        }
        .control-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            margin-bottom: 30px;
        }
        button {
            padding: 20px;
            font-size: 16px;
            font-weight: bold;
            border: none;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.3s ease;
            color: white;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        button:hover {
            transform: translateY(-3px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.2);
        }
        button:active {
            transform: translateY(0);
        }
        .btn-forward {
            grid-column: 2;
            background: #4CAF50;
        }
        .btn-forward:hover {
            background: #45a049;
        }
        .btn-left {
            background: #2196F3;
        }
        .btn-left:hover {
            background: #0b7dda;
        }
        .btn-right {
            background: #2196F3;
        }
        .btn-right:hover {
            background: #0b7dda;
        }
        .btn-backward {
            grid-column: 1 / -1;
            background: #f44336;
        }
        .btn-backward:hover {
            background: #da190b;
        }
        .btn-stop {
            grid-column: 1 / -1;
            background: #FF9800;
            font-size: 18px;
        }
        .btn-stop:hover {
            background: #e68900;
        }
        .special-controls {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }
        .btn-back180 {
            background: #9C27B0;
        }
        .btn-back180:hover {
            background: #7b1fa2;
        }
        .btn-back360 {
            background: #E91E63;
        }
        .btn-back360:hover {
            background: #c2185b;
        }
        .status {
            text-align: center;
            margin-top: 30px;
            padding: 15px;
            background: #f5f5f5;
            border-radius: 10px;
            color: #666;
        }
        .status-text {
            font-weight: bold;
            color: #333;
        }
        .encoder-display {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin-top: 20px;
            margin-bottom: 20px;
        }
        .encoder-card {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 10px;
            text-align: center;
        }
        .encoder-label {
            font-size: 14px;
            opacity: 0.9;
            margin-bottom: 10px;
            font-weight: bold;
        }
        .encoder-value {
            font-size: 28px;
            font-weight: bold;
            font-family: 'Courier New', monospace;
        }
        .encoder-unit {
            font-size: 12px;
            margin-top: 5px;
            opacity: 0.8;
        }
        .btn-update {
            width: 100%;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 15px;
            font-size: 16px;
            font-weight: bold;
            border: none;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.3s ease;
            color: white;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .btn-update:hover {
            transform: translateY(-3px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.2);
        }
        .btn-update:active {
            transform: translateY(0);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Robot Control</h1>
        
        <div class="encoder-display">
            <div class="encoder-card">
                <div class="encoder-label">🔴 Right Wheel</div>
                <div class="encoder-value" id="rightEncoderValue">0.0</div>
                <div class="encoder-unit">degrees</div>
            </div>
            <div class="encoder-card">
                <div class="encoder-label">🔵 Left Wheel</div>
                <div class="encoder-value" id="leftEncoderValue">0.0</div>
                <div class="encoder-unit">degrees</div>
            </div>
        </div>
        
        <div class="control-grid">
            <button class="btn-forward" onclick="sendCommand('forward')">Forward</button>
            <button class="btn-left" onclick="sendCommand('turnLeft')">← Left</button>
            <button class="btn-right" onclick="sendCommand('turnRight')">Right →</button>
        </div>
        
        <div class="control-grid">
            <button class="btn-backward" onclick="sendCommand('backward')">Backward</button>
        </div>
        
        <div class="control-grid">
            <button class="btn-stop" onclick="sendCommand('stop')">🛑 STOP</button>
        </div>
        
        <div class="special-controls" style="margin-top: 20px;">
            <button class="btn-back180" onclick="sendCommand('back180')">Back 180°</button>
            <button class="btn-back360" onclick="sendCommand('back360')">Back 360°</button>
        </div>
        
        <div style="margin-top: 20px;">
            <button class="btn-update" onclick="openOTAUpdate()">📤 Update Firmware</button>
        </div>
        
        <div class="status">
            <p>Status: <span class="status-text" id="status">Ready</span></p>
        </div>
    </div>

    <script>
        function sendCommand(command) {
            fetch('/cmd?action=' + command)
                .then(response => response.text())
                .then(data => {
                    updateStatus(command);
                })
                .catch(error => console.error('Error:', error));
        }

        function updateStatus(command) {
            const statusMap = {
                'forward': '→ Moving Forward',
                'backward': '← Moving Backward',
                'turnLeft': '↺ Turning Left',
                'turnRight': '↻ Turning Right',
                'stop': '🛑 Stopped',
                'back180': '↩ Backward 180°',
                'back360': '↩↩ Backward 360°'
            };
            document.getElementById('status').textContent = statusMap[command] || 'Command sent';
        }

        function updateEncoders() {
            fetch('/encoders')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('rightEncoderValue').textContent = data.rightAngle.toFixed(1);
                    document.getElementById('leftEncoderValue').textContent = data.leftAngle.toFixed(1);
                })
                .catch(error => console.error('Error:', error));
        }

        function openOTAUpdate() {
            window.location.href = '/update';
        }

        // Update encoder values every 500ms
        setInterval(updateEncoders, 500);
    </script>
</body>
</html>
)=====";

void handleRoot()
{
    server.send(200, "text/html", INDEX_HTML);
}

void handleCommand()
{
    if (server.hasArg("action"))
    {
        String action = server.arg("action");

        if (action == "forward")
        {
            moveForward();
        }
        else if (action == "backward")
        {
            moveBackward();
        }
        else if (action == "turnLeft")
        {
            turnLeft();
        }
        else if (action == "turnRight")
        {
            turnRight();
        }
        else if (action == "stop")
        {
            stopMotors();
        }
        else if (action == "back180")
        {
            backwardTurn180();
        }
        else if (action == "back360")
        {
            backwardTurn360();
        }

        server.send(200, "text/plain", "OK");
    }
    else
    {
        server.send(400, "text/plain", "Missing action parameter");
    }
}

void handleEncoders()
{
    float rightAngle = getRightEncoderAngle();
    float leftAngle = getLeftEncoderAngle();

    String json = "{\"rightAngle\":" + String(rightAngle, 2) +
                  ",\"leftAngle\":" + String(leftAngle, 2) + "}";

    server.send(200, "application/json", json);
}

// ============= SETUP =============
void setup()
{
    Serial.begin(9600);
    delay(100);

    Serial.println("\n\n");
    Serial.println("========== Robot Control Startup ==========");

    // Initialize servos
    rightServo.attach(RIGHT_SERVO_PIN);
    leftServo.attach(LEFT_SERVO_PIN);
    stopMotors();
    delay(500);

    Serial.println("Servos initialized");

    // Initialize encoders
    initEncoders();

    // WiFi AP setup
    Serial.println("\nStarting WiFi AP...");
    WiFi.mode(WIFI_AP_STA); // Allow both AP and STA modes
    WiFi.disconnect();      // Disconnect from any previous network
    delay(100);

    // Configure static IP for AP
    WiFi.softAPConfig(apIP, gateway, subnet);

    // Start AP with channel 1 and max TX power
    WiFi.softAP(ssid, password, 1, false, 8);
    delay(500);

    // Get AP IP and verify
    IPAddress IP = WiFi.softAPIP();

    Serial.println("\n========== WiFi AP Configuration ==========");
    Serial.print("AP Mode: ENABLED");
    Serial.println();
    Serial.print("AP SSID: ");
    Serial.println(ssid);
    Serial.print("AP Password: ");
    Serial.println(password);
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.print("AP Gateway: ");
    Serial.println(gateway);
    Serial.print("AP Subnet: ");
    Serial.println(subnet);
    Serial.print("AP MAC: ");
    Serial.println(WiFi.softAPmacAddress());
    Serial.println("==========================================");

    // Web server routes
    server.on("/", handleRoot);
    server.on("/cmd", handleCommand);
    server.on("/encoders", handleEncoders);

    // Initialize ElegantOTA
    ElegantOTA.begin(&server);

    server.begin();
    Serial.println("\nWeb server started on port 80");
    Serial.println("==========================================");
    Serial.println("ROBOT READY FOR CONTROL!");
    Serial.println("==========================================");
    Serial.println("\nConnect to WiFi AP and open in browser:");
    Serial.print("http://192.168.4.1");
    Serial.println();
    Serial.print("Firmware update: http://192.168.4.1/update");
    Serial.println();
    Serial.println("==========================================\n");
}

// ============= LOOP =============
void loop()
{
    server.handleClient();
    ElegantOTA.loop();
    delay(10);
}
