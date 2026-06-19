#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Servo.h>

// ======================================================
// WiFi AP
// ======================================================
const char *ssid = "RobotControl";
const char *password = "12345678";
float TURN_90 = 286.0; // Calibration value — change via web UI

struct Encoder
{
    float lastAngle;
    float totalAngle;
};

void updateEncoder(Encoder &enc, float currentAngle);
void switchI2CBus(int sda, int scl);
uint16_t readRawAngle();
void stopMotors();
void readEncoders();

Servo servo_l;
Servo servo_r;
Servo servo_up;

int stopL = 1470;
int stopR = 1470;
int lift_up = 180;
int lift_down = 0;
int speedLeft = 33;
int speedRight = 33;
float distance = -1;

IPAddress apIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

ESP8266WebServer server(80);

Encoder left = {0, 0};
Encoder right = {0, 0};

// ======================================================
// AS5600
// ======================================================
#define AS5600_ADDR 0x36
#define RAW_ANGLE_H 0x0C

int currentSDA = -1;
int currentSCL = -1;

enum RobotState
{
    IDLE,
    TURNING_RIGHT,
    TURNING_LEFT,
    MOVING_FORWARD,
    MOVING_BACKWARD
};

RobotState robotState = IDLE;

float targetLeft = 0;
float targetRight = 0;
unsigned long turnStartTime = 0;
const unsigned long TURN_TIMEOUT = 5000; // 5 seconds max turn time
void initDrive()
{
    servo_l.attach(12);
    servo_r.attach(14);
    stopMotors();
}
void setSpeed(int left, int right)
{
    servo_l.writeMicroseconds(stopL + left);
    servo_r.writeMicroseconds(stopR - right);
}

void stopMotors()
{
    setSpeed(0, 0);
}

void startTurnRight90()
{
    if (robotState != IDLE)
        return;

    targetLeft = left.totalAngle + TURN_90;
    targetRight = right.totalAngle - TURN_90;
    turnStartTime = millis();

    robotState = TURNING_RIGHT;
}

void updateRobot()
{
    switch (robotState)
    {
    case IDLE:
        stopMotors();
        break;

    case TURNING_RIGHT:
    {
        // Timeout safety: stop after TURN_TIMEOUT ms even if encoder fails
        if (millis() - turnStartTime > TURN_TIMEOUT)
        {
            stopMotors();
            robotState = IDLE;
            break;
        }

        // Use only LEFT encoder to control both wheels (right encoder not working)
        if (left.totalAngle < targetLeft)
        {
            // Both wheels run: left forward, right backward
            setSpeed(speedLeft, -speedRight);
        }
        else
        {
            // Left reached target → stop both
            stopMotors();
            robotState = IDLE;
        }

        break;
    }
    }
}
void turnRight90()
{
    float startLeft = left.totalAngle;

    float targetLeft = startLeft + TURN_90;

    unsigned long startTime = millis();

    while (true)
    {
        readEncoders();

        // Timeout safety
        if (millis() - startTime > TURN_TIMEOUT)
            break;

        // Use only LEFT encoder to control both wheels
        if (left.totalAngle < targetLeft)
        {
            setSpeed(44, -44);
        }
        else
        {
            break;
        }

        yield();
    }

    stopMotors();
}

void resetEncoders()
{
    left.totalAngle = 0;
    right.totalAngle = 0;
}
void switchI2CBus(int sda, int scl)
{
    if (currentSDA != sda || currentSCL != scl)
    {
        Wire.begin(sda, scl);
        Wire.setClock(100000);

        currentSDA = sda;
        currentSCL = scl;

        delay(2);
    }
}
void readEncoders()
{

    // Left encoder (D3/D4)
    switchI2CBus(D3, D4);
    uint16_t raw1 = readRawAngle();
    float deg2 = raw1 * 360.0f / 4096.0f;
    updateEncoder(left, deg2);

    // Right encoder (D1/D2)
    switchI2CBus(D1, D2);
    uint16_t raw2 = readRawAngle();
    float deg1 = raw2 * 360.0f / 4096.0f;
    updateEncoder(right, deg1);
}

void updateEncoder(Encoder &enc, float currentAngle)
{
    float delta = currentAngle - enc.lastAngle;

    // crossed 360 -> 0
    if (delta < -180)
        delta += 360;

    // crossed 0 -> 360
    if (delta > 180)
        delta -= 360;

    enc.totalAngle += delta;
    enc.lastAngle = currentAngle;
}

uint16_t readRawAngle()
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(RAW_ANGLE_H);

    if (Wire.endTransmission(false) != 0)
        return 0;

    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() < 2)
        return 0;

    uint8_t high = Wire.read();
    uint8_t low = Wire.read();

    return ((high << 8) | low) & 0x0FFF;
}

// ======================================================
// Web page
// ======================================================
void handleRoot()
{
    String html = R"rawliteral(
<!DOCTYPE html>
<html>

<head>

<meta charset="UTF-8">

<title>Robot Control</title>

<style>

body{
    background:#1d1d1d;
    color:white;
    font-family:Arial;
    text-align:center;
}

.card{
    display:inline-block;
    margin-top:20px;
    padding:20px;
    background:#2d2d2d;
    border-radius:15px;
    min-width:380px;
    max-width:500px;
}

.value{
    font-size:48px;
    color:#00ff88;
}

.small{
    font-size:20px;
    color:#66ccff;
}

hr{
    margin:20px;
    border:0;
    height:1px;
    background:#444;
}

button{
    font-size:22px;
    padding:10px 22px;
    border:none;
    border-radius:10px;
    background:#2196F3;
    color:white;
    cursor:pointer;
    margin:5px;
}

button:hover{
    background:#1976D2;
}

button:disabled{
    background:#555;
    cursor:not-allowed;
}

button.danger{
    background:#f44336;
}
button.danger:hover{
    background:#d32f2f;
}

button.success{
    background:#4CAF50;
}
button.success:hover{
    background:#388E3C;
}

.cal-box{
    background:#3d3d3d;
    border-radius:10px;
    padding:15px;
    margin:15px 0;
}

.cal-box input[type=range]{
    width:100%;
    margin:10px 0;
}

.cal-box input[type=number]{
    font-size:20px;
    padding:6px 10px;
    border-radius:8px;
    border:none;
    width:100px;
    text-align:center;
    background:#555;
    color:#fff;
}

.cal-label{
    font-size:18px;
    color:#aaa;
    margin-top:5px;
}

.result-box{
    font-size:18px;
    padding:10px;
    border-radius:8px;
    margin-top:10px;
    min-height:24px;
}

.result-box.ok{
    background:#1b5e20;
    color:#a5d6a7;
}

.result-box.info{
    background:#0d47a1;
    color:#90caf9;
}

.result-box.warn{
    background:#e65100;
    color:#ffcc80;
}

</style>

</head>

<body>

<div class="card">

<h2>🤖 Robot Control</h2>

<!-- Encoder Display -->
<h3>Left Encoder (D3/D4)</h3>
<div class="value" id="deg2">0°</div>
<div class="small">Raw: <span id="raw1">0</span> | Total: <span id="total1">0.00</span>°</div>

<hr>

<h3>Right Encoder (D1/D2)</h3>
<div class="value" id="deg1">0°</div>
<div class="small">Raw: <span id="raw2">0</span> | Total: <span id="total2">0.00</span>°</div>

<hr>

<!-- Control Buttons -->
<button onclick="turnRight90()" id="btnTurn">
  Turn Right 90°
</button>

<button class="danger" onclick="stopMotors()" id="btnStop">
  Stop Motors
</button>

<hr>

<!-- Calibration Section -->
<h3>⚙️ Calibration</h3>
<div class="cal-box">
  <div style="font-size:16px;color:#ccc;">
    TURN_90 value (encoder degrees for 90° robot turn)
  </div>
  <input type="range" id="turnSlider" min="0" max="500" value="168" step="1"
         oninput="document.getElementById('turnInput').value=this.value">
  <div>
    <input type="number" id="turnInput" value="168" min="0" max="1000" step="1"
           oninput="document.getElementById('turnSlider').value=this.value">
    <span style="font-size:18px;color:#aaa;"> encoder °</span>
  </div>
  <div style="margin-top:12px;">
    <button class="success" onclick="setTurnValue()">Set Value</button>
    <button onclick="testTurnValue()" id="btnTest">Test Turn</button>
  </div>
  <div id="calResult" class="result-box info">Ready. Set a value and press "Test Turn".</div>
</div>

</div>

<script>

// ---- Encoder data update ----
function update(){
fetch("/data")
.then(r=>r.json())
.then(d=>{
// Left encoder (D3/D4): raw1, deg1, total1
document.getElementById("deg1").innerHTML = d.deg1.toFixed(2)+"°";
document.getElementById("raw1").innerHTML = d.raw1;
document.getElementById("total1").innerHTML = d.total1.toFixed(2);
// Right encoder (D1/D2): raw2, deg2, total2
document.getElementById("deg2").innerHTML = d.deg2.toFixed(2)+"°";
document.getElementById("raw2").innerHTML = d.raw2;
document.getElementById("total2").innerHTML = d.total2.toFixed(2);
});
}

setInterval(update, 50);
update();

// ---- Load current TURN_90 on page load ----
fetch("/getTurnValue").then(r=>r.text()).then(v=>{
  var val = parseFloat(v);
  document.getElementById("turnSlider").value = val;
  document.getElementById("turnInput").value = val;
});

// ---- Turn Right 90 ----
function turnRight90(){
  var btn = document.getElementById("btnTurn");
  btn.disabled = true;
  btn.innerHTML = "Turning...";
  fetch("/turnRight90").then(function(){
    setTimeout(function(){
      btn.disabled = false;
      btn.innerHTML = "Turn Right 90°";
    }, 5000);
  });
}

// ---- Stop Motors ----
function stopMotors(){
  fetch("/stopMotors");
}

// ---- Set TURN_90 value ----
function setTurnValue(){
  var val = document.getElementById("turnInput").value;
  fetch("/setTurnValue?val=" + val).then(function(){
    document.getElementById("calResult").className = "result-box ok";
    document.getElementById("calResult").innerHTML = "✅ TURN_90 set to " + val + "°";
  });
}

// ---- Test Turn ----
function testTurnValue(){
  var val = document.getElementById("turnInput").value;
  var btn = document.getElementById("btnTest");
  btn.disabled = true;
  btn.innerHTML = "Testing...";

  // First reset encoders and state to get a clean measurement
  fetch("/resetEncoders").then(function(){
    // Set the value
    return fetch("/setTurnValue?val=" + val);
  }).then(function(){
    // Execute the turn
    return fetch("/turnRight90");
  }).then(function(){
    // Wait for turn to finish, then read results
    // Increase timeout if your robot turns slowly
    setTimeout(function(){
      fetch("/data").then(r=>r.json()).then(d=>{
        var leftTotal = d.total1;
        var rightTotal = d.total2;
        var leftDeg = Math.abs(leftTotal);
        var rightDeg = Math.abs(rightTotal);
        var avg = ((leftDeg + rightDeg) / 2).toFixed(1);
        document.getElementById("calResult").className = "result-box warn";
        document.getElementById("calResult").innerHTML =
          "📊 Test result: Left=" + leftDeg.toFixed(1) + "°  Right=" + rightDeg.toFixed(1) + "°  Avg=" + avg + "°<br>" +
          "If robot turned exactly 90°, this is your TURN_90 value!";
        btn.disabled = false;
        btn.innerHTML = "Test Turn";
      });
    }, 8000);
  });
}

</script>

</body>

</html>
)rawliteral";

    server.send(200, "text/html", html);
}

void handleTurnRight90()
{
    startTurnRight90();
    server.send(200, "text/plain", "Turning...");
}

void handleSetTurnValue()
{
    if (server.hasArg("val"))
    {
        TURN_90 = server.arg("val").toFloat();
        server.send(200, "text/plain", String(TURN_90));
        Serial.print("TURN_90 set to: ");
        Serial.println(TURN_90);
    }
    else
    {
        server.send(400, "text/plain", "Missing val parameter");
    }
}

void handleGetTurnValue()
{
    server.send(200, "text/plain", String(TURN_90));
}

void handleResetEncoders()
{
    robotState = IDLE;
    stopMotors();
    resetEncoders();
    server.send(200, "text/plain", "OK");
}

void handleStopMotors()
{
    robotState = IDLE;
    stopMotors();
    server.send(200, "text/plain", "Stopped");
}
void handleData()
{
    // Read raw values and update encoder structs for accurate readings
    switchI2CBus(D3, D4);
    uint16_t raw1 = readRawAngle();
    float degL = raw1 * 360.0f / 4096.0f;
    updateEncoder(left, degL);

    switchI2CBus(D1, D2);
    uint16_t raw2 = readRawAngle();
    float degR = raw2 * 360.0f / 4096.0f;
    updateEncoder(right, degR);

    String json = "{";

    json += "\"raw2\":" + String(raw2) + ",";
    json += "\"raw1\":" + String(raw1) + ",";
    json += "\"deg1\":" + String(left.lastAngle, 2) + ",";
    json += "\"deg2\":" + String(right.lastAngle, 2) + ",";

    json += "\"total1\":" + String(left.totalAngle, 2) + ",";
    json += "\"total2\":" + String(right.totalAngle, 2);

    json += "}";

    server.send(200, "application/json", json);
}
// ======================================================
// Setup
// ======================================================
void setup()
{
    Serial.begin(9600);

    initDrive();

    switchI2CBus(D1, D2);

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, gateway, subnet);
    WiFi.softAP(ssid, password);

    Serial.println();
    Serial.println("================================");
    Serial.println("RobotControl AP Started");
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("================================");

    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/turnRight90", handleTurnRight90);
    server.on("/setTurnValue", handleSetTurnValue);
    server.on("/getTurnValue", handleGetTurnValue);
    server.on("/resetEncoders", handleResetEncoders);
    server.on("/stopMotors", handleStopMotors);

    server.begin();

    Serial.println("HTTP server started");
}

// ======================================================
// Loop
// ======================================================
void loop()
{
    readEncoders();
    updateRobot();
    server.handleClient();
}
