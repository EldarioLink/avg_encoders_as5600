#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Servo.h>

// ======================================================
// WiFi AP
// ======================================================
const char *ssid = "RobotControl";
const char *password = "12345678";
const float TURN_90 = 168.0; // Example only!

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
int speedLeft = 44;
int speedRight = 44;
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
        int leftSpeed = 0;
        int rightSpeed = 0;

        if (left.totalAngle < targetLeft)
            leftSpeed = speedLeft;
        else
            leftSpeed = 0;

        if (right.totalAngle > targetRight)
            rightSpeed = -speedRight;
        else
            rightSpeed = 0;

        setSpeed(leftSpeed, rightSpeed);

        if (leftSpeed == 0 && rightSpeed == 0)
        {
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
    float startRight = right.totalAngle;

    float targetLeft = startLeft + TURN_90;
    float targetRight = startRight - TURN_90;

    while (true)
    {
        readEncoders();

        int leftSpeed = 0;
        int rightSpeed = 0;

        if (left.totalAngle < targetLeft)
            leftSpeed = 44;

        if (right.totalAngle > targetRight)
            rightSpeed = -44;

        setSpeed(leftSpeed, rightSpeed);

        if (leftSpeed == 0 && rightSpeed == 0)
            break;

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

<title>AS5600 Monitor</title>

<style>

body{
    background:#1d1d1d;
    color:white;
    font-family:Arial;
    text-align:center;
}

.card{
    display:inline-block;
    margin-top:30px;
    padding:25px;
    background:#2d2d2d;
    border-radius:15px;
    min-width:350px;
}

.value{
    font-size:56px;
    color:#00ff88;
}

.small{
    font-size:24px;
    color:#66ccff;
}

hr{
    margin:30px;
}
    button{
    font-size:24px;
    padding:12px 25px;
    border:none;
    border-radius:10px;
    background:#2196F3;
    color:white;
    cursor:pointer;
}

button:hover{
    background:#1976D2;
}

</style>

</head>

<body>

<div class="card">

<h2>AS5600 Encoders</h2>

<h3>Encoder 1 (D1 / D2)</h3>

<div class="value" id="deg1">0°</div>

<div class="small">
Raw:
<span id="raw1">0</span>
</div>

<hr>

<h3>Encoder 2 (D3 / D4)</h3>

<div class="value" id="deg2">0°</div>

<div class="small">
Raw:
<span id="raw2">0</span>
</div>

<br><br>

<button onclick="turnRight90()">
Turn Right 90°
</button>
</div>

<script>

function update(){

fetch("/data")
.then(r=>r.json())
.then(d=>{

document.getElementById("deg1").innerHTML=d.deg1.toFixed(2)+"°";
document.getElementById("raw2").innerHTML=d.raw2;

document.getElementById("deg2").innerHTML=d.deg2.toFixed(2)+"°";
document.getElementById("raw1").innerHTML=d.raw1;

});

}

setInterval(update,20);

update();
function turnRight90()
{
    fetch("/turnRight90");
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
void handleData()
{
    // Read raw values directly from the encoders
    switchI2CBus(D1, D2);
    uint16_t raw2 = readRawAngle();

    switchI2CBus(D3, D4);
    uint16_t raw1 = readRawAngle();

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