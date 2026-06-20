#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// пинове за мотори
#define Pin_Motor_1_PWM 25
#define Pin_Motor_1_IN1 27
#define Pin_Motor_1_IN2 26

#define Pin_Motor_2_PWM 13
#define Pin_Motor_2_IN1 12
#define Pin_Motor_2_IN2 14

#define Pin_Motor_3_PWM 32
#define Pin_Motor_3_IN1 33
#define Pin_Motor_3_IN2 15

#define Pin_Motor_4_PWM 4
#define Pin_Motor_4_IN1 16
#define Pin_Motor_4_IN2 17

// пинове за енкодери
#define ENC_Left_C1  5
#define ENC_Left_C2  2
#define ENC_Right_C1 18
#define ENC_Right_C2 19
#define ENC_M3_C1 36
#define ENC_M3_C2 39

// пинове за потенциометри
#define POT_1_PIN 34  // за избор на мисия
#define POT_2_PIN 35  // за скоростната кутия 

// I2C
#define SDA_PIN 21
#define SCL_PIN 22
#define PCF8574_ADDR 0x27

// Wi-Fi
const char* ssid     = "";
const char* password = "";

const float wheelDiameter = 4.2;
const int   EncCRP        = 350;
const float countPerCm    = EncCRP / (PI * wheelDiameter);

float MOVE_KP = 2.0;
float MOVE_KI = 0.04;
float MOVE_KD = 1.0;

float TURN_KP = 3.0;
float TURN_KD = 2.3;

int MIN_SPEED = 65;

// дефинираме BNO055 жироскопа
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// зануляване на енкодерите
volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;
volatile long m3EncoderCount    = 0;

// матрица с мисии (редове = скоростни нива, колони = мисии)
const int missionMatrix[4][5] = {
    {1,  2,  3,  4,  5},   // Скорост 1
    {6,  7,  8,  9,  10},  // Скорост 2
    {11, 12, 13, 14, 15},  // Скорост 3
    {16, 17, 18, 19, 20}   // Скорост 4
};

volatile bool stopMissionFlag = false; // помага за прекъсване на мисията при натискане на стоп бутона

void IRAM_ATTR readLeftEnc()
{
    if (digitalRead(ENC_Left_C2) > 0) 
    leftEncoderCount++;
    else                            
    leftEncoderCount--;
}

void IRAM_ATTR readRightEnc()
{
    if (digitalRead(ENC_Right_C2) > 0) 
    rightEncoderCount++;
    else                             
    rightEncoderCount--;
}

void IRAM_ATTR readM3Enc()
{
    if (digitalRead(ENC_M3_C2) > 0) 
    m3EncoderCount++;
    else                            
    m3EncoderCount--;
}


uint8_t readButtons() {
    // искаме дании от PCF8574, който е свързан към бутоните
    Wire.requestFrom(PCF8574_ADDR, 1);
    
    if (Wire.available()) {
        uint8_t rawData = Wire.read(); // Прочитаме оригиналния байт 
        uint8_t invertedData = ~rawData; // Обръщаме всички битове, защото бутоните са активни ниско (0 когато са натиснати)
        
        return invertedData; // Връщаме обърнатия резултат
    }
    return 0;
}

// ======================== СТОП ФУНКЦИИ ========================
void stopRobot() {
    ledcWrite(0, 0); 
    ledcWrite(1, 0);
    digitalWrite(Pin_Motor_1_IN1, LOW); 
    digitalWrite(Pin_Motor_1_IN2, LOW);
    digitalWrite(Pin_Motor_2_IN1, LOW); 
    digitalWrite(Pin_Motor_2_IN2, LOW);
}

void stopAttachment() {
    ledcWrite(2, 0); 
    ledcWrite(3, 0);
    digitalWrite(Pin_Motor_3_IN1, LOW); 
    digitalWrite(Pin_Motor_3_IN2, LOW);
    digitalWrite(Pin_Motor_4_IN1, LOW);
    digitalWrite(Pin_Motor_4_IN2, LOW);
}
bool checkStopDelay(int ms) {
    if (stopMissionFlag == true) {
        return true;
    }
    
    unsigned long start = millis();
    
    while (millis() - start < ms) {
        //безжичната връзка за качване на код (Over-The-Air)
        ArduinoOTA.handle();
        // Вземаме текущото състояние на бутоните
        uint8_t btns = readButtons();
        
        // Разшифроваме побитовите маски по дългия начин:
        // (1 << 7) съответства на стойност 128 (00000001 става 10000000)
        // (1 << 3) съответства на стойност 8   (00000001 става 00001000)
        bool button7_isPressed = ((btns & 128) != 0); 
        bool button3_isPressed = ((btns & 8) != 0);
        
        if (button7_isPressed == true && button3_isPressed == false) {
            stopMissionFlag = true; 
            stopRobot();            
            stopAttachment();       
            return true;            
        }
        delay(5);
    }
    
    return stopMissionFlag;
}

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// пинове за мотори
#define Pin_Motor_1_PWM 25
#define Pin_Motor_1_IN1 27
#define Pin_Motor_1_IN2 26

#define Pin_Motor_2_PWM 13
#define Pin_Motor_2_IN1 12
#define Pin_Motor_2_IN2 14

#define Pin_Motor_3_PWM 32
#define Pin_Motor_3_IN1 33
#define Pin_Motor_3_IN2 15

#define Pin_Motor_4_PWM 4
#define Pin_Motor_4_IN1 16
#define Pin_Motor_4_IN2 17

// пинове за енкодери
#define ENC_Left_C1  5
#define ENC_Left_C2  2
#define ENC_Right_C1 18
#define ENC_Right_C2 19
#define ENC_M3_C1 36
#define ENC_M3_C2 39

// пинове за потенциометри
#define POT_1_PIN 34  // за избор на мисия
#define POT_2_PIN 35  // за скоростната кутия 

// I2C
#define SDA_PIN 21
#define SCL_PIN 22
#define PCF8574_ADDR 0x27

// Wi-Fi
const char* ssid     = "";
const char* password = "";

const float wheelDiameter = 4.2;
const int   EncCRP        = 350;
const float countPerCm    = EncCRP / (PI * wheelDiameter);

float MOVE_KP = 2.0;
float MOVE_KI = 0.04;
float MOVE_KD = 1.0;

float TURN_KP = 3.0;
float TURN_KD = 2.3;

int MIN_SPEED = 65;

// дефинираме BNO055 жироскопа
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// зануляване на енкодерите
volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;
volatile long m3EncoderCount    = 0;

// матрица с мисии (редове = скоростни нива, колони = мисии)
const int missionMatrix[4][5] = {
    {1,  2,  3,  4,  5},   // Скорост 1
    {6,  7,  8,  9,  10},  // Скорост 2
    {11, 12, 13, 14, 15},  // Скорост 3
    {16, 17, 18, 19, 20}   // Скорост 4
};

volatile bool stopMissionFlag = false; // помага за прекъсване на мисията при натискане на стоп бутона

void IRAM_ATTR readLeftEnc()
{
    if (digitalRead(ENC_Left_C2) > 0) 
    leftEncoderCount++;
    else                            
    leftEncoderCount--;
}

void IRAM_ATTR readRightEnc()
{
    if (digitalRead(ENC_Right_C2) > 0) 
    rightEncoderCount++;
    else                             
    rightEncoderCount--;
}

void IRAM_ATTR readM3Enc()
{
    if (digitalRead(ENC_M3_C2) > 0) 
    m3EncoderCount++;
    else                            
    m3EncoderCount--;
}


uint8_t readButtons() {
    // искаме дании от PCF8574, който е свързан към бутоните
    Wire.requestFrom(PCF8574_ADDR, 1);
    
    if (Wire.available()) {
        uint8_t rawData = Wire.read(); // Прочитаме оригиналния байт 
        uint8_t invertedData = ~rawData; // Обръщаме всички битове, защото бутоните са активни ниско (0 когато са натиснати)
        
        return invertedData; // Връщаме обърнатия резултат
    }
    return 0;
}


void stopRobot() { // метод за спиране на робота
    ledcWrite(0, 0); 
    ledcWrite(1, 0);
    digitalWrite(Pin_Motor_1_IN1, LOW); 
    digitalWrite(Pin_Motor_1_IN2, LOW);
    digitalWrite(Pin_Motor_2_IN1, LOW); 
    digitalWrite(Pin_Motor_2_IN2, LOW);
}

void stopAttachment() { // метод за спиране на моторите за представките
    ledcWrite(2, 0); 
    ledcWrite(3, 0);
    digitalWrite(Pin_Motor_3_IN1, LOW); 
    digitalWrite(Pin_Motor_3_IN2, LOW);
    digitalWrite(Pin_Motor_4_IN1, LOW);
    digitalWrite(Pin_Motor_4_IN2, LOW);
}
bool checkStopDelay(int ms) {
    if (stopMissionFlag == true) {
        return true;
    }
    
    unsigned long start = millis();
    
    while (millis() - start < ms) {
        //безжичната връзка за качване на код (Over-The-Air)
        ArduinoOTA.handle();
        // Вземаме текущото състояние на бутоните
        uint8_t btns = readButtons();
        
        bool button7_isPressed = ((btns & 128) != 0); 
        bool button3_isPressed = ((btns & 8) != 0);
        
        if (button7_isPressed == true && button3_isPressed == false) {
            stopMissionFlag = true; 
            stopRobot();            
            stopAttachment();       
            return true;            
        }
        delay(5);
    }
    
    return stopMissionFlag;
}

// допълнителни методи 
double getHeading() {
    sensors_event_t event;
    bno.getEvent(&event);
    return event.orientation.x;
}

float angleDiff(float from, float to) {
    float d = to - from;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// движение напред 
void move(float distanceCm, int speed, ) {
    if (stopMissionFlag) return;

    leftEncoderCount = 0; rightEncoderCount = 0;
    long target = (long)(countPerCm * distanceCm);
    float startHeading = getHeading();
    float integral = 0.0f,
    prevError = 0.0f;
    /*
    bool forward = (direction == 'F');

    if (forward) {
        digitalWrite(Pin_Motor_1_IN1, LOW);  digitalWrite(Pin_Motor_1_IN2, HIGH);
        digitalWrite(Pin_Motor_2_IN1, LOW);  digitalWrite(Pin_Motor_2_IN2, HIGH);
    } else {
        digitalWrite(Pin_Motor_1_IN1, HIGH); digitalWrite(Pin_Motor_1_IN2, LOW);
        digitalWrite(Pin_Motor_2_IN1, HIGH); digitalWrite(Pin_Motor_2_IN2, LOW);
    }
    */

    unsigned long lastTime = micros();
    while (true) {
        ArduinoOTA.handle();
        { uint8_t b = readButtons(); #
            if ((b & (1<<7)) && !(b & (1<<3))) 
            { stopMissionFlag = true; 
                stopRobot(); 
                return; 
            }
        }

        long avgCount = (abs(leftEncoderCount) + abs(rightEncoderCount)) / 2;
        if (avgCount >= target)  break;

        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastTime = now;

        float currHeading = getHeading();
        float error = angleDiff(currHeading, startHeading);
        integral += error * dt;
        if (integral >  30.0f) 
            integral =  30.0f;
        if (integral < -30.0f) 
            integral = -30.0f;

        float derivative = (error - prevError) / dt;
        prevError = error;

        float correction = MOVE_KP * error + MOVE_KI * integral + MOVE_KD * derivative;
        if (!forward) correction = -correction;

        long slowStart = (long)(target * 0.70);
        int currentSpeed = speed;
        if (avgCount > slowStart) {
            currentSpeed = speed - (int)((long)(speed - MIN_SPEED) * (avgCount - slowStart) / (target - slowStart));
        }
        if (currentSpeed < MIN_SPEED) currentSpeed = MIN_SPEED;

        int leftSpeed  = currentSpeed + (int)correction;
        int rightSpeed = currentSpeed - (int)correction;

        if (leftSpeed  > 255) leftSpeed  = 255;
        if (leftSpeed  < MIN_SPEED) leftSpeed  = MIN_SPEED;
        if (rightSpeed > 255) rightSpeed = 255;
        if (rightSpeed < MIN_SPEED) rightSpeed = MIN_SPEED;

        ledcWrite(0, leftSpeed); ledcWrite(1, rightSpeed);
        delay(10);
    }
    ledcWrite(0, MIN_SPEED); ledcWrite(1, MIN_SPEED);
    checkStopDelay(80); stopRobot(); checkStopDelay(50);
}
// движение назад
void moveBack(float distanceCm, int speed) {
    if (stopMissionFlag) return;

    leftEncoderCount = 0; rightEncoderCount = 0;
    long target = (long)(countPerCm * distanceCm);
    float startHeading = getHeading();
    float integral = 0.0f, prevError = 0.0f;

    digitalWrite(Pin_Motor_1_IN1, HIGH); 
    digitalWrite(Pin_Motor_1_IN2, LOW);
    digitalWrite(Pin_Motor_2_IN1, HIGH); 
    digitalWrite(Pin_Motor_2_IN2, LOW);

    unsigned long lastTime = micros();
    while (true) {
        ArduinoOTA.handle();
        { uint8_t b = readButtons(); 
            if ((b & (1<<7)) && !(b & (1<<3))) 
            { 
                stopMissionFlag = true; 
                stopRobot(); 
                return; 
            } 
        }

        long avgCount = (abs(leftEncoderCount) + abs(rightEncoderCount)) / 2;
        if (avgCount >= target) break;

        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastTime = now;

        float currHeading = getHeading();
        float error = angleDiff(currHeading, startHeading);
        integral += error * dt;
        if (integral >  30.0f) integral =  30.0f;
        if (integral < -30.0f) integral = -30.0f;

        float derivative = (error - prevError) / dt;
        prevError = error;

        float correction = -(MOVE_KP * error + MOVE_KI * integral + MOVE_KD * derivative);

        long slowStart = (long)(target * 0.70);
        int currentSpeed = speed;
        if (avgCount > slowStart) {
            currentSpeed = speed - (int)((long)(speed - MIN_SPEED) * (avgCount - slowStart) / (target - slowStart));
        }
        if (currentSpeed < MIN_SPEED) currentSpeed = MIN_SPEED;

        int leftSpeed  = currentSpeed + (int)correction;
        int rightSpeed = currentSpeed - (int)correction;

        if (leftSpeed  > 255) leftSpeed  = 255;
        if (leftSpeed  < MIN_SPEED) leftSpeed  = MIN_SPEED;
        if (rightSpeed > 255) rightSpeed = 255;
        if (rightSpeed < MIN_SPEED) rightSpeed = MIN_SPEED;

        ledcWrite(0, leftSpeed); ledcWrite(1, rightSpeed);
        delay(10);
    }
    ledcWrite(0, MIN_SPEED); ledcWrite(1, MIN_SPEED);
    checkStopDelay(80); stopRobot(); checkStopDelay(50);
}

// завиване
void turn(float angle, int speed) {
    if (stopMissionFlag) return;

    float lastHeading = getHeading();
    float angleTurned = 0.0f;
    float slowZone = 25.0f, stopZone = 1.5f, prevError = 0.0f;

    if (angle > 0) {
        digitalWrite(Pin_Motor_1_IN1, HIGH);
        digitalWrite(Pin_Motor_1_IN2, LOW);
        digitalWrite(Pin_Motor_2_IN1, LOW); 
        digitalWrite(Pin_Motor_2_IN2, HIGH);
    } else {
        digitalWrite(Pin_Motor_1_IN1, LOW);
        digitalWrite(Pin_Motor_1_IN2, HIGH);
        digitalWrite(Pin_Motor_2_IN1, HIGH);
        digitalWrite(Pin_Motor_2_IN2, LOW);
    }

    unsigned long lastTime = micros();
    while (true) {
        ArduinoOTA.handle();
        { uint8_t b = readButtons(); 
            if ((b & (1<<7)) && !(b & (1<<3))) 
            { 
                stopMissionFlag = true; 
                stopRobot(); 
                return; 
            } 
        }

        float currHeading = getHeading();
        float delta = angleDiff(lastHeading, currHeading);
        angleTurned += delta; lastHeading = currHeading;

        float remaining = abs(angle) - abs(angleTurned);
        if (remaining <= stopZone) break;

        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastTime = now;

        float error = remaining;
        float derivative = (error - prevError) / dt;
        prevError = error;

        float pidOut = TURN_KP * error - TURN_KD * abs(derivative);
        int motorSpeed = (int)pidOut;
        if (motorSpeed > speed)     motorSpeed = speed;
        if (motorSpeed < MIN_SPEED) motorSpeed = MIN_SPEED;

        if (remaining < slowZone) {
            int slowSpeed = (int)((remaining - stopZone) / (slowZone - stopZone) * (speed - MIN_SPEED) + MIN_SPEED);
            if (motorSpeed > slowSpeed) motorSpeed = slowSpeed;
        }

        if (motorSpeed > 255)       motorSpeed = 255;
        if (motorSpeed < MIN_SPEED) motorSpeed = MIN_SPEED;

        ledcWrite(0, motorSpeed); ledcWrite(1, motorSpeed);
        delay(10);
    }
    stopRobot();
    checkStopDelay(50);
}

// методи за представките
void m3Forward(int speed, int timeMs) {
    if (stopMissionFlag) return;
    digitalWrite(Pin_Motor_3_IN1, LOW);
    digitalWrite(Pin_Motor_3_IN2, HIGH);
    ledcWrite(2, speed); 
    checkStopDelay(timeMs); 
    stopAttachment();
}
void m3Back(int speed, int timeMs) {
    if (stopMissionFlag) return;
    digitalWrite(Pin_Motor_3_IN1, HIGH);
    digitalWrite(Pin_Motor_3_IN2, LOW);
    ledcWrite(2, speed);
    checkStopDelay(timeMs); 
    stopAttachment();
}
void m4Forward(int speed, int timeMs) {
    if (stopMissionFlag) return;
    digitalWrite(Pin_Motor_4_IN1, LOW); digitalWrite(Pin_Motor_4_IN2, HIGH);
    ledcWrite(3, speed);
 checkStopDelay(timeMs); 
stopAttachment();
}
void m4Back(int speed, int timeMs) {
    if (stopMissionFlag) return;
    digitalWrite(Pin_Motor_4_IN1, HIGH); digitalWrite(Pin_Motor_4_IN2, LOW);
    ledcWrite(3, speed);
 checkStopDelay(timeMs); 
stopAttachment();
}

void resetEncoders() {
    leftEncoderCount = 0; rightEncoderCount = 0;
    m3EncoderCount = 0;
}

// мисии 
void run1() {
    resetEncoders(); 
    move(64, 100, 'F');
    checkStopDelay(500);
    turn(87, 70);
    checkStopDelay(500);
    move(2, 100, 'F');
    checkStopDelay(500);
    moveBack(5, 100);
    checkStopDelay(500);
    turn(-87, 70);
    checkStopDelay(500);
    moveBack(64, 100);
}

void run2() {
    resetEncoders();
    move(87, 100, 'F');
    checkStopDelay(500);
    turn(67, 70);
    checkStopDelay(500);
    move(8, 100, 'F');
    checkStopDelay(500);
    moveBack(10, 100);
    checkStopDelay(500);
    resetEncoders();
    turn(-70, 70);
    checkStopDelay(500);
    moveBack(37, 100);
    turn(15, 70);
    moveBack(50, 100);
}

void run3() {
    resetEncoders();
    move(8, 100, 'F');
    moveBack(10, 100);
}

void run4() {
    resetEncoders(); 
    turn(8, 70);
    move(21, 100, 'F');
    moveBack(27, 100);
}

void run5() {
    resetEncoders(); 
    checkStopDelay(500);
    move(55, 100, 'F');
    checkStopDelay(500);
    turn(90, 70);
    checkStopDelay(500);
    move(5, 100, 'F');
    checkStopDelay(500);
    m4Forward(90, 150);
    checkStopDelay(500);
    moveBack(5, 100);
    checkStopDelay(500);
    turn(-90, 70);
    checkStopDelay(500);
    moveBack(55, 100);
}

void run6() {
    resetEncoders(); 
    turn(-35, 70);
    move(2, 100, 'F');
    moveBack(8, 100);
}

void run8() {
    resetEncoders(); 
    checkStopDelay(500);
    move(52, 100, 'F');
    checkStopDelay(500);
    turn(90, 70);
    checkStopDelay(500);
    move(2, 100, 'F');
    checkStopDelay(500);
    moveBack(3, 100);
    checkStopDelay(500);
    turn(-90, 70);
    checkStopDelay(500);
    moveBack(55, 100);
}

// меню система 
void executeRun(int missionNumber, int speedLevel) {
    stopMissionFlag = false;

    Serial.printf("\n>>> СТАРТ НА МИСИЯ %d (Скорост ниво: %d) <<<\n", missionNumber, speedLevel);

    switch (missionNumber) {
        case 1:  run1(); break;
        case 2:  run2(); break;
        case 3:  run3(); break;
        case 4:  run4(); break;
        case 5:  run5(); break;
        case 6:  run6(); break;
        case 7:  run8(); break;
       
        default:
            Serial.printf("Мисия %d не е дефинирана!\n", missionNumber);
            break;
    }

    if (stopMissionFlag) {
        Serial.println("ПРЕКЪСНАТО ");
    } else {
        Serial.println("КРАЙ НА МИСИЯТА\n");
    }

    stopMissionFlag = false;
    stopRobot();
    stopAttachment();
}

// код който се рънва само веднъж
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n--- MEHANO ROBOT BOOT ");

    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    delay(100);

    // PCF8574 бутони
    Wire.beginTransmission(PCF8574_ADDR);
    Wire.write(0xFF);
    Wire.endTransmission();
    delay(50);

    // Жироскоп
    if (!bno.begin()) {
        Serial.println("BNO055 не работи!");
        while (1);
    }
    bno.setExtCrystalUse(true);
    delay(100);
    Serial.println("BNO055 е онлайн!");

    // Мотори
    pinMode(Pin_Motor_1_IN1, OUTPUT); pinMode(Pin_Motor_1_IN2, OUTPUT);
    pinMode(Pin_Motor_2_IN1, OUTPUT); pinMode(Pin_Motor_2_IN2, OUTPUT);
    pinMode(Pin_Motor_3_IN1, OUTPUT); pinMode(Pin_Motor_3_IN2, OUTPUT);
    pinMode(Pin_Motor_4_IN1, OUTPUT); pinMode(Pin_Motor_4_IN2, OUTPUT);
    digitalWrite(Pin_Motor_3_IN1, LOW); digitalWrite(Pin_Motor_3_IN2, LOW);

    ledcSetup(0, 5000, 8); ledcAttachPin(Pin_Motor_1_PWM, 0);
    ledcSetup(1, 5000, 8); ledcAttachPin(Pin_Motor_2_PWM, 1);
    ledcSetup(2, 5000, 8); ledcAttachPin(Pin_Motor_3_PWM, 2);
    ledcSetup(3, 5000, 8); ledcAttachPin(Pin_Motor_4_PWM, 3);

    // Енкодери
    pinMode(ENC_Left_C1,  INPUT_PULLUP); pinMode(ENC_Left_C2,  INPUT_PULLUP);
    pinMode(ENC_Right_C1, INPUT_PULLUP); pinMode(ENC_Right_C2, INPUT_PULLUP);
    pinMode(ENC_M3_C1, INPUT); 
pinMode(ENC_M3_C2, INPUT);
    

    attachInterrupt(digitalPinToInterrupt(ENC_Left_C1),  readLeftEnc,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_Right_C1), readRightEnc, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_M3_C1),    readM3Enc,    RISING);
    // attachInterrupt за M4 е премахнат

    // Потенциометри
    pinMode(POT_1_PIN, INPUT);
    pinMode(POT_2_PIN, INPUT);

    // Wi-Fi + OTA
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(250); attempts++; }

    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.setHostname("Mehano-Robot");
        ArduinoOTA.begin();
        Serial.print("OTA е готово! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Wi-Fi не се свърза. OTA е изключено.");
    }

    stopRobot();
   

// код който се изпълнява винаги
void loop() {
    ArduinoOTA.handle();

    // Четем потенциометрите 
    int raw1 = analogRead(POT_1_PIN);
    int raw2 = analogRead(POT_2_PIN);

    // рънове 
    int pot1_index = 0;
    if      (raw1 < 300)  pot1_index = 0;
    else if (raw1 < 1300) pot1_index = 1;
    else if (raw1 < 2100) pot1_index = 2;
    else if (raw1 < 3000) pot1_index = 3;
    else                  pot1_index = 4;

    // скоростна кутия 
    int pot2_index = 0;
    if      (raw2 < 250)  pot2_index = 0;
    else if (raw2 < 1000) pot2_index = 1;
    else if (raw2 < 1600) pot2_index = 2;
    else                  pot2_index = 3;

    // Крайното число от матрицата
    int finalNumber = missionMatrix[pot2_index][pot1_index];

    uint8_t btns = readButtons();

    if (btns == 136) {
        executeRun(finalNumber, pot2_index + 1);
        delay(1000); // Дебънс след мисия
    }

    // Сериен монитор за проверка
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 150) {
        Serial.printf("Пот1(мисия): %d | Пот2(скорост): %d | РУН: %d | Чака старт...\r",
                      pot1_index + 1, pot2_index + 1, finalNumber);
        lastPrint = millis();
    }

    delay(20);
}
