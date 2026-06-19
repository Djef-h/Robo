#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ======================== ПИНОВЕ - МОТОРИ ========================
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

// ======================== ПИНОВЕ - ЕНКОДЕРИ ========================
#define ENC_Left_C1  5
#define ENC_Left_C2  2
#define ENC_Right_C1 18
#define ENC_Right_C2 19
#define ENC_M3_C1 36
#define ENC_M3_C2 39
// ENC_M4_C1 и ENC_M4_C2 са премахнати — пинове 34 и 35 се ползват от потенциометрите!

// ======================== ПИНОВЕ - ПОТЕНЦИОМЕТРИ ========================
#define POT_1_PIN 34  // Избор на мисия (колона в матрицата)
#define POT_2_PIN 35  // Избор на скорост (ред в матрицата)

// ======================== I2C / PCF8574 ========================
#define SDA_PIN 21
#define SCL_PIN 22
#define PCF8574_ADDR 0x27

// ======================== WI-FI ========================
const char* ssid     = "HONOR X7d";
const char* password = "mech5795";

// ======================== ПАРАМЕТРИ НА РОБОТА ========================
const float wheelDiameter = 4.2;
const int   EncCRP        = 350;
const float countPerCm    = EncCRP / (PI * wheelDiameter);

float MOVE_KP = 2.0;
float MOVE_KI = 0.04;
float MOVE_KD = 1.0;

float TURN_KP = 3.0;
float TURN_KD = 2.3;

int MIN_SPEED = 65;

// ======================== ЖИРОСКОП ========================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ======================== ЕНКОДЕРИ ========================
volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;
volatile long m3EncoderCount    = 0;
// m4EncoderCount е премахнат — пин 34/35 са за потенциометрите

// ======================== МАТРИЦА С МИСИИ ========================
// Ред    = ниво на скорост (Пот 2): 0=Скор1, 1=Скор2, 2=Скор3, 3=Скор4
// Колона = позиция на мисия  (Пот 1): 0..4
const int missionMatrix[4][5] = {
    {1,  2,  3,  4,  5},   // Скорост 1
    {6,  7,  8,  9,  10},  // Скорост 2
    {11, 12, 13, 14, 15},  // Скорост 3
    {16, 17, 18, 19, 20}   // Скорост 4
};

// ======================== СПИРАЩ ФЛАГ ========================
volatile bool stopMissionFlag = false;

// ======================== ПРЕКЪСВАНИЯ ========================
void IRAM_ATTR readLeftEnc()  { leftEncoderCount  += (digitalRead(ENC_Left_C2)  > 0) ? 1 : -1; }
void IRAM_ATTR readRightEnc() { rightEncoderCount += (digitalRead(ENC_Right_C2) > 0) ? 1 : -1; }
void IRAM_ATTR readM3Enc()    { m3EncoderCount    += (digitalRead(ENC_M3_C2)    > 0) ? 1 : -1; }
// readM4Enc е премахнат

// ======================== ЧЕТЕНЕ НА БУТОНИ ========================
uint8_t readButtons() {
    Wire.requestFrom(PCF8574_ADDR, 1);
    if (Wire.available()) return ~Wire.read();
    return 0;
}

// ======================== СТОП ФУНКЦИИ ========================
void stopRobot() {
    ledcWrite(0, 0); ledcWrite(1, 0);
    digitalWrite(Pin_Motor_1_IN1, LOW); digitalWrite(Pin_Motor_1_IN2, LOW);
    digitalWrite(Pin_Motor_2_IN1, LOW); digitalWrite(Pin_Motor_2_IN2, LOW);
}

void stopAttachment() {
    ledcWrite(2, 0); ledcWrite(3, 0);
    digitalWrite(Pin_Motor_3_IN1, LOW); digitalWrite(Pin_Motor_3_IN2, LOW);
    digitalWrite(Pin_Motor_4_IN1, LOW); digitalWrite(Pin_Motor_4_IN2, LOW);
}

bool checkStopDelay(int ms) {
    if (stopMissionFlag) return true;
    unsigned long start = millis();
    while (millis() - start < ms) {
        ArduinoOTA.handle();
        uint8_t btns = readButtons();
        // Стоп само ако е натиснат P7 (бит 7) БЕЗ стартовия бутон (бит 3)
        if ((btns & (1 << 7)) && !(btns & (1 << 3))) {
            stopMissionFlag = true;
            stopRobot();
            stopAttachment();
            return true;
        }
        delay(5);
    }
    return stopMissionFlag;
}

// ======================== ЖИРОСКОП ПОМОЩНИ ========================
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

// ======================== ДВИЖЕНИЕ НАПРЕД/НАЗАД ========================
void move(float distanceCm, int speed, char direction) {
    if (stopMissionFlag) return;

    leftEncoderCount = 0; rightEncoderCount = 0;
    long target = (long)(countPerCm * distanceCm);
    float startHeading = getHeading();
    float integral = 0.0f, prevError = 0.0f;
    bool forward = (direction == 'F');

    if (forward) {
        digitalWrite(Pin_Motor_1_IN1, LOW);  digitalWrite(Pin_Motor_1_IN2, HIGH);
        digitalWrite(Pin_Motor_2_IN1, LOW);  digitalWrite(Pin_Motor_2_IN2, HIGH);
    } else {
        digitalWrite(Pin_Motor_1_IN1, HIGH); digitalWrite(Pin_Motor_1_IN2, LOW);
        digitalWrite(Pin_Motor_2_IN1, HIGH); digitalWrite(Pin_Motor_2_IN2, LOW);
    }

    unsigned long lastTime = micros();
    while (true) {
        ArduinoOTA.handle();
        { uint8_t b = readButtons(); if ((b & (1<<7)) && !(b & (1<<3))) { stopMissionFlag = true; stopRobot(); return; } }

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

void moveBack(float distanceCm, int speed) {
    if (stopMissionFlag) return;

    leftEncoderCount = 0; rightEncoderCount = 0;
    long target = (long)(countPerCm * distanceCm);
    float startHeading = getHeading();
    float integral = 0.0f, prevError = 0.0f;

    digitalWrite(Pin_Motor_1_IN1, HIGH); digitalWrite(Pin_Motor_1_IN2, LOW);
    digitalWrite(Pin_Motor_2_IN1, HIGH); digitalWrite(Pin_Motor_2_IN2, LOW);

    unsigned long lastTime = micros();
    while (true) {
        ArduinoOTA.handle();
        { uint8_t b = readButtons(); if ((b & (1<<7)) && !(b & (1<<3))) { stopMissionFlag = true; stopRobot(); return; } }

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

// ======================== ЗАВОИ ========================
void turn(float angle, int speed) {
    if (stopMissionFlag) return;

    float lastHeading = getHeading();
    float angleTurned = 0.0f;
    float slowZone = 25.0f, stopZone = 1.5f, prevError = 0.0f;

    if (angle > 0) {
        digitalWrite(Pin_Motor_1_IN1, HIGH); digitalWrite(Pin_Motor_1_IN2, LOW);
        digitalWrite(Pin_Motor_2_IN1, LOW);  digitalWrite(Pin_Motor_2_IN2, HIGH);
    } else {
        digitalWrite(Pin_Motor_1_IN1, LOW);  digitalWrite(Pin_Motor_1_IN2, HIGH);
        digitalWrite(Pin_Motor_2_IN1, HIGH); digitalWrite(Pin_Motor_2_IN2, LOW);
    }

    unsigned long lastTime = micros();
    while (true) {
        ArduinoOTA.handle();
        { uint8_t b = readButtons(); if ((b & (1<<7)) && !(b & (1<<3))) { stopMissionFlag = true; stopRobot(); return; } }

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
    stopRobot(); checkStopDelay(50);
}

// ======================== ПРИКАЧЕН ИНВЕНТАР ========================
void m3Forward(int speed, int timeMs) {
    if (stopMissionFlag) return;
    digitalWrite(Pin_Motor_3_IN1, LOW); digitalWrite(Pin_Motor_3_IN2, HIGH);
    ledcWrite(2, speed); checkStopDelay(timeMs); stopAttachment();
}
void m3Back(int speed, int timeMs) {
    if (stopMissionFlag) return;
    digitalWrite(Pin_Motor_3_IN1, HIGH); digitalWrite(Pin_Motor_3_IN2, LOW);
    ledcWrite(2, speed); checkStopDelay(timeMs); stopAttachment();
}
void m4Forward(int speed, int timeMs) {
    if (stopMissionFlag) return;
    digitalWrite(Pin_Motor_4_IN1, LOW); digitalWrite(Pin_Motor_4_IN2, HIGH);
    ledcWrite(3, speed); checkStopDelay(timeMs); stopAttachment();
}
void m4Back(int speed, int timeMs) {
    if (stopMissionFlag) return;
    digitalWrite(Pin_Motor_4_IN1, HIGH); digitalWrite(Pin_Motor_4_IN2, LOW);
    ledcWrite(3, speed); checkStopDelay(timeMs); stopAttachment();
}

void resetEncoders() {
    leftEncoderCount = 0; rightEncoderCount = 0;
    m3EncoderCount = 0;
}

// ======================== МИСИИ ========================
void run1() {
    resetEncoders(); Serial.println("СТАРТ: Run 1 - Osnoven");
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
    resetEncoders(); Serial.println("СТАРТ: Run 2 - Misiq 2");
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
    resetEncoders(); Serial.println("СТАРТ: Run 3 - Cana Pernik");
    move(8, 100, 'F');
    moveBack(10, 100);
}

void run4() {
    resetEncoders(); Serial.println("СТАРТ: Run 4 - Kubce av");
    turn(8, 70);
    move(21, 100, 'F');
    moveBack(27, 100);
}

void run5() {
    resetEncoders(); Serial.println("СТАРТ: Run 5 - Vzemi casata");
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
    resetEncoders(); Serial.println("СТАРТ: Run 6 - Musala");
    turn(-35, 70);
    move(2, 100, 'F');
    moveBack(8, 100);
}

void run8() {
    resetEncoders(); Serial.println("СТАРТ: Run 8 - Ostani casa");
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

// ======================== ИЗПЪЛНЕНИЕ НА МИСИЯ ПО НОМЕР ========================
void executeRun(int missionNumber, int speedLevel) {
    stopMissionFlag = false;

    Serial.printf("\n>>> СТАРТ НА МИСИЯ %d (Скорост ниво: %d) <<<\n", missionNumber, speedLevel);

    // Тук свързваме числата от матрицата с реалните мисии.
    // Разширете switch-а когато добавяте нови мисии!
    switch (missionNumber) {
        case 1:  run1(); break;
        case 2:  run2(); break;
        case 3:  run3(); break;
        case 4:  run4(); break;
        case 5:  run5(); break;
        case 6:  run6(); break;
        case 7:  run8(); break;
        // Добавете тук нови case-ове за числата в матрицата
        default:
            Serial.printf("Мисия %d не е дефинирана!\n", missionNumber);
            break;
    }

    if (stopMissionFlag) {
        Serial.println("ПРЕКЪСНАТО / ABORTED!");
    } else {
        Serial.println(">>> КРАЙ НА МИСИЯТА <<<\n");
    }

    stopMissionFlag = false;
    stopRobot();
    stopAttachment();
}

// ======================== SETUP ========================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n--- MEHANO ROBOT BOOT ---");

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
    pinMode(ENC_M3_C1, INPUT); pinMode(ENC_M3_C2, INPUT);
    // ENC_M4 е изключен — пинове 34/35 са за потенциометрите

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
    Serial.println("=== ГОТОВО | Завъртете потенциометрите и натиснете Бутон 1 ===");
}

// ======================== MAIN LOOP ========================
void loop() {
    ArduinoOTA.handle();

    // --- Четем потенциометрите ---
    int raw1 = analogRead(POT_1_PIN);
    int raw2 = analogRead(POT_2_PIN);

    // Индекс за Пот 1 (избор на мисия, колона 0..4)
    int pot1_index = 0;
    if      (raw1 < 300)  pot1_index = 0;
    else if (raw1 < 1300) pot1_index = 1;
    else if (raw1 < 2100) pot1_index = 2;
    else if (raw1 < 3000) pot1_index = 3;
    else                  pot1_index = 4;

    // Индекс за Пот 2 (скорост, ред 0..3)
    int pot2_index = 0;
    if      (raw2 < 250)  pot2_index = 0;
    else if (raw2 < 1000) pot2_index = 1;
    else if (raw2 < 1600) pot2_index = 2;
    else                  pot2_index = 3;

    // Крайното число от матрицата
    int finalNumber = missionMatrix[pot2_index][pot1_index];

    // --- Четем бутоните ---
    uint8_t btns = readButtons();

    // Бутон 1 (P4) -> СТАРТ на избраната мисия
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
