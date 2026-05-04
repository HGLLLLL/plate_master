#include <Arduino.h>
#include <PS4Controller.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// ================= 腳位定義 (保持不變) =================
#define ENA_RF 13
#define IN1_RF 14
#define IN2_RF 12
#define ENB_RR 27
#define IN3_RR 25
#define IN4_RR 26

#define ENA_LF 17
#define IN1_LF 4
#define IN2_LF 16
#define ENB_LR 19
#define IN3_LR 18
#define IN4_LR 5

#define ENCA_RF 32
#define ENCB_RF 34
#define ENCA_RR 33
#define ENCB_RR 35
#define ENCA_LF 21
#define ENCB_LF 36
#define ENCA_LR 22
#define ENCB_LR 39

// ================= 系統參數 =================
const int basePPR = 11;         
const float gearRatio = 30.0;   
const float totalPPR = basePPR * gearRatio; 

volatile long countLF = 0, countLR = 0, countRF = 0, countRR = 0;
volatile unsigned long lastMicrosLF = 0, lastMicrosLR = 0, lastMicrosRF = 0, lastMicrosRR = 0;
const unsigned long DEBOUNCE_US = 150;

long prevCountLF = 0, prevCountLR = 0, prevCountRF = 0, prevCountRR = 0;
unsigned long previousMillis = 0;
const int reportInterval = 50; 
const float MAX_RPM = 230.0; 

// ================= 搖桿曲線與靈敏度死區 =================
const float K1_X = 0.15, K1_Y = 0.50;   
const float K2_X = 0.80, K2_Y = 0.50;   
const int DEADZONE_PS4 = 15; 
const float STEER_SENSITIVITY = 0.4; 
const float THROTTLE_SENSITIVITY = 0.8;

// ================= PID 控制器結構 =================
struct MotorPID {
  float kp, ki, kd;
  volatile float setpoint;
  float integral, prevRPM, filteredRPM;
};

MotorPID pidLF = {0.7, 0.4, 1.5, 0, 0, 0, 0};
MotorPID pidLR = {0.7, 0.4, 1.5, 0, 0, 0, 0};
MotorPID pidRF = {0.7, 0.4, 1.5, 0, 0, 0, 0};
MotorPID pidRR = {0.7, 0.4, 1.5, 0, 0, 0, 0};

// --- 函式宣告 ---
void setTargetRPM(float leftRPM, float rightRPM);
int computePID(MotorPID &pid, float currentRPM);
void applyMotorPWM(int pwmLF, int pwmLR, int pwmRF, int pwmRR);
float applyUserCurve(float x);

// ISR
void IRAM_ATTR isrLF() {
  unsigned long now = micros();
  if (now - lastMicrosLF > DEBOUNCE_US) {
    if (digitalRead(ENCB_LF) == HIGH) countLF++; else countLF--;
    lastMicrosLF = now;
  }
}
void IRAM_ATTR isrLR() {
  unsigned long now = micros();
  if (now - lastMicrosLR > DEBOUNCE_US) {
    if (digitalRead(ENCB_LR) == HIGH) countLR++; else countLR--;
    lastMicrosLR = now;
  }
}
void IRAM_ATTR isrRF() {
  unsigned long now = micros();
  if (now - lastMicrosRF > DEBOUNCE_US) {
    if (digitalRead(ENCB_RF) == HIGH) countRF++; else countRF--;
    lastMicrosRF = now;
  }
}
void IRAM_ATTR isrRR() {
  unsigned long now = micros();
  if (now - lastMicrosRR > DEBOUNCE_US) {
    if (digitalRead(ENCB_RR) == HIGH) countRR++; else countRR--;
    lastMicrosRR = now;
  }
}

// ================= 搖桿曲線 =================
float applyUserCurve(float x) {
    if (x <= K1_X) return (K1_Y / K1_X) * x;
    else if (x <= K2_X) return K1_Y + (K2_Y - K1_Y) / (K2_X - K1_X) * (x - K1_X);
    else return K2_Y + (1.0 - K2_Y) / (1.0 - K2_X) * (x - K2_X);
}

// ================= 任務：PS4 控制處理 (核心 0) =================
void Task_Input(void *pvParameters) {
    // 伺服馬達按鍵
    bool lastBtnUp = false, lastBtnDown = false;
    // 風扇按鍵
    bool lastBtnSquare = false, lastBtnTriangle = false, lastBtnCross = false;

    for (;;) {
        if (PS4.isConnected()) {
            // --- 1. 底盤動力控制 ---
            int rawY = PS4.LStickY(); 
            int rawX = PS4.RStickX();

            float normT = (float)rawY / 128.0;
            float normS = (float)rawX / 128.0;

            float absT = fabsf(normT);
            float absS = fabsf(normS);
            float dz_float = (float)DEADZONE_PS4 / 128.0;
            
            float finalAbsT = (absT < dz_float) ? 0 : applyUserCurve(absT);
            float finalAbsS = (absS < dz_float) ? 0 : applyUserCurve(absS);

            // 套用油門與轉向靈敏度
            float finalT = ((normT >= 0) ? finalAbsT : -finalAbsT) * THROTTLE_SENSITIVITY;
            float finalS = ((normS >= 0) ? finalAbsS : -finalAbsS) * STEER_SENSITIVITY;

            float leftPower = finalT + finalS;
            float rightPower = finalT - finalS;

            float maxVal = fmaxf(1.0f, fmaxf(fabsf(leftPower), fabsf(rightPower)));
            setTargetRPM((leftPower / maxVal) * MAX_RPM, (rightPower / maxVal) * MAX_RPM);

            // --- 2. 伺服馬達控制 (上/下 鍵) ---
            bool currUp = PS4.Up();
            bool currDown = PS4.Down();

            if (currUp && !lastBtnUp) {
                Serial2.println("S:260");
                // Serial.println(">> 伺服馬達: 260 度"); 
            }
            if (currDown && !lastBtnDown) {
                Serial2.println("S:0");
                // Serial.println(">> 伺服馬達: 0 度");
            }
            lastBtnUp = currUp; lastBtnDown = currDown;

            // --- 3. 函道風扇控制 (形狀鍵) ---
            bool currSquare = PS4.Square();
            bool currTriangle = PS4.Triangle();
            bool currCross = PS4.Cross();

            if (currSquare && !lastBtnSquare) {
                Serial2.println("F:E");
            }
            if (currTriangle && !lastBtnTriangle) {
                Serial2.println("F:R");
            }
            if (currCross && !lastBtnCross) {
                Serial2.println("F:S");
            }
            lastBtnSquare = currSquare; lastBtnTriangle = currTriangle; lastBtnCross = currCross;

        } else {
            setTargetRPM(0, 0); 
        }
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

// ================= 初始化 =================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 15, 23);

  const int motorPins[] = {ENA_LF, IN1_LF, IN2_LF, ENB_LR, IN3_LR, IN4_LR, ENA_RF, IN1_RF, IN2_RF, ENB_RR, IN3_RR, IN4_RR};
  for (int i = 0; i < 12; i++) { pinMode(motorPins[i], OUTPUT); digitalWrite(motorPins[i], LOW); }
  
  pinMode(ENCA_LF, INPUT_PULLUP); pinMode(ENCB_LF, INPUT);
  pinMode(ENCA_LR, INPUT_PULLUP); pinMode(ENCB_LR, INPUT);
  pinMode(ENCA_RF, INPUT_PULLUP); pinMode(ENCB_RF, INPUT);
  pinMode(ENCA_RR, INPUT_PULLUP); pinMode(ENCB_RR, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_LF), isrLF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_LR), isrLR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RF), isrRF, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_RR), isrRR, RISING);

  PS4.begin("70:4B:CA:57:BA:6A"); 
  Serial.println("PS4 Controller Listening...");

  xTaskCreatePinnedToCore(Task_Input, "PS4InputTask", 4096, NULL, 2, NULL, 0);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= reportInterval) {
    noInterrupts();
    long dLF = countLF - prevCountLF; long dLR = countLR - prevCountLR;
    long dRF = countRF - prevCountRF; long dRR = countRR - prevCountRR;
    prevCountLF = countLF; prevCountLR = countLR; prevCountRF = countRF; prevCountRR = countRR;
    interrupts();

    float mult = (60000.0 / reportInterval) / totalPPR;
    float rpmLF = -dLF * mult; float rpmLR = -dLR * mult;
    float rpmRF = dRF * mult;  float rpmRR = dRR * mult;

    if (abs(rpmLF) > 500) rpmLF = pidLF.filteredRPM;
    if (abs(rpmLR) > 500) rpmLR = pidLR.filteredRPM;
    if (abs(rpmRF) > 500) rpmRF = pidRF.filteredRPM;
    if (abs(rpmRR) > 500) rpmRR = pidRR.filteredRPM;

    applyMotorPWM(computePID(pidLF, rpmLF), computePID(pidLR, rpmLR), computePID(pidRF, rpmRF), computePID(pidRR, rpmRR));
    previousMillis = currentMillis;

    // ================= Serial Plotter =================
    Serial.printf("LF_RPM:%.2f, LR_RPM:%.2f, RF_RPM:%.2f, RR_RPM:%.2f\n", 
                  pidLF.filteredRPM, pidLR.filteredRPM, pidRF.filteredRPM, pidRR.filteredRPM);
  }
}

// ================= PID 運算 =================
int computePID(MotorPID &pid, float currentRPM) {
  pid.filteredRPM = (0.8 * pid.filteredRPM) + (0.2 * currentRPM); 

  if (pid.setpoint == 0) { 
    pid.integral = 0; 
    pid.prevRPM = pid.filteredRPM; 
    return 0; 
  }

  float error = pid.setpoint - pid.filteredRPM;
  float dTerm = pid.kd * (pid.prevRPM - pid.filteredRPM);

  float testOutput = (pid.kp * error) + (pid.ki * pid.integral) + dTerm;
  
  bool isSaturated = (testOutput >= 255 && error > 0) || (testOutput <= -255 && error < 0);
  if (!isSaturated) {
    pid.integral += error;
  }
  
  pid.integral *= 0.98; 

  float output = (pid.kp * error) + (pid.ki * pid.integral) + dTerm;
  pid.prevRPM = pid.filteredRPM;
  
  return constrain((int)output, -255, 255);
}

void setTargetRPM(float leftRPM, float rightRPM) {
  pidLF.setpoint = leftRPM; pidLR.setpoint = leftRPM;
  pidRF.setpoint = rightRPM; pidRR.setpoint = rightRPM;
}

void applyMotorPWM(int pwmLF, int pwmLR, int pwmRF, int pwmRR) {
  digitalWrite(IN1_LF, pwmLF > 0 ? HIGH : LOW); digitalWrite(IN2_LF, pwmLF < 0 ? HIGH : LOW); analogWrite(ENA_LF, abs(pwmLF));
  digitalWrite(IN3_LR, pwmLR > 0 ? HIGH : LOW); digitalWrite(IN4_LR, pwmLR < 0 ? HIGH : LOW); analogWrite(ENB_LR, abs(pwmLR));
  digitalWrite(IN1_RF, pwmRF > 0 ? HIGH : LOW); digitalWrite(IN2_RF, pwmRF < 0 ? HIGH : LOW); analogWrite(ENA_RF, abs(pwmRF));
  digitalWrite(IN3_RR, pwmRR > 0 ? HIGH : LOW); digitalWrite(IN4_RR, pwmRR < 0 ? HIGH : LOW); analogWrite(ENB_RR, abs(pwmRR));
}