/*
 * ======================================================================================
 * PROYEK: ROBOT LINE FOLLOWER - VERSI STABIL & TANGGUH
 * ARSITEKTUR: Arduino Uno + L293D Motor Shield (AFMotor) + 8 Sensor Array
 * 
 * FITUR UTAMA:
 *  - PID Controller dengan Anti-Windup dan Dynamic Speed Adjustment
 *  - Toleransi Garis Putus-Putus (Broken/Dashed Line)
 *  - Deteksi & Handling Simpang/Junction dengan Sensor Sayap
 *  - Deteksi Dead-End dengan Smart U-Turn (tidak kembali ke arah semula)
 *  - Belok Tajam/90° dan Belok Melengkung menggunakan Sensor Sayap
 *  - Failsafe Recovery System
 *
 * PEMETAAN KABEL (WIRING MAPPING):
 *  - Sensor 1-6 (Analog):  Arduino Pin A0-A5 (Array Utama)
 *  - Sensor 7 (Digital):   Arduino Pin D9  (Sayap Kiri)
 *  - Sensor 8 (Digital):   Arduino Pin D10 (Sayap Kanan)
 *  - Motor Kiri:           Terminal M3 pada Shield L293D
 *  - Motor Kanan:          Terminal M4 pada Shield L293D
 *
 * CATATAN TUNING:
 *  - threshold[]: Sesuaikan dengan sensor dan lintasan Anda
 *  - Kp/Ki/Kd: Tune untuk responsivitas dan stabilitas optimal
 *  - LINE_LOST_DELAY: Naikkan untuk garis putus-putus lebih panjang
 *  - DEAD_END_TIMEOUT: Sesuaikan untuk deteksi buntu yang akurat
 *  - spinTime di handleDeadEnd(): Kalibrasi untuk putar ~160-200°
 * ======================================================================================
 */

#include <AFMotor.h>

// =========================== KONFIGURASI & TUNING ===========================
const int MAX_SPEED      = 200;
const int BASE_SPEED     = 150;
const int MIN_BASE_SPEED = 50;
const int TURN_SPEED     = 200;

float Kp = 0.045;
float Ki = 0.0004;   // kecil agar tidak overshoot
float Kd = 7.0;

const int DEADBAND       = 15;
const int BRAKE_FACTOR   = 90;
const int LINE_LOST_DELAY = 140;    // toleransi putus-putus (ms)
const int SHARP_ERROR     = 1300;
const float SHARP_MULTIPLIER = 1.9;

const int DEAD_END_TIMEOUT = 450;   // ms: kalau hilang garis > ini, anggap buntu
const int JUNCTION_COUNT_MIN = 2;   // jumlah sensor aktif minimal utk deteksi simpang

// =========================== HARDWARE ===========================
AF_DCMotor motorLeft(3);
AF_DCMotor motorRight(4);

const int SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5};
const int NUM_SENSORS = 6;
const int S7_SAYAP = 9;   // sensor sayap kiri
const int S8_SAYAP = 10;  // sensor sayap kanan

// Threshold sensor (sesuaikan)
int threshold[] = {560, 570, 511, 517, 542, 517};

// =========================== VARIABEL ===========================
float lastError = 0;
float integral  = 0;
int lastDirection = 1; // 1: kanan, -1: kiri
unsigned long lastLineTime = 0;
unsigned long lastSeenTime = 0;

// =========================== SETUP ===========================
void setup() {
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);

  pinMode(S7_SAYAP, INPUT);
  pinMode(S8_SAYAP, INPUT);

  pinMode(13, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(13, HIGH); delay(80);
    digitalWrite(13, LOW);  delay(80);
  }
}

// =========================== LOOP ===========================
void loop() {
  int activeCount = 0;
  int position = readLinePosition(activeCount);

  // ================= PUTUS-PUTUS & KEHILANGAN =================
  if (position == -1) {
    unsigned long now = millis();

    // toleransi putus-putus
    if (now - lastLineTime < LINE_LOST_DELAY) {
      setMotorSpeed(BASE_SPEED - 25, BASE_SPEED - 25);
      return;
    }

    // deteksi dead-end / buntu
    if (now - lastSeenTime > DEAD_END_TIMEOUT) {
      handleDeadEnd();   // mutar cari jalan lain
      return;
    }

    // fallback ke failsafe biasa
    handleFailsafe();
    return;
  } else {
    lastLineTime = millis();
    lastSeenTime = millis();
  }

  // ================= DETEKSI SIMPANG & BELAKU =================
  if (activeCount >= JUNCTION_COUNT_MIN + 1) {
    handleJunction(activeCount);
    return;
  }

  // ================= PID =================
  int setPoint = 2500;
  int error = position - setPoint;

  if (abs(error) <= DEADBAND) {
    error = 0;
  }

  float P = error;
  integral += error;
  integral = constrain(integral, -8000, 8000);  // anti windup
  float D = error - lastError;

  float multiplier = (abs(error) > SHARP_ERROR) ? SHARP_MULTIPLIER : 1.0;
  float correction = (Kp * P + Ki * integral + Kd * D) * multiplier;
  lastError = error;

  // ================= KECEPATAN DINAMIS =================
  int dynamicBase = BASE_SPEED - (abs(error) / 400.0 * BRAKE_FACTOR);
  dynamicBase = constrain(dynamicBase, MIN_BASE_SPEED, BASE_SPEED);

  int leftSpeed  = dynamicBase + correction;
  int rightSpeed = dynamicBase - correction;

  // ================= BELAKU MELENGKUNG / TAJAM =================
  // jika sayap mendeteksi, bantu belok
  if (digitalRead(S7_SAYAP) == HIGH) { // sayap kiri kena garis: belok kiri
    leftSpeed  -= 40;
    rightSpeed += 40;
  }
  if (digitalRead(S8_SAYAP) == HIGH) { // sayap kanan kena garis: belok kanan
    leftSpeed  += 40;
    rightSpeed -= 40;
  }

  setMotorSpeed(leftSpeed, rightSpeed);
}

// =========================== FUNGSI SENSOR ===========================
int readLinePosition(int &activeCount) {
  long weightedSum = 0;
  long sum = 0;
  bool onLine = false;

  bool extremeLeftActive = false;
  bool extremeRightActive = false;

  activeCount = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = analogRead(SENSOR_PINS[i]);
    if (val > threshold[i]) {
      weightedSum += i * 1000;
      sum++;
      onLine = true;
      activeCount++;
      if (i == 0) extremeLeftActive = true;
      if (i == NUM_SENSORS - 1) extremeRightActive = true;
    }
  }

  // LATCH ARAH TERAKHIR
  if (extremeLeftActive && !extremeRightActive) {
    lastDirection = -1;
  } else if (extremeRightActive && !extremeLeftActive) {
    lastDirection = 1;
  }

  if (!onLine) return -1;
  return weightedSum / sum;
}

// =========================== HANDLER SIMPANG & BUNTU ===========================
void handleJunction(int activeCount) {
  // Simpang (banyak sensor aktif): pilih arah sesuai memori terakhir
  // Jika sayap mendeteksi, prioritaskan sayap
  if (digitalRead(S7_SAYAP) == HIGH) {
    setMotorSpeed(-TURN_SPEED, TURN_SPEED); // belok kiri
    lastDirection = -1;
  } else if (digitalRead(S8_SAYAP) == HIGH) {
    setMotorSpeed(TURN_SPEED, -TURN_SPEED); // belok kanan
    lastDirection = 1;
  } else {
    // default: ikuti arah terakhir
    if (lastDirection == -1) {
      setMotorSpeed(-TURN_SPEED, TURN_SPEED);
    } else {
      setMotorSpeed(TURN_SPEED, -TURN_SPEED);
    }
  }
  delay(120); // pendek, cukup untuk masuk jalur baru
}

void handleDeadEnd() {
  // Dead-end: putar 160-200 derajat ke arah berlawanan dari lastDirection
  int spinTime = 280; // sesuaikan dengan kecepatan & gear
  if (lastDirection == 1) {
    setMotorSpeed(-TURN_SPEED, TURN_SPEED); // putar kiri
  } else {
    setMotorSpeed(TURN_SPEED, -TURN_SPEED); // putar kanan
  }
  delay(spinTime);
}

// =========================== FAILSAFE PUTUS GARIS ===========================
void handleFailsafe() {
  // Prioritas sayap
  if (digitalRead(S7_SAYAP) == HIGH) {
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
    lastDirection = -1;
    return;
  }
  if (digitalRead(S8_SAYAP) == HIGH) {
    setMotorSpeed(TURN_SPEED, -TURN_SPEED);
    lastDirection = 1;
    return;
  }

  // Memori arah terakhir
  if (lastDirection == -1) {
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
  } else {
    setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  }
}

// =========================== MOTOR ===========================
void setMotorSpeed(int left, int right) {
  left  = constrain(left,  -MAX_SPEED, MAX_SPEED);
  right = constrain(right, -MAX_SPEED, MAX_SPEED);

  if (left >= 0) {
    motorLeft.run(FORWARD);
    motorLeft.setSpeed(left);
  } else {
    motorLeft.run(BACKWARD);
    motorLeft.setSpeed(-left);
  }

  if (right >= 0) {
    motorRight.run(FORWARD);
    motorRight.setSpeed(right);
  } else {
    motorRight.run(BACKWARD);
    motorRight.setSpeed(-right);
  }
}