/*
 * ======================================================================================
 * PROYEK: ROBOT LINE FOLLOWER STANDAR KOMPETISI INTERNASIONAL
 * ARSITEKTUR: Arduino Uno + L293D Motor Shield + 8 Channel Hybrid Sensor Array
 * PENULIS:
 * TANGGAL: 2023-10-27
 *
 * FITUR UTAMA:
 *  - Fusi Sensor Hibrida (6 Centroid Analog + 2 Digital Marker/Recovery)
 *  - PID Lanjutan dengan Anti-Windup, Clamping, dan Filter Derivatif Low-Pass
 *  - Kalibrasi Lingkungan Otomatis (Normalisasi Min/Max Per-Channel)
 *  - Kompensasi Deadband Dinamis (Linearisasi Respons Motor)
 *  - Failsafe Pemulihan Garis (Backtracking berbasis Memori)
 *  - Arsitektur State Machine Modular
 *
 * PEMETAAN KABEL (WIRING MAPPING):
 *  - Sensor 1 (Analog):    Arduino Pin A0 (Paling Kiri)
 *  - Sensor 2 (Analog):    Arduino Pin A1
 *  - Sensor 3 (Analog):    Arduino Pin A2
 *  - Sensor 4 (Analog):    Arduino Pin A3
 *  - Sensor 5 (Analog):    Arduino Pin A4
 *  - Sensor 6 (Analog):    Arduino Pin A5
 *  - Sensor 7 (Digital):   Arduino Pin D9 (Sayap Kanan Dalam)
 *  - Sensor 8 (Digital):   Arduino Pin D10 (Sayap Kanan Luar)
 *  - Motor Kiri:           Terminal M3 pada Shield L293D
 *  - Motor Kanan:          Terminal M4 pada Shield L293D
 * ======================================================================================
 */

#include <AFMotor.h>

// ======================================================================================
// 1. KONFIGURASI SISTEM & KONSTANTA
// ======================================================================================

// --- KONSTANTA GAIN PID (PERLU TUNING) ---
// Nilai ini menentukan "kepribadian" robot.
// Kp: Besarnya reaksi. Terlalu tinggi = osilasi. Terlalu rendah = lamban.
// Ki: Koreksi error steady-state. Jaga tetap RENDAH untuk menghindari windup.
// Kd: Peredaman. Memprediksi error masa depan untuk memperhalus gerakan.
float Kp = 0.18;    // Rentang Awal: 0.10 - 0.25
float Ki = 0.0008;  // Rentang Awal: 0.0001 - 0.001
float Kd = 2.5;     // Rentang Awal: 1.5 - 4.0 (L293D butuh Kd tinggi karena lag)

// --- PENGATURAN KECEPATAN MOTOR (0-255) ---
// Sesuaikan berdasarkan tegangan baterai (misal: 2S LiPo vs 9V block)
const int BASE_SPEED = 160;       // Kecepatan nominal garis lurus
const int MAX_SPEED = 255;        // Batas keras perangkat keras
const int TURN_SPEED_LIMIT = 200; // Batas kecepatan saat berbelok tajam
const int DEADBAND_OFFSET = 60;   // PWM minimum untuk mengatasi gesekan (Spesifik Motor)

// --- KONFIGURASI SENSOR ---
const int NUM_SENSORS_ANALOG = 6;
const int NUM_SENSORS_TOTAL = 8;
const int SENSOR_THRESHOLD = 500; // Ambang batas logika digital (skala 0-1000)

// --- DEFINISI PIN ---
// Catatan: Pin A0-A5 adalah Analog asli. D9/D10 digunakan sebagai Input Digital.
const int pinSensorsAnalog = {A0, A1, A2, A3, A4, A5};
const int PIN_SENSOR_7 = 9;   // Digital Extension
const int PIN_SENSOR_8 = 10;  // Digital Extension

// --- OBJEK MOTOR (Library AFMotor) ---
// Menggunakan M3 untuk Kiri dan M4 untuk Kanan (Standar wiring chassis 4WD/2WD)
AF_DCMotor motorLeft(3);
AF_DCMotor motorRight(4);

// ======================================================================================
// 2. VARIABEL GLOBAL & MANAJEMEN STATE
// ======================================================================================

// Array Kalibrasi
int minSensorValues;
int maxSensorValues;

// Variabel Memori PID
float lastError = 0;
float integral = 0;
float derivativeFilter = 0;

// Memori Failsafe
int lastKnownDirection = 0; // -1 = Kiri, 1 = Kanan
bool onLine = false;

// ======================================================================================
// 3. RUTIN SETUP & KALIBRASI
// ======================================================================================

void setup() {
  Serial.begin(115200); // Telemetri kecepatan tinggi
  Serial.println("--- SYSTEM BOOT: LINE FOLLOWER INIT ---");

  // Inisialisasi Pin Digital
  pinMode(PIN_SENSOR_7, INPUT);
  pinMode(PIN_SENSOR_8, INPUT);
  // Pin analog tidak memerlukan inisialisasi pinMode INPUT pada arsitektur AVR

  // Inisialisasi Shield Motor
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);

  // --- URUTAN AUTO-KALIBRASI ---
  // Robot akan berputar untuk memindai garis hitam dan lantai putih
  calibrateSensors();
}

/**
 * Memutar robot di tempat untuk memaparkan sensor ke permukaan Hitam dan Putih.
 * Merekam rentang dinamis untuk normalisasi data yang akurat.
 */
void calibrateSensors() {
  Serial.println("Status: CALIBRATING...");

  // Inisialisasi Min/Max dengan nilai invers
  for (int i = 0; i < NUM_SENSORS_ANALOG; i++) {
    minSensorValues[i] = 1023; // Mulai dengan max yang mungkin
    maxSensorValues[i] = 0;    // Mulai dengan min yang mungkin
  }

  // Logika Putaran: Putar Kanan lalu Kiri untuk memindai garis
  // Catatan: Sesuaikan kecepatan jika robot berputar terlalu cepat/lambat
  int calSpeed = 130;
  motorLeft.setSpeed(calSpeed);
  motorRight.setSpeed(calSpeed);
  
  unsigned long startTime = millis();
  
  // Fase 1: Putar Kanan (1.5 detik)
  motorLeft.run(FORWARD);
  motorRight.run(BACKWARD);
  while (millis() - startTime < 1500) {
    scanSensorsMinMax();
  }

  // Fase 2: Putar Kiri (1.5 detik) - Kembali ke tengah
  motorLeft.run(BACKWARD);
  motorRight.run(FORWARD);
  while (millis() - startTime < 3000) {
    scanSensorsMinMax();
  }

  // Hentikan Motor
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
  
  Serial.println("Status: CALIBRATION COMPLETE");
  
  // Output Debug Data Kalibrasi (Opsional)
  for (int i = 0; i < NUM_SENSORS_ANALOG; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(": "); Serial.print(minSensorValues[i]);
    Serial.print("-"); Serial.print(maxSensorValues[i]);
    Serial.print(" | ");
  }
  Serial.println();
  delay(2000); // Jeda aman sebelum balapan dimulai
}

/**
 * Helper function untuk membaca dan memperbarui nilai min/max sensor
 */
void scanSensorsMinMax() {
  for (int i = 0; i < NUM_SENSORS_ANALOG; i++) {
    int val = analogRead(pinSensorsAnalog[i]);
    // Perbarui Min
    if (val < minSensorValues[i]) minSensorValues[i] = val;
    // Perbarui Max
    if (val > maxSensorValues[i]) maxSensorValues[i] = val;
  }
}

// ======================================================================================
// 4. LOOP UTAMA (CORE LOOP)
// ======================================================================================

void loop() {
  // 1. BACA SENSOR & HITUNG POSISI
  // Membaca nilai analog, menormalisasi, dan menggabungkan dengan input digital
  int position = readLinePosition();
  
  // 2. PEMILIHAN LOGIKA KONTROL
  if (onLine) {
    // Operasi Normal: Garis terdeteksi, jalankan PID
    computePID(position);
  } else {
    // Failsafe: Garis Hilang -> Jalankan pemulihan berbasis memori
    executeRecovery();
  }
}

// ======================================================================================
// 5. ALGORITMA FUSI SENSOR
// ======================================================================================

/**
 * Membaca 6 sensor Analog dan 2 sensor Digital.
 * Menghitung Rata-rata Tertimbang (Centroid) untuk presisi sub-piksel.
 * 
 * Returns: Nilai posisi dari 0 (Kiri) hingga 7000 (Kanan). Titik tengah ideal ~3500.
 * Mengupdate status Global 'onLine' dan 'lastKnownDirection'.
 */
int readLinePosition() {
  long weightedSum = 0;
  long sum = 0;
  bool lineFoundAny = false;

  // --- PROSES SENSOR ANALOG (0-5) ---
  for (int i = 0; i < NUM_SENSORS_ANALOG; i++) {
    int raw = analogRead(pinSensorsAnalog[i]);
    
    // Normalisasi ke 0-1000 menggunakan data kalibrasi
    // Rumus: (raw - min) * 1000 / (max - min)
    int denominator = maxSensorValues[i] - minSensorValues[i];
    if (denominator <= 0) denominator = 1; // Cegah pembagian dengan nol
    
    long value = ((long)(raw - minSensorValues[i]) * 1000) / denominator;
    value = constrain(value, 0, 1000);

    // Filter Noise: Abaikan nilai di bawah lantai noise (misal < 50)
    // Ini membantu mempertajam centroid
    if (value < 50) value = 0; 
    
    // Deteksi keberadaan garis
    if (value > 200) lineFoundAny = true;

    // Perhitungan Rata-rata Tertimbang
    // Bobot posisi adalah i * 1000 (0, 1000, 2000,... 5000)
    weightedSum += value * (i * 1000);
    sum += value;
  }

  // --- PROSES SENSOR DIGITAL (6-7) ---
  // Sensor 7 (D9) dan 8 (D10) diperlakukan sebagai bobot ekstrem
  // Asumsi: Sensor memberikan output HIGH saat garis (Hitam) terdeteksi.
  // Jika sensor Active LOW, gunakan!digitalRead(...)
  int val7 = digitalRead(PIN_SENSOR_7) == HIGH? 1000 : 0;
  int val8 = digitalRead(PIN_SENSOR_8) == HIGH? 1000 : 0;

  if (val7 > 0) {
    weightedSum += val7 * 6000;
    sum += val7;
    lineFoundAny = true;
  }
  if (val8 > 0) {
    weightedSum += val8 * 7000;
    sum += val8;
    lineFoundAny = true;
  }

  // --- LOGIKA FAILSAFE ---
  if (!lineFoundAny) {
    onLine = false;
    // Jika garis hilang, kembalikan nilai ekstrem berdasarkan arah terakhir yang diketahui
    // Ini membantu PID 'mendorong' robot kembali ke arah yang benar sebelum masuk mode recovery penuh
    if (lastKnownDirection < 0) return 0;       // Hilang di Kiri, asumsikan di Kiri ekstrem
    else return 7000;                           // Hilang di Kanan, asumsikan di Kanan ekstrem
  }

  // Garis ditemukan
  onLine = true;
  int position = weightedSum / sum;
  
  // Perbarui Memori Arah untuk Recovery
  // Titik tengah array adalah 3500 (antara sensor indeks 3 dan 4)
  if (position < 3500) lastKnownDirection = -1; // Garis ada di Kiri robot
  else if (position > 3500) lastKnownDirection = 1; // Garis ada di Kanan robot
  
  return position;
}

// ======================================================================================
// 6. KONTROLER PID LANJUTAN
// ======================================================================================

void computePID(int inputPosition) {
  // Setpoint adalah titik tengah teoretis dari array sensor (0 hingga 7000)
  int setPoint = 3500; 
  
  // 1. Hitung Error
  int error = inputPosition - setPoint;

  // 2. Term INTEGRAL (I) dengan ANTI-WINDUP
  integral += error;
  
  // Clamping: Batasi akumulasi integral
  // Nilai 10000 dipilih agar max kontribusi I tidak melebihi ~20% kecepatan motor
  integral = constrain(integral, -10000, 10000); 
  
  // Zero-Cross Reset: Jika error melewati nol (robot melewati tengah garis), 
  // reset integral untuk mencegah overshoot akibat "memori" kesalahan lama
  if ((lastError < 0 && error > 0) |

| (lastError > 0 && error < 0)) {
    integral = 0;
  }

  // 3. Term DERIVATIVE (D) dengan LOW PASS FILTER
  float rawDerivative = error - lastError;
  
  // Filter Derivatif: Alpha = 0.7 (70% data baru, 30% data lama)
  // Ini meredam lonjakan "jitter" dari ADC yang bisa memanaskan motor driver
  derivativeFilter = (0.7 * rawDerivative) + (0.3 * derivativeFilter);

  // 4. Hitung Output PID Total
  // Output positif berarti robot terlalu ke Kanan (Error > 0), perlu belok Kiri
  float output = (Kp * error) + (Ki * integral) + (Kd * derivativeFilter);

  // Perbarui Memori Error
  lastError = error;

  // 5. Kirim ke Aktuator
  setMotorSpeed(output);
}

// ======================================================================================
// 7. AKTUASI & DINAMIKA MOTOR
// ======================================================================================

/**
 * Mengubah output PID menjadi sinyal PWM motor dengan kompensasi Deadband.
 * correction: Nilai positif berarti robot perlu belok ke Kiri (kurangi motor Kiri, tambah Kanan)
 */
void setMotorSpeed(float correction) {
  int leftSpeed, rightSpeed;

  // Logika Diferensial Standar
  // Jika correction positif (Robot di Kanan garis), kurangi motor Kiri agar belok Kiri
  // Jika correction negatif (Robot di Kiri garis), kurangi motor Kanan agar belok Kanan
  // Catatan: Tanda +/- mungkin perlu dibalik tergantung polaritas kabel motor Anda
  leftSpeed = BASE_SPEED + correction;
  rightSpeed = BASE_SPEED - correction;

  // --- BATASAN KECEPATAN (CONSTRAINTS) ---
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // --- KOMPENSASI DEADBAND (LINEARISASI) ---
  // Motor DC tidak bergerak pada PWM rendah (misal < 60). Kita memetakan ulang.
  // Jika kecepatan yang diminta > 0 tapi < 60, paksa menjadi 60.
  // Ini membuat respons kontrol menjadi linear dan responsif.
  
  // Motor Kiri
  if (leftSpeed > 0 && leftSpeed < DEADBAND_OFFSET) leftSpeed = DEADBAND_OFFSET;
  else if (leftSpeed < 0 && leftSpeed > -DEADBAND_OFFSET) leftSpeed = -DEADBAND_OFFSET;
  
  // Motor Kanan
  if (rightSpeed > 0 && rightSpeed < DEADBAND_OFFSET) rightSpeed = DEADBAND_OFFSET;
  else if (rightSpeed < 0 && rightSpeed > -DEADBAND_OFFSET) rightSpeed = -DEADBAND_OFFSET;

  // --- INTERFACE HARDWARE L293D ---
  
  // KONTROL MOTOR KIRI
  if (leftSpeed > 0) {
    motorLeft.run(FORWARD);
    motorLeft.setSpeed(leftSpeed);
  } else {
    // Pengereman aktif / Mundur untuk belokan tajam
    motorLeft.run(BACKWARD); 
    motorLeft.setSpeed(abs(leftSpeed));
  }

  // KONTROL MOTOR KANAN
  if (rightSpeed > 0) {
    motorRight.run(FORWARD);
    motorRight.setSpeed(rightSpeed);
  } else {
    motorRight.run(BACKWARD);
    motorRight.setSpeed(abs(rightSpeed));
  }
}

// ======================================================================================
// 8. SISTEM FAILSAFE & PEMULIHAN
// ======================================================================================

void executeRecovery() {
  // LINE LOST! Robot buta.
  // Strategi: Putar di tempat ke arah terakhir garis terlihat.
  
  int recoverySpeed = 150; // Kecepatan putar yang aman

  if (lastKnownDirection == -1) {
    // Garis terakhir terlihat di KIRI -> Putar Kiri di tempat
    motorLeft.run(BACKWARD);
    motorRight.run(FORWARD);
  } else {
    // Garis terakhir terlihat di KANAN -> Putar Kanan di tempat
    motorLeft.run(FORWARD);
    motorRight.run(BACKWARD);
  }
  
  motorLeft.setSpeed(recoverySpeed);
  motorRight.setSpeed(recoverySpeed);
}