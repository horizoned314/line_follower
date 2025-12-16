# Robot Line Follower - Versi Stabil & Tangguh

Implementasi line follower robot yang robust untuk Arduino Uno dengan fitur-fitur canggih untuk menangani berbagai kondisi lintasan.

## 🎯 Fitur Utama

### 1. **PID Controller dengan Anti-Windup**
- Kontrol PID yang stabil dengan dynamic speed adjustment
- Anti-windup protection (clamping ±8000)
- Sharp error multiplier untuk respons cepat pada belokan tajam
- Deadband filtering untuk stabilitas di garis lurus

### 2. **Toleransi Garis Putus-Putus**
- Menangani garis terputus-putus dengan delay 140ms
- Robot tetap maju saat garis hilang sementara
- Mencegah osilasi berlebihan pada garis tidak sempurna

### 3. **Deteksi Junction/Simpang**
- Mendeteksi persimpangan berdasarkan jumlah sensor aktif
- Menggunakan sensor sayap untuk menentukan arah belokan
- Memori arah terakhir untuk konsistensi navigasi

### 4. **Dead-End Detection & Smart U-Turn**
- Deteksi otomatis dead-end setelah 450ms kehilangan garis
- U-turn cerdas yang memutar berlawanan arah terakhir
- Mencegah robot kembali ke arah semula

### 5. **Belok Tajam & Melengkung**
- Sensor sayap (wing sensors) untuk deteksi belokan 90°
- Assistensi tambahan saat belok tajam
- Handling smooth untuk belokan melengkung

### 6. **Failsafe Recovery System**
- Prioritas sensor sayap untuk pemulihan
- Memori arah terakhir untuk recovery strategy
- Spin-in-place recovery saat garis hilang

## 🔧 Hardware Requirements

### Komponen
- **Microcontroller**: Arduino Uno
- **Motor Driver**: L293D Motor Shield (AFMotor library)
- **Sensors**: 
  - 6x Sensor Analog (A0-A5) - Array utama
  - 2x Sensor Digital (D9, D10) - Sensor sayap kiri/kanan
- **Motors**: 2x DC Motors
- **Power**: Sesuai spesifikasi motor (7-12V recommended)

### Wiring Map
```
Sensor 1-6 (Analog):  A0, A1, A2, A3, A4, A5
Sensor 7 (Sayap Kiri): D9
Sensor 8 (Sayap Kanan): D10
Motor Kiri:           Terminal M3 pada L293D Shield
Motor Kanan:          Terminal M4 pada L293D Shield
LED Indicator:        Pin 13 (built-in)
```

## ⚙️ Parameter Tuning

### 1. Threshold Sensor
```cpp
int threshold[] = {560, 570, 511, 517, 542, 517};
```
**Cara kalibrasi:**
1. Baca nilai sensor pada permukaan putih (background)
2. Baca nilai sensor pada garis hitam
3. Set threshold di tengah-tengah kedua nilai
4. Formula: `threshold = (nilai_putih + nilai_hitam) / 2`

### 2. PID Gains
```cpp
float Kp = 0.045;   // Proportional gain
float Ki = 0.0004;  // Integral gain (kecil!)
float Kd = 7.0;     // Derivative gain
```
**Prosedur tuning:**
1. Mulai dengan `Ki = 0`, `Kd = 0`
2. Naikkan `Kp` sampai robot mulai osilasi
3. Kurangi `Kp` sedikit (80-90% dari nilai osilasi)
4. Tambahkan `Kd` untuk meredam osilasi
5. Tambahkan `Ki` sangat kecil untuk koreksi steady-state

### 3. Kecepatan Motor
```cpp
const int BASE_SPEED     = 150;  // Kecepatan normal (0-255)
const int MAX_SPEED      = 200;  // Kecepatan maksimum
const int MIN_BASE_SPEED = 50;   // Kecepatan minimum saat belok tajam
const int TURN_SPEED     = 200;  // Kecepatan saat spin recovery
```
**Penyesuaian:**
- Sesuaikan dengan tegangan baterai
- Motor yang lebih kuat: naikkan BASE_SPEED
- Lintasan licin: kurangi TURN_SPEED

### 4. Timing Parameters
```cpp
const int LINE_LOST_DELAY   = 140;  // Toleransi garis putus (ms)
const int DEAD_END_TIMEOUT  = 450;  // Timeout deteksi buntu (ms)
```
**Kalibrasi:**
- `LINE_LOST_DELAY`: Naikkan jika garis putus-putus lebih panjang
- `DEAD_END_TIMEOUT`: Sesuaikan dengan panjang jarak dead-end

### 5. Dead-End Spin Time
```cpp
int spinTime = 280;  // Durasi putar U-turn (ms)
```
**Kalibrasi:**
- Robot harus putar ~160-200 derajat
- Test di tempat, ukur durasi untuk 180°
- Kurangi sedikit agar tidak kembali ke arah semula

### 6. Advanced Parameters
```cpp
const int DEADBAND           = 15;    // Error deadband
const int BRAKE_FACTOR       = 90;    // Dynamic braking intensity
const int SHARP_ERROR        = 1300;  // Threshold error tajam
const float SHARP_MULTIPLIER = 1.9;   // Multiplier untuk sharp turn
const int JUNCTION_COUNT_MIN = 2;     // Min sensor aktif untuk junction
```

## 🚀 Quick Start

### 1. Upload Code
```bash
# Menggunakan Arduino IDE:
1. Buka main.ino
2. Pilih Board: Arduino Uno
3. Pilih Port yang sesuai
4. Upload

# Atau menggunakan arduino-cli:
arduino-cli compile --fqbn arduino:avr:uno main.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno main.ino
```

### 2. Kalibrasi Sensor
1. Letakkan robot di lintasan
2. Baca nilai sensor menggunakan Serial Monitor
3. Update array `threshold[]` dengan nilai yang sesuai
4. Re-upload code

### 3. Tuning PID
1. Mulai dengan nilai default
2. Jalankan robot dan observasi
3. Adjust Kp, Ki, Kd sesuai behavior:
   - Terlalu lambat respons → naikkan Kp
   - Osilasi berlebihan → naikkan Kd
   - Tidak bisa koreksi steady-state → naikkan Ki sedikit

### 4. Test Scenarios
- [ ] Garis lurus panjang
- [ ] Belokan 90 derajat kiri/kanan
- [ ] Belokan melengkung/smooth
- [ ] Garis putus-putus
- [ ] Simpang T/Y
- [ ] Dead-end/buntu

## 📊 Algoritma Flow

```
START
  ↓
Read Sensors → Calculate Position
  ↓
Position = -1? ────YES──→ Line Lost?
  ↓ NO                      ↓
  ↓                    Tolerance OK? ──YES→ Keep Moving
  ↓                         ↓ NO
  ↓                    Dead-End? ──YES→ U-Turn (handleDeadEnd)
  ↓                         ↓ NO
  ↓                    Failsafe Recovery
  ↓
Junction Detected? ──YES→ handleJunction()
  ↓ NO
  ↓
Calculate PID
  ↓
Apply Dynamic Speed
  ↓
Check Wing Sensors
  ↓
Set Motor Speed
  ↓
LOOP
```

## 🔍 Troubleshooting

### Robot Osilasi Berlebihan
- **Penyebab**: Kp terlalu tinggi
- **Solusi**: Kurangi Kp atau naikkan Kd

### Robot Lambat Respons
- **Penyebab**: Kp terlalu rendah atau BRAKE_FACTOR terlalu tinggi
- **Solusi**: Naikkan Kp atau kurangi BRAKE_FACTOR

### Tidak Deteksi Garis Putus-Putus
- **Penyebab**: LINE_LOST_DELAY terlalu rendah
- **Solusi**: Naikkan nilai LINE_LOST_DELAY

### U-Turn Kembali ke Arah Semula
- **Penyebab**: spinTime terlalu rendah atau terlalu tinggi
- **Solusi**: Kalibrasi spinTime untuk putar 160-200°

### Tidak Deteksi Junction
- **Penyebab**: JUNCTION_COUNT_MIN tidak sesuai atau threshold sensor salah
- **Solusi**: Sesuaikan JUNCTION_COUNT_MIN atau kalibrasi ulang threshold

### Robot Terus Spin di Tempat
- **Penyebab**: Semua sensor tidak mendeteksi garis
- **Solusi**: 
  - Cek kalibrasi threshold
  - Cek pencahayaan lintasan
  - Cek koneksi sensor

## 📝 Code Structure

```
main.ino
├── KONFIGURASI & TUNING
│   ├── Speed parameters
│   ├── PID gains
│   └── Timing constants
│
├── HARDWARE
│   ├── Motor definitions
│   └── Sensor pin mapping
│
├── VARIABEL
│   └── State tracking variables
│
├── setup()
│   └── Hardware initialization
│
├── loop() [MAIN]
│   ├── readLinePosition()
│   ├── PID calculation
│   ├── Junction handling
│   ├── Dead-end handling
│   └── Motor control
│
└── HELPER FUNCTIONS
    ├── readLinePosition()
    ├── handleJunction()
    ├── handleDeadEnd()
    ├── handleFailsafe()
    └── setMotorSpeed()
```

## 🎓 Advanced Tips

### 1. Optimasi untuk Kecepatan Tinggi
- Naikkan Kd untuk stabilitas lebih baik
- Kurangi Ki untuk mencegah overshoot
- Naikkan SHARP_MULTIPLIER untuk respons lebih cepat

### 2. Optimasi untuk Presisi
- Turunkan BASE_SPEED
- Naikkan Ki sedikit untuk tracking lebih akurat
- Kurangi DEADBAND untuk respons lebih halus

### 3. Handling Lintasan Sulit
- Naikkan LINE_LOST_DELAY untuk garis putus yang panjang
- Sesuaikan DEAD_END_TIMEOUT sesuai karakteristik lintasan
- Gunakan sensor sayap dengan positioning optimal

## 📚 Dependencies

- **AFMotor Library**: Motor control untuk L293D Shield
  ```
  Arduino IDE: Sketch → Include Library → Manage Libraries → Search "Adafruit Motor Shield"
  ```

## 🤝 Contributing

Contributions are welcome! Silakan sesuaikan parameter dan algoritma untuk kasus penggunaan Anda.

## 📄 License

Open source - silakan gunakan dan modifikasi sesuai kebutuhan.

## 📞 Support

Jika mengalami masalah:
1. Periksa wiring dan koneksi hardware
2. Verifikasi kalibrasi sensor
3. Test motor secara terpisah
4. Monitor Serial output untuk debugging

---

**Happy Line Following! 🤖🏁**
