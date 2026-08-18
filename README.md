# KRTI 2026 — Sistem Wahana Otonom VTOL (Kingphoenix)

Repository terintegrasi untuk sistem wahana otonom VTOL KRTI (Kontes Robot Terbang Indonesia) 2026:
- **Companion Computer**: Raspberry Pi 5 (8GB)
- **AI Accelerator**: Hailo-8 M.2 (YOLOv26 HEF models)
- **Vision & Depth Sensor**: Intel RealSense D435i (RGB + Stereo Depth Anti-Collision)
- **Flight Controller**: Pixhawk / ArduPilot via MAVLink (UART `/dev/ttyAMA0` 921600 baud)
- **Payload Release**: PCA9685 16-Channel 12-bit PWM I2C Controller (Channels 3 & 4)
- **Ground Control Station**: Web-based GCS (FastAPI Backend + Leaflet/WebSocket Frontend)

---

## 📁 Struktur Direktori & Modul

```text
KRTI2026-WP/
├── config.py                 # Konfigurasi sentral (parameter kamera, PID, MAVLink, servo, safety)
├── camera_calibration.json   # Parameter intrinsik & distorsi RealSense D435i
├── servo_diag.py             # Alat diagnostik & pengujian hardware PCA9685 I2C
├── launch_mission.sh         # Shell script runner untuk menjalankan misi autonomous
│
├── core/                     # Library utama persepsi, estimasi keadaan & kontrol
│   ├── camera.py             # Pipeline capture RealSense D435i & loader kalibrasi
│   ├── depth_safety.py       # Monitor anti-kolusi stereo depth D435i & spatial clearance
│   ├── drone.py              # Controller MAVLink ArduPilot (GUIDED mode, velocity, takeoff/land)
│   ├── fusion.py             # Fusi data deteksi YOLO dengan depth map
│   ├── gate_kf.py            # Kalman Filter 8-state untuk estimasi posisi & kecepatan objek
│   ├── guidance.py           # Kontroler visual servoing PID (vx, vy, vz)
│   ├── hailo_detector.py     # Wrapper inferensi YOLO pada akselerator Hailo-8
│   ├── tracker.py            # Pelacak bounding box dengan IoU matching & smoothing
│   └── vision_utils.py       # Utilitas visual, estimasi pose ArUco 6DOF & HUD overlay
│
├── control/                  # Modul aktuator & kendali servo payload
│   ├── servo.py              # Class ServoController PCA9685 (buka, tutup, lepas) & mode CLI
│   ├── mavlink_servo.py      # Listener MAVLink DO_SET_SERVO / SERVO_OUTPUT_RAW
│   └── mavlink_toggle_servo.py # Listener MAVLink DO_SET_RELAY & DO_SET_SERVO dari GCS/Mission Planner
│
├── missions/                 # Implementasi state machine misi otonom
│   ├── gate_mission.py       # Misi otonom melewati gate (Search -> Approach -> Precision Align -> Pass)
│   ├── container_mission.py  # Misi otonom First Aid Kit payload drop (YOLO Container + Servo)
│   └── waypoint_engine.py    # Mesin navigasi visual Waypoint & deteksi ArUco marker
│
├── tools/                    # Alat bantu pengujian & kalibrasi
│   ├── bbox_calibration.py   # GUI kalibrasi interactive untuk center offset bounding box
│   ├── depth_viewer.py       # Visualizer depth map RealSense secara real-time
│   └── hailo_live.py         # Viewer live test deteksi objek YOLO pada Hailo-8
│
├── backend/                  # Backend Ground Control Station (GCS)
│   ├── main.py               # Server FastAPI (REST endpoints, WebSocket telemetry, static files)
│   ├── mavlink_manager.py    # Background worker thread MAVLink telemetry & mission upload
│   └── requirements.txt      # Dependency Python backend GCS
│
├── frontend/                 # Web UI Ground Control Station
│   ├── index.html            # Antarmuka web GCS (Peta, Telemetri, Status, Kontrol Misi)
│   ├── app.js                # Logika WebSocket, render peta Leaflet & interaksi GCS
│   ├── style.css             # Tema & styling web GCS
│   └── vendor/               # Library vendor offline (Leaflet.js, Leaflet.css)
│
├── pi/                       # Skrip service & streaming Raspberry Pi
│   ├── camera_server.py      # Server streaming video MJPEG
│   ├── detector.py           # Hailo-8 overlay detector untuk MJPEG stream
│   ├── pca_servo.py          # Modul MAVProxy untuk mirror servo ArduPilot ke PCA9685
│   ├── start_cameras.sh      # Skrip peluncuran streaming kamera
│   └── start_mavproxy.sh     # Skrip peluncuran MAVProxy routing telemetry
│
├── model/                    # Model neural network YOLO yang sudah di-compile ke HEF
│   ├── 320px-v1.hef          # Model input 320x320
│   ├── 640px-v1.hef          # Model input 640x640
│   ├── KP2026V1-YOLOv26.hef  # Model YOLOv26 versi 1
│   └── KP2026V2-YOLO26.hef   # Model YOLOv26 versi 2
│
└── docs/                     # Dokumentasi teknis & regulasi KRTI 2026
    ├── ARCHITECTURE.md       # Arsitektur sistem multi-tier
    ├── RULES.md              # Ringkasan aturan & mekanisme kontes
    ├── STRATEGY.md           # Strategi navigasi & mitigasi risiko
    ├── ROADMAP.md            # Roadmap pengembangan tim
    ├── GCS_STRATEGY.md       # Desain protokol & arsitektur GCS
    └── Panduan-KRTI-2026l.pdf # Buku panduan resmi KRTI 2026
```

---

## ⚙️ Panduan Penggunaan & Pengujian

### 1. Diagnostik & Pengujian Hardware Servo (PCA9685)
Gunakan modul diagnostik untuk memastikan komunikasi I2C dan pergerakan servo:
```bash
# 1. Scan bus I2C (mencari alamat 0x40 PCA9685)
python3 servo_diag.py --scan

# 2. Uji sweep servo (gerakan halus 0° -> 180° -> 0°)
python3 servo_diag.py --sweep --both

# 3. Uji pulsa PWM langsung (1500us / netral)
python3 servo_diag.py --pulse 1500 --channel 3

# 4. Mode interactive manual (o = buka, c = tutup, q = keluar)
python3 control/servo.py
```

### 2. Menjalankan MAVLink Servo Controller
Untuk menghubungkan perintah drop payload dari Mission Planner / GCS ke modul servo:
```bash
# Menjalankan listener MAVLink untuk payload release
python3 control/mavlink_toggle_servo.py --port /dev/ttyAMA0 --baud 921600

# Mode dry-run untuk simulasi di meja lab tanpa hardware
python3 control/mavlink_toggle_servo.py --dry-run
```

### 3. Menjalankan Misi Autonomous
Jalankan misi dengan launcher otomatis atau langsung via Python:
```bash
# Menjalankan Misi Gate (Gate Passing v4.0)
./launch_mission.sh --gates 1 --alt 1.4

# Menjalankan Misi Drop Container (First Aid Kit Drop)
python3 missions/container_mission.py --conf 0.35

# Menjalankan Misi dalam mode simulasi / vision-only (tanpa kirim perintah MAVLink)
python3 missions/gate_mission.py --vision-only
```

### 4. Menjalankan Ground Control Station (GCS)
Jalankan server backend FastAPI:
```bash
# Jalankan FastAPI GCS server
uvicorn backend.main:app --host 0.0.0.0 --port 8000
```
Buka browser dan akses `http://localhost:8000` (atau IP Raspberry Pi di jaringan lokal).

### 5. Kalibrasi Bounding Box & Kamera
```bash
# Kalibrasi offset & scaling bounding box YOLO secara interaktif
python3 tools/bbox_calibration.py --hef model/320px-v1.hef

# Visualisasi live deteksi Hailo-8
python3 tools/hailo_live.py --hef model/320px-v1.hef

# Visualisasi sensor stereo depth RealSense
python3 tools/depth_viewer.py
```

---

## 🛡️ Prinsip Keamanan & Fail-safe

1. **Depth Anti-Collision**: Modul `core/depth_safety.py` memonitor jalur depan drone secara real-time. Jika obstacle terdeteksi `< 0.3m`, sistem langsung memicu status EMERGENCY / Hover.
2. **Detection Timeout**: Jika objek target (Gate / Container) hilang selama `> 2.0s`, wahana otomatis beralih ke state `SEARCH` atau Hover untuk mencegah drifting.
3. **Heartbeat Fail-safe**: Jika komunikasi MAVLink dengan Flight Controller terputus `> 5.0s`, sinyal PWM servo otomatis dilepas (`duty_cycle = 0`) untuk keamanan aktuator.
4. **Dry-Run Compatibility**: Setiap modul memiliki mode `--dry-run` / `--vision-only` sehingga seluruh logika perangkat lunak dapat diuji dan diverifikasi sebelum pengujian terbang di lapangan.
