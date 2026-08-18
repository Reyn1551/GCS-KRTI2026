# ARCHITECTURE.md — Arsitektur Sistem VTOL (RPi5 + Hailo)

## 1. Prinsip Desain

Pisahkan sistem jadi 3 lapisan yang independen dan bisa ditest terpisah:

```
[PERSEPSI]        [KEPUTUSAN / MISSION STATE MACHINE]      [AKSI]
Kamera → Hailo  →  gate_mission.py (per-misi state)     →  Perintah ke FC
(YOLO: gate,        + logika retry/checkpoint              (via MAVLink)
 container,         + kombinasi ArUco pose (opsional)    →  servo.py
 waypoint)          + logging ke mission_logs/               (payload release)
```

Alasan: kamu baru di tahap uji inferensi Hailo — jangan buru-buru menyatukan
semuanya jadi satu skrip monolitik. Struktur berlapis ini yang akan
menyelamatkan waktu kalian saat harus debug "kenapa drop meleset" tanpa
harus curiga ke seluruh pipeline.

## 2. Komponen

### 2.1 Persepsi — `hailo.py`
- Load HEF model (3 kelas: `gate`, `container`, `waypoint`).
- Output per frame: list deteksi `{class, confidence, bbox(x,y,w,h), frame_id, timestamp}`.
- Tambahkan **temporal filtering** (mis. butuh N frame berturut-turut dengan
  confidence > threshold sebelum dianggap "terkunci") supaya keputusan misi
  tidak goyah karena satu frame noise.
- Gunakan `camera_calibration.json` untuk undistort bbox atau menghitung
  estimasi jarak/sudut objek relatif ke kamera (penting untuk approach gate
  dan align drop point).
- Untuk `waypoint`: gabungkan deteksi YOLO (kasar, cepat, robust jarak jauh)
  dengan **deteksi ArUco marker OpenCV** (presisi pose 6DOF, tapi butuh
  jarak dekat & marker cukup besar di frame) — YOLO untuk cari & approach,
  ArUco untuk precision alignment saat sudah dekat. Ini pola umum yang
  efektif untuk drone lomba serupa.

### 2.2 Keputusan — `gate_mission.py` (dan modul misi lain yang akan ditambah)
- Implementasikan sebagai **state machine eksplisit** mengikuti Misi 1–5 di
  `RULES.md`, contoh state:
  `IDLE → M1_MANUAL_THROUGH_GATES → M1_AT_WP1 → M2_AUTO_TO_DROP →
  M2_DROPPING → M3_TRIPLE_GATE → M4_LINE_FOLLOW → M5_FINAL_GATE_LANDING →
  DONE / RETRY_<checkpoint>`.
- Setiap state punya:
  - kondisi masuk (entry condition, biasanya dari hasil persepsi/mode pilot),
  - aksi yang dijalankan selama state itu (approach, align, descend, release,
    dsb.),
  - kondisi keluar/sukses,
  - kondisi gagal → checkpoint retry sesuai `RULES.md` §5.
- **Timer global 10 menit** harus di-track di level ini (bukan di persepsi
  atau di FC), karena retry tidak mereset waktu.
- Semua keputusan (state transition, alasan) ditulis ke `mission_logs/`
  dengan timestamp — ini juga jadi bukti untuk juri (validasi data logger
  disebut di aturan RP, kemungkinan diminta juri juga di VTOL).

### 2.3 Aksi — perintah ke Flight Controller & `servo.py`
- **Belum ada di kode sekarang** — ini gap besar yang perlu direncanakan
  sebelum uji terbang pertama. Lihat `ROADMAP.md` Fase 2.
- Rekomendasi: gunakan **MAVLink** (via `pymavlink` atau `MAVSDK-Python`)
  untuk komunikasi RPi5 ↔ flight controller (ArduPilot/PX4). Companion
  computer mengirim **offboard/guided setpoint** (velocity atau position
  relatif), bukan menulis langsung ke motor — FC tetap pegang stabilisasi
  & failsafe dasar (ini juga lebih aman & lazim di kontes serupa).
- `servo.py` tetap terpisah, khusus payload release — dipanggil oleh state
  machine hanya saat kondisi align-drop terpenuhi (align = container
  terdeteksi di tengah frame bawah, ketinggian & kecepatan dalam batas).

## 3. Diagram Alir Data (per frame, runtime)

```
Frame kamera
   │
   ▼
hailo.py: inferensi YOLO → deteksi mentah
   │
   ▼
Filter temporal + kalkulasi posisi relatif (pakai kalibrasi kamera)
   │
   ▼
gate_mission.py: update state machine
   │              │
   │              ├─ jika perlu gerak → kirim setpoint MAVLink ke FC
   │              └─ jika kondisi drop terpenuhi → panggil servo.py (release)
   ▼
Log ke mission_logs/ (state, deteksi, aksi, timestamp)
```

## 4. Kebutuhan yang Belum Ada (untuk didiskusikan/dibangun)

1. **Interface ke Flight Controller** (MAVLink) — belum terlihat di daftar
   file. Ini prasyarat sebelum state machine bisa benar-benar "auto" karena
   Misi 2–5 semua otonom.
2. **Mode switch handling**: transisi Manual (Misi 1, pilot pegang RC) →
   Autonomous (Misi 2 dst, companion computer ambil alih) harus jelas siapa
   yang trigger (operator tekan tombol? atau otomatis begitu WP1
   terdeteksi tercapai?). Sesuai aturan, transisi ini dilakukan manual oleh
   pilot ("peserta diizinkan menyentuh wahana... mengubah pengaturan ke
   Mode Autonomous"), jadi companion computer harus **standby menunggu**
   sinyal mode-switch dari FC/RC, bukan memaksa switch sendiri.
3. **Collision avoidance** independen dari YOLO gate/container/waypoint
   (misalnya rangefinder/lidar) — disyaratkan di aturan §3.3.7.6, belum
   terlihat di file yang ada.
4. **ELS (Emergency Landing System)**: watchdog terpisah dari mission
   logic yang memantau heartbeat MAVLink; jika lost >15 detik → trigger
   auto-land. Ini idealnya berjalan sebagai proses/thread independen yang
   tidak bergantung pada state machine misi (supaya tetap jalan walau
   mission code crash).
5. **Test harness / simulator**: sebelum uji terbang, mampu memutar ulang
   video rekaman (`hailo-video-output/`, `output_live.avi`) melalui
   `gate_mission.py` dalam mode `--dry-run` untuk validasi logika state
   machine tanpa risiko hardware.

## 5. Struktur Direktori yang Disarankan (evolusi dari yang sekarang)

```
KP2026/
├── perception/
│   ├── hailo.py
│   ├── hailo_video_render.py
│   └── aruco_pose.py          # baru: presisi WP alignment
├── mission/
│   ├── state_machine.py       # baru: kerangka umum state+checkpoint+timer
│   ├── gate_mission.py        # existing, refactor pakai state_machine.py
│   ├── waypoint_mission.py    # baru
│   ├── line_follow_mission.py # baru (Misi 4)
│   └── landing_mission.py     # baru (Misi 5)
├── control/
│   ├── mavlink_client.py      # baru: koneksi & setpoint ke FC
│   ├── servo.py               # existing: payload release
│   └── failsafe_watchdog.py   # baru: ELS independen
├── config/
│   ├── field_dims.yaml        # angka dari RULES.md (gate size, WP size, dll)
│   └── camera_calibration.json
├── mission_logs/
├── model/
└── launch_mission.sh
```
