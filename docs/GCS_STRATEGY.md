# GCS_STRATEGY.md — Strategi Ground Control Station (dengan & tanpa RTK)

Dokumen ini melengkapi `ARCHITECTURE.md` §4 (gap: belum ada interface ke
Flight Controller). Fokus: bagaimana GCS dan skema waypoint dirancang agar
fleksibel — bisa pakai RTK (presisi tinggi, butuh GPS bagus) atau tanpa RTK
(vision-based, wajib untuk area GPS-denied sesuai tema misi VTOL).

## 1. Kenapa Perlu Dua Skema (bukan pilih salah satu)

Tema misi VTOL eksplisit menyebut **area GPS-denied** (lihat `RULES.md`
§1). Artinya:

- **RTK/GPS** hanya bisa diandalkan di *sebagian* lintasan (mis. area
  terbuka antara Start Zone dan gate pertama, kalau langit terbuka).
- Untuk approach presisi ke gate sempit, dropping ke Red Box, line
  following, dan landing presisi — GPS/RTK **tidak cukup** (akurasi GPS
  standar ~1-3m, RTK ~2cm *tapi* butuh sinyal satelit bagus & base station
  — sering tidak realistis untuk manuver dekat objek fisik/indoor).

**Strategi**: RTK/waypoint GPS sebagai *opsional enhancement* untuk fase
transit, vision-based guidance dari companion computer (RPi5+Hailo) sebagai
*wajib* untuk fase presisi. Keduanya berjalan di atas flight stack yang
sama (ArduPilot), jadi tidak perlu dua sistem terpisah.

## 2. Referensi Open Source yang Dipakai

| Komponen | Pilihan Open Source | Peran |
|---|---|---|
| Flight stack (firmware FC) | **ArduPilot** (Copter atau QuadPlane, tergantung konfigurasi VTOL kalian) | Stabilisasi, EKF, failsafe, geofence, eksekusi mission/waypoint, mode AUTO/GUIDED |
| GCS software (pre-flight & monitoring) | **Mission Planner** (Windows/Mono) atau **QGroundControl** (cross-platform, lebih ringan di Linux/RPi) | Set mission, atur parameter, kalibrasi sensor, monitor telemetry real-time |
| Companion-side automation | **pymavlink** atau **MAVSDK-Python** | RPi5 kirim/override setpoint saat fase vision-guided, baca telemetry untuk state machine |
| Command-line/scriptable GCS | **MAVProxy** | Berguna untuk debugging cepat di lapangan tanpa GUI, atau dijalankan headless di RPi5 sebagai MAVLink router |
| Simulasi sebelum uji terbang | **ArduPilot SITL** (+ opsional Gazebo/jMAVSim) | Uji mission file & state machine `gate_mission.py` tanpa hardware — cocok untuk Fase 1–2 di `ROADMAP.md` |
| RTK correction (jika dipakai) | **RTKLIB** atau layanan **NTRIP** (mis. dari CORS lokal) | Kirim koreksi RTCM3 ke GPS rover di wahana |

## 3. Dua Skema Waypoint

### 3.1 Skema A — Waypoint GPS/RTK (untuk fase transit di area terbuka)

- Mission file format standar ArduPilot (`.waypoints` / QGC WPL110), berisi
  waypoint lat/lon/alt absolut.
- Cocok untuk: perjalanan Start Zone → area sebelum Gate 1 (kalau
  konfigurasi lapangan memungkinkan sinyal GPS bagus di situ).
- Mode FC: `AUTO` (menjalankan mission list) atau `GUIDED` dengan setpoint
  posisi global dari companion computer.
- **RTK opsional**: jika dipasang (GPS rover RTK di wahana + base station
  RTK di darat, koreksi via radio telemetry atau NTRIP/4G), akurasi posisi
  bisa turun ke level cm — berguna kalau lapangan lomba memang punya area
  terbuka cukup luas dan kalian mau approach awal lebih presisi/cepat
  tanpa bergantung vision dari jarak jauh.
- **Tanpa RTK**: pakai GPS standar (akurasi ~1-3m) hanya untuk *kasar*
  menuju area target, lalu serahkan ke vision begitu objek mulai terdeteksi
  dari kamera (± beberapa meter dari gate/waypoint).

### 3.2 Skema B — Vision-Guided (wajib, untuk semua fase presisi & GPS-denied)

- Tidak pakai mission file absolut. Companion computer (RPi5) mengirim
  setpoint **relatif** ke FC via MAVLink `SET_POSITION_TARGET_LOCAL_NED`
  atau kontrol kecepatan (`velocity setpoint`) berdasarkan posisi bbox
  objek (`gate`, `container`, `waypoint`) dari `hailo.py`.
- Mode FC: `GUIDED` (ArduPilot menerima setpoint eksternal per-frame dari
  companion computer, FC tetap pegang stabilisasi rendah-level & failsafe).
- Ini yang dijelaskan sebagai jalur "companion computer standby → ambil
  alih" di `ARCHITECTURE.md` §4.2.
- **Tidak bergantung GPS sama sekali** — cocok untuk syarat GPS-denied di
  aturan dan untuk manuver presisi (Triple Gate, dropping, line follow,
  landing).

### 3.3 Hybrid (rekomendasi realistis untuk kontes)

```
Start Zone --[Manual, pilot RC]--> Gate 1&2 --[opsional GPS/RTK kasar
  jika area terbuka]--> mendekati WP1 --[vision-guided mulai ambil
  alih begitu objek terdeteksi stabil]--> WP1 (switch ke Autonomous)
  --[full vision-guided]--> WP2 (drop) --> Triple Gate --> WP4 --> line
  follow --> WP5 --> Single Gate --> Landing Pad (vision/ArUco-guided)
```

Titik transisi GPS→vision **harus eksplisit** di state machine
(`gate_mission.py`), bukan otomatis berdasarkan "GPS makin tidak akurat" —
gunakan trigger yang jelas, mis. "objek terdeteksi dengan confidence tinggi
dan stabil selama N frame" → switch companion computer mulai kirim
setpoint GUIDED, override mission GPS.

## 4. Fitur GCS yang Perlu Disiapkan Tim

Idealnya GCS (Mission Planner/QGC) dipakai untuk **pre-flight setup**, bukan
runtime kontrol saat fase vision-guided (itu tugas companion computer).
Fitur yang perlu disiapkan/dilatih tim:

1. **Set Mission** — buat & simpan mission file per skenario:
   - `mission_seleksi_wilayah.waypoints` (Misi 1–3, area 15×15m sesuai
     `RULES.md` §9)
   - `mission_final.waypoints` (Misi 1–5, lapangan final 60×80m)
   - Simpan semua mission file di repo (`config/missions/`), bukan cuma di
     GCS lokal — supaya versinya konsisten & bisa direview seperti kode.
2. **Konfigurasi Parameter ArduPilot** — kategori yang relevan untuk VTOL
   KRTI (nama parameter contoh untuk ArduPilot Copter/QuadPlane, cek versi
   firmware kalian karena nama bisa sedikit beda):
   - **Geofence**: `FENCE_ENABLE`, `FENCE_RADIUS`, `FENCE_ALT_MAX`,
     `FENCE_ACTION` — wajib sesuai `RULES.md` §7 (geofence ±50m horizontal,
     ±20m vertikal untuk FW, sesuaikan untuk VTOL dengan lapangan Seremoni).
   - **Failsafe**: `FS_GCS_ENABLE`, `FS_EKF_THRESH`, `RTL_ALT`,
     `BATT_FS_LOW_ACT`, `FS_THR_ENABLE` — untuk memenuhi syarat
     Emergency Landing System (auto-land saat lost contact >15 detik).
   - **EKF/Sumber Posisi** (penting untuk GPS-denied): `EK3_SRC1_POSXY`,
     `EK3_SRC1_VELXY`, `AHRS_EKF_TYPE` — bisa diarahkan pakai
     non-GPS source (mis. optical flow / external vision) saat GPS tidak
     tersedia/tidak diandalkan.
   - **RTK** (jika dipakai): `GPS_TYPE` (sesuai receiver, mis. u-blox F9P),
     setup *moving baseline* jika mau heading dari dual-antenna GPS.
   - **VTOL/QuadPlane** (jika pakai ArduPilot QuadPlane): `Q_ENABLE`,
     parameter transisi (`Q_TRANSITION_MS`, dll), tuning motor VTOL
     (`Q_M_*`).
   - **Link Companion Computer**: `SERIALx_PROTOCOL = 2` (MAVLink2),
     baud rate sesuai koneksi RPi5↔FC.
3. **Kalibrasi** — kompas, accelerometer, radio RC, ESC — standar
   prosedur ArduPilot, lakukan ulang tiap kali ganti frame/motor.
4. **Monitoring Real-Time** — battery voltage, GPS fix type (No Fix/3D
   Fix/RTK Float/RTK Fixed kalau pakai RTK), mode aktif, jarak ke
   geofence, EKF health — GCS dipakai officer/pit crew di darat untuk
   memantau, bukan untuk kirim kontrol saat fase otonom.
5. **Log Review** — ArduPilot dataflash log (`.bin`) untuk analisis
   pasca-uji-terbang (terpisah dari `mission_logs/` punya kalian yang
   berisi keputusan AI/state machine). Kombinasikan keduanya saat debug:
   log FC untuk "apa yang FC lakukan", `mission_logs/` untuk "kenapa
   companion computer memutuskan begitu".

## 5. Perbandingan Skema (untuk keputusan tim)

| Aspek | Full GPS/RTK Auto Mission | Full Vision (GPS-denied) | Hybrid (rekomendasi) |
|---|---|---|---|
| Sesuai tema GPS-denied | ❌ Tidak sesuai aturan | ✅ Sesuai | ✅ Sesuai (vision di fase kritis) |
| Presisi approach gate/drop | Rendah–sedang (GPS standar) / Tinggi (RTK, tapi butuh sinyal bagus) | Tinggi (kalau model & kontrol matang) | Tinggi di fase presisi |
| Kompleksitas software | Rendah | Tinggi | Sedang–Tinggi |
| Ketergantungan hardware tambahan | RTK base+rover (biaya, setup di lapangan) | Kamera+Hailo (sudah ada) | Kamera+Hailo wajib, RTK opsional |
| Risiko kegagalan saat sinyal GPS buruk | Tinggi | Tidak ada (tidak bergantung GPS) | Rendah (GPS cuma bantu transit awal) |
| Waktu setup di hari-H | Perlu setup base station RTK tiap lokasi baru | Tidak perlu | Perlu setup RTK hanya jika dipakai |

**Rekomendasi**: mulai dari kolom **Hybrid** tapi implementasikan **Full
Vision** logic dulu (karena itu yang wajib & berbobot skor besar di Misi
2–5), RTK ditambahkan belakangan sebagai *bonus* kalau waktu & anggaran
ada, khususnya untuk mempercepat fase transit sebelum objek pertama
terdeteksi kamera.

## 6. Langkah Praktis (terhubung ke `ROADMAP.md`)

Tambahkan ke **Fase 2 (Integrasi Flight Controller)** di `ROADMAP.md`:

- [ ] Install & konfigurasi ArduPilot (Copter/QuadPlane) di FC, set
      parameter dasar (geofence, failsafe, EKF source) sesuai §4 di atas.
- [ ] Setup Mission Planner/QGroundControl, buat mission file kosong dulu
      untuk validasi link MAVLink RPi5↔FC.
- [ ] Uji dengan **ArduPilot SITL** dulu sebelum hardware: jalankan
      `gate_mission.py` (setelah refactor jadi state machine) melawan SITL,
      kirim setpoint GUIDED palsu, pastikan FC merespons sesuai ekspektasi
      — ini jauh lebih murah & aman daripada langsung ke FC fisik.
- [ ] Baru setelah SITL lolos, lanjut ke bench test FC fisik (propeller
      lepas) seperti sudah direncanakan di `ROADMAP.md` Fase 2.
- [ ] Kalau tim memutuskan pakai RTK: alokasikan waktu terpisah untuk
      setup base station + uji fix RTK di lokasi latihan **sebelum** hari-H,
      karena kualitas RTK sangat bergantung kondisi lokasi (obstruksi
      sinyal, jarak ke base).

## 7. UI Custom Mission Control: Web-based, dan Kenapa Dipisah dari Jalur Safety

Pertanyaan yang wajar muncul: mission control panel kustom (§4/§6) itu
dibuat web-based, native app, atau CLI? Jawabannya **web-based untuk
monitoring & mission setup, TAPI abort/RTL/disarm tidak boleh cuma lewat
web itu**. Alasannya menyangkut keselamatan, bukan preferensi teknis semata.

### 7.1 Kenapa Web-Based untuk Panel Mission Control

- Tim sudah full Python (`hailo.py`, `gate_mission.py`, dst) — **FastAPI/
  Flask + WebSocket** paling natural untuk expose state mission, live
  telemetry, dan overlay video tanpa nulis stack UI baru dari nol.
- Bisa diakses dari **laptop atau tablet pit crew** lewat browser biasa,
  tanpa install apapun — RPi5 jalan sebagai WiFi Access Point atau ikut
  jaringan lapangan, device pit crew tinggal buka `http://<ip-rpi5>:8000`.
- Video overlay Hailo (bbox+class) paling gampang di-stream sebagai **MJPEG
  over HTTP** (`multipart/x-mixed-replace`) — jauh lebih ringan CPU-nya di
  RPi5 dibanding WebRTC, penting karena RPi5 juga sedang menjalankan
  inference Hailo + mission logic bersamaan. WebRTC baru dipertimbangkan
  kalau nanti MJPEG terbukti terlalu lag untuk kebutuhan monitoring.
- Cocok untuk fitur di §4 dokumen ini: pilih skenario misi, load field
  profile, lihat state machine & sisa waktu timer, review log — semuanya
  read-mostly / low-frequency command, aman lewat HTTP/WebSocket.

### 7.2 Kenapa Abort/RTL/Disarm TIDAK Boleh Bergantung ke Web Dashboard Ini

- Web dashboard berjalan **di atas RPi5 dan jaringan WiFi lapangan** — dua
  titik gagal (companion computer crash, WiFi putus/interferensi) yang
  sama sekali di luar kendali flight controller.
- Kalau tombol "Abort" di web app ternyata satu-satunya jalur untuk
  memerintah RTL, dan RPi5/WiFi bermasalah tepat saat itu — safety officer
  tidak punya cara menyelamatkan wahana. Ini bertentangan dengan prinsip
  di `AGENTS.md` §2 (fail-safe eksplisit, tidak boleh diam-diam gagal).
- **Jalur abort yang benar**: radio telemetry MAVLink *langsung* ke flight
  controller (via Mission Planner/QGroundControl di laptop safety officer,
  atau RC switch fisik ke mode RTL/LAND) — jalur ini independen dari RPi5
  dan WiFi dashboard sama sekali. Web dashboard boleh **menampilkan**
  tombol abort sebagai kenyamanan (ikut kirim perintah MAVLink kalau
  link RPi5 masih hidup), tapi tidak boleh jadi satu-satunya jalur.

### 7.3 Ringkasan Pembagian

| Fungsi | Media | Alasan |
|---|---|---|
| Set misi, lihat state & timer, review log, tuning threshold | **Web (FastAPI/Flask + WebSocket) di RPi5** | Nyaman, low-risk kalau delay/putus sesaat |
| Live video + telemetry overlay | **Web (MJPEG stream)** | Murah CPU, cukup untuk monitoring visual |
| Abort / RTL / LAND / Disarm | **Radio telemetry MAVLink langsung ke FC** (Mission Planner/QGC di laptop safety officer, atau RC fisik) | Harus independen dari RPi5 & WiFi — ini nyawa wahana |
| Konfigurasi parameter FC & kalibrasi | **Mission Planner/QGC** (radio telemetry langsung, bukan lewat RPi5) | Standar ArduPilot, tidak perlu direinvent, dan lebih aman dilakukan lewat jalur radio yang sama dengan abort |

## 8. Catatan Penting

- Selama fase `GUIDED` (vision-guided), **GCS di darat tetap harus bisa
  override ke `RTL`/`LAND` kapan saja** dari pilot/safety officer — ini
  wajib untuk keselamatan (lihat `RULES.md` §7 & `AGENTS.md` §2). Jangan
  desain sistem yang butuh companion computer "sehat" agar bisa
  di-override manual.
- Simpan semua parameter FC (`.param` file) di repo git, sama seperti
  mission file — supaya kalau ganti wahana/FC bisa restore konfigurasi
  dengan cepat, dan ada riwayat perubahan (parameter yang salah sering
  jadi penyebab insiden, penting bisa audit).
