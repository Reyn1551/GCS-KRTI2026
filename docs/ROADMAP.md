# ROADMAP.md — Rencana Pengembangan Bertahap

Status saat ini: **model YOLO sudah dilatih (gate, container, waypoint) dan
sudah diuji inferensi di Hailo. Belum ada integrasi flight controller.
Belum ada uji terbang.**

Jangan loncat fase — tiap fase harus lulus kriteria sebelum lanjut, karena
ini menyangkut keselamatan wahana terbang sungguhan.

## Fase 0 — Validasi Model (sedang berjalan)
- [x] Model YOLO 3 kelas jalan di Hailo (`hailo.py`).
- [ ] Ukur precision/recall tiap kelas pada rentang jarak & sudut yang
      relevan dengan geometri lapangan (gate 1.5–2m, waypoint 2×2m,
      container/red box).
- [ ] Tentukan confidence threshold & jarak deteksi minimum yang reliable
      per kelas.
- [ ] Validasi `camera_calibration.json` masih akurat untuk kamera yang
      dipakai (cek reprojection error).

**Kriteria lulus fase**: tahu persis di jarak/sudut berapa tiap kelas
terdeteksi reliable (>90% pada confidence threshold yang dipilih).

## Fase 1 — Mission Logic Offline (tanpa hardware terbang)
- [ ] Refactor `gate_mission.py` jadi state machine eksplisit (lihat
      `ARCHITECTURE.md` §2.2) dengan checkpoint sesuai `RULES.md` §5.
- [ ] Tambah mode `--replay <video>` yang menjalankan state machine di atas
      video rekaman (`output_live.avi` dll) untuk validasi logic.
- [ ] Tambah mode `--dry-run` yang mencetak/log aksi (approach, release,
      dst) tanpa mengirim sinyal hardware apapun.
- [ ] Implementasi timer global 10 menit + logika retry-ke-checkpoint.

**Kriteria lulus fase**: state machine bisa "menyelesaikan" seluruh Misi
1–5 secara logis di atas data rekaman/simulasi, termasuk skenario retry.

## Fase 2 — Integrasi Flight Controller (bench test, propeller lepas)
- [ ] Pilih & pasang library MAVLink (`pymavlink` / `MAVSDK-Python`).
- [ ] Bangun `control/mavlink_client.py`: koneksi ke FC, baca telemetry
      (posisi, mode, battery, heartbeat), kirim setpoint dasar.
- [ ] Implementasi `failsafe_watchdog.py` independen: pantau heartbeat,
      trigger auto-land/RTH jika lost contact >15 detik (sesuai ELS di
      `RULES.md`).
- [ ] Uji di bangku (propeller dilepas / motor tidak terpasang): companion
      computer bisa memerintah mode Guided/Offboard dan FC merespons.
- [ ] Integrasikan `servo.py` (payload release) dipicu dari state machine,
      uji release mekanis di darat (drop test tanpa terbang).

**Kriteria lulus fase**: FC menerima & mengeksekusi perintah dari RPi5,
failsafe watchdog terbukti trigger saat koneksi diputus paksa, payload
release presisi mekanis terverifikasi.

## Fase 3 — Uji Terbang Tertambat/Terkendali (tethered / indoor terbatas)
- [ ] Uji hover manual dengan companion computer running tapi read-only
      (hanya logging, belum kirim perintah gerak) — pastikan tidak ada
      interferensi ke FC.
- [ ] Uji approach 1 objek (mis. waypoint) dalam mode terkendali/tertambat,
      companion computer kirim setpoint kecil, pilot siap override kapan
      saja.
- [ ] Validasi latency end-to-end (lihat `STRATEGY.md` §4) di kondisi
      terbang nyata, bukan cuma bench test.

**Kriteria lulus fase**: minimal satu approach otonom sederhana berhasil
dengan pilot bisa override instan, tanpa insiden.

## Fase 4 — Uji Misi Parsial (lapangan latihan, bertahap per misi)
- [ ] Misi 1 saja (manual melewati gate) — validasi transisi manual→auto.
- [ ] Misi 2 saja (approach + drop) — ini prioritas skor tertinggi, alokasikan
      waktu paling banyak di sini.
- [ ] Misi 3 saja (triple gate) — validasi dimensi fisik wahana muat.
- [ ] Misi 4 & 5 — line following dan landing presisi.

## Fase 5 — Uji Misi Penuh & Simulasi Kondisi Kontes
- [ ] Jalankan seluruh Misi 1–5 berurutan dengan timer 10 menit nyata.
- [ ] Simulasikan skenario retry di tiap titik checkpoint.
- [ ] Uji dengan gangguan realistis: cahaya berbeda, angin, objek sedikit
      bergeser posisi (karena juri/lapangan lomba tidak akan identik
      dengan lapangan latihan).

## Fase 6 — Persiapan Kontes
- [ ] Siapkan checklist pre-flight fisik (lihat draft di bawah, sesuaikan).
- [ ] Siapkan dokumentasi data logger (mission_logs format) untuk
      diperlihatkan ke juri jika diminta validasi.
- [ ] Backup wahana/komponen sesuai izin aturan (wahana cadangan spesifikasi
      identik, jika ada).
- [ ] Pastikan semua kelengkapan keselamatan terpasang: nav lights,
      emergency stop, geofencing aktif & teruji, helm tim.

---

### Checklist Pre-Flight Singkat (living document, lengkapi seiring waktu)
- [ ] Battery & baterai kondisi baik, tidak menggelembung.
- [ ] Geofencing aktif & radius sesuai lapangan hari itu.
- [ ] Failsafe/ELS teruji hari itu (simulasi lost-link).
- [ ] Model Hailo & versi kode yang dipakai sudah dikonfirmasi (hash/versi
      dicatat di log, hindari "salah upload versi lama").
- [ ] Mekanisme payload release dicek manual (tidak macet/nyangkut).
- [ ] Kamera & kalibrasi tidak bergeser sejak kalibrasi terakhir.
- [ ] Nav lights menyala (merah/hijau).
