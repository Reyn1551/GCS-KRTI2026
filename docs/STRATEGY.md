# STRATEGY.md — Strategi Pengembangan & Skor, Divisi VTOL

## 1. Prioritas Berdasarkan Skor (jangan bangun semua misi merata)

Skor: M1=10, **M2=40**, M3=20, M4=15, M5=15. Plus M2 sempurna = jalur ke
Absolute Victory (menang instan, tanpa banding skor).

**Urutan investasi waktu development yang disarankan:**

1. **Misi 2 (dropping ke Red Box)** — bobot terbesar & jalur kemenangan
   instan. Ini butuh: deteksi `container` yang stabil dari berbagai sudut
   & ketinggian, estimasi posisi relatif akurat, kontrol approach yang bisa
   menahan posisi (hover/align) tepat di atas target, dan timing release
   payload yang benar (mempertimbangkan kecepatan wahana + jatuh bebas
   payload, kalau masih bergerak saat release).
2. **Misi 3 (Triple Gate)** — 20 poin, tapi ini murni "melewati lorong
   sempit secara utuh", jadi lebih ke masalah kontrol stabil & presisi
   dimensi wahana vs lorong (2m panjang total, gerbang 1m spacing) daripada
   masalah AI kompleks. Kalau wahana secara fisik terlalu besar/tidak
   stabil, ini akan gagal terus — cek dimensi fisik wahana vs lorong dulu.
3. **Misi 1 (manual gate)** — cuma 10 poin dan dikendalikan manual oleh
   pilot, jadi risiko software rendah, tapi tetap wajib lolos supaya masuk
   mode Autonomous. Fokus software di sini minim — investasi waktu ke
   training pilot lebih efisien daripada software.
4. **Misi 4 (line following)** & **Misi 5 (single gate + landing presisi)**
   — 15+15 poin, deteksi garis putus-putus + presisi mendarat. Kerjakan
   setelah M2 & M3 solid, karena secara skor lebih kecil, tapi jangan
   diabaikan — gagal landing = M5 nol dan berisiko rusak wahana untuk
   misi berikutnya kalau ada retry final.

## 2. Karena Kalian Baru di Tahap Uji Inferensi

Realistis: sebelum bicara skor, kalian butuh chain lengkap
persepsi→keputusan→aksi jalan sekali secara end-to-end (lihat
`ROADMAP.md`). Strategi yang disarankan:

- **Jangan optimalkan akurasi model YOLO dulu sampai berlebihan** kalau
  belum ada jalur ke flight controller — akurasi 90% tidak berguna kalau
  tidak ada cara mengeksekusi aksinya. Cukup model yang "cukup baik" dulu
  (confidence stabil di jarak & sudut realistis lapangan), lalu buktikan
  seluruh pipeline jalan di simulasi/bench test, baru kembali tuning model.
- **Bangun di urutan: bench test → indoor tethered test → hover test →
  full mission test**, jangan loncat ke uji misi penuh di lapangan
  langsung dari uji inferensi.
- Gunakan video yang sudah direkam (`hailo-video-output/`,
  `output_live.avi`) sebagai data replay untuk mengembangkan &
  memvalidasi `gate_mission.py` state machine tanpa perlu terbang setiap
  kali iterasi.

## 3. Trade-off Teknis yang Perlu Diputuskan Tim (bukan keputusan software semata)

- **GPS-denied requirement (§3.3.7.6, §3.3 tema)**: karena misi eksplisit
  menyebut area GPS-denied, arsitektur navigasi **tidak boleh bergantung
  penuh ke GPS**. Opsi realistis untuk companion computer RPi5+Hailo:
  - Visual odometry / optical flow (butuh sensor tambahan atau kamera
    bawah + downward-facing flow sensor),
  - Pure vision-based waypoint tracking (approach berbasis ukuran &
    posisi bbox objek di frame, tanpa perlu koordinat global) — ini paling
    cocok dengan setup kalian sekarang (YOLO + kamera) dan lebih murah
    untuk diimplementasikan duluan.
  - Kombinasi dengan IMU dari FC untuk dead-reckoning jangka pendek antar
    deteksi.
  Rekomendasi: mulai dari **pure vision-based (bbox-driven) navigation**
  dulu karena paling sesuai kemampuan kalian saat ini, upgrade ke sensor
  fusion kalau waktu memungkinkan.

- **Precision landing (Misi 5)**: pertimbangkan marker khusus (ArUco) di
  Final Landing Pad kalau boleh dipasang sendiri, karena YOLO generik bbox
  pad merah/biru kurang presisi untuk align akhir dibanding marker
  fiducial dengan pose 6DOF.

- **Payload release timing**: uji jatuh bebas First Aid Kit (100g, dari
  berbagai ketinggian hover) di darat dulu untuk kalibrasi offset
  drop-point vs posisi release — jangan andalkan estimasi teoritis saja.

## 4. Metrik yang Perlu Ditrack dari Awal (masukkan ke `mission_logs/`)

- Precision & recall per kelas (`gate`, `container`, `waypoint`) pada
  berbagai jarak & sudut — supaya tahu batas jarak deteksi yang reliable.
- Latency end-to-end: frame capture → deteksi → keputusan → aksi terkirim.
  Ini menentukan kecepatan aman wahana mendekati gate/target (kalau
  latency tinggi, wahana harus melambat).
- Jarak error drop (cm dari titik tengah Red Box) tiap uji simulasi/real.
- Waktu tiap state machine menyelesaikan misi (untuk estimasi apakah 10
  menit cukup, dan berapa slack untuk retry).
