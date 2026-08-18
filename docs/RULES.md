# RULES.md — Ringkasan Aturan Resmi Divisi VTOL, KRTI 2026

Sumber: *Panduan Kontes Robot Terbang Indonesia (KRTI) 2026*, Bab 3.3.
Ringkasan ini untuk acuan development. **Jika ragu, cek dokumen aslinya
atau juknis final** — panitia menyatakan aturan bisa berubah (Bab IV).

## 1. Tema

"Smart City – Emergency Service: The Guardian of IKN" — VTOL sebagai
penyelamat otonom: navigasi manual + otonom berbasis visi (vision-based
navigation), melewati rintangan, area GPS-denied, mengirim First Aid Kit
dengan presisi.

Lokasi final: Lapangan Seremoni, IKN.

## 2. Objek Lapangan (relevan untuk model deteksi)

| Objek | Spesifikasi |
|---|---|
| **Start Zone** | Alas merah/biru, 1000×1000 mm |
| **Waypoint (WP)** | Alas oranye 2000×2000 mm, ArUco marker 500×500mm + 100×100mm di tengah. Warna oranye: R233 G146 B17. WP1–WP4. |
| **Single Gate** | Bukaan ±1500×1500mm dalam frame 1900×2000mm |
| **Double Gate** | 2 gerbang berjarak 1m |
| **Triple Gate** | 3 gerbang serial, jarak antar gerbang 1m, panjang lorong total 2m. Tiang aluminium profile 2020, panel triplek 5mm |
| **First Aid Kit** | Tas OneMed merah, ±145×36×123mm, berat asli 200–300g, **diisi ulang tim jadi minimal 100g** untuk lomba |
| **Drop Point / Red Box (WP2)** | Kotak/keranjang merah sebagai target dropping |
| **Final Landing Pad** | Alas merah/biru 5000×5000mm |
| **Line (garis pandu)** | Banner hitam, untuk Misi 4 (line following / path tracking berbasis visi) |

Catatan untuk model YOLO kita (`gate`, `container`, `waypoint`): `container`
kemungkinan = Red Box/drop target, `waypoint` = banner oranye+ArUco.
**Gate belum dipecah per single/double/triple di kelas model** — pertimbangkan
apakah perlu dibedakan atau cukup deteksi generik + geometri jarak antar
deteksi untuk membedakan jenis gate.

## 3. Struktur Misi (sekuensial, total waktu 10 menit termasuk semua retry)

| Misi | Dari → Ke | Mode | Syarat Sukses |
|---|---|---|---|
| 1 | Start → WP1 | **Manual** | Lewati Gate 1 & 2, lalu mendarat/melintas di atas WP1 ATAU sebagian wahana sudah masuk Double Gate. Boleh mendarat di WP1, pilot boleh sentuh wahana & ganti ke mode Autonomous. |
| 2 | WP1 → WP2 | **Autonomous** | Lewati Double Gate (lorong 1m), capai WP2 (Drop Point), jatuhkan First Aid Kit ke Red Box. Poin tambahan jika masuk box. Paket yang sudah jatuh tidak bisa diambil lagi. |
| 3 | WP2 → WP4 | **Autonomous** | Lewati Triple Gate (lorong serial 2m) secara utuh, capai WP4. (Catatan: penomoran di dokumen loncat WP2→WP4, WP3 disebut implisit di §3.3.11.2 sebagai titik seleksi wilayah — perlakukan WP4 sebagai titik setelah triple gate.) |
| 4 | WP4 → WP5 | **Autonomous** | Deteksi & ikuti garis hitam putus-putus di lapangan, berhenti/melintas presisi di WP5. |
| 5 | WP5 → Final Landing Pad | **Autonomous** | Lewati 1 Single Gate terakhir, mendarat otonom stabil di landing pad. |

**Absolute Victory**: tim pertama yang mendarat sempurna DENGAN paket sudah
masuk Red Box di Misi 2 → langsung menang, tanpa perlu bandingkan skor.

## 4. Skoring (total 100, jika tidak ada Absolute Victory)

| Misi | Elemen | Skor Maks | Rincian |
|---|---|---|---|
| 1 (Manual) | Take-off + Gate 1&2 | 10 | |
| 2 (Auto) | Navigasi ke WP2 + Dropping | 40 | 15 capai WP2; +25 paket masuk Red Box; +10 paket jatuh di area WP2 (tidak masuk box) |
| 3 (Auto) | Triple Gate → WP3 | 20 | Harus utuh lewati lorong |
| 4 (Auto) | WP3 → WP4 | 15 | Berhasil berada di atas WP4 |
| 5 (Auto) | Single Gate + Landing presisi | 15 | |

→ **Prioritas software**: Misi 2 (dropping akurat ke Red Box) adalah bobot
tertinggi (40 poin, dan jalur ke Absolute Victory). Investasi terbesar di
akurasi deteksi `container` + kontrol presisi payload release ada di sini.

Tie-break: skor sama → waktu penyelesaian tercepat menang.

## 5. Retry

- Diawali aba-aba "RETRY" oleh Team Leader. **Waktu tetap berjalan** (10
  menit total, kontinu, tidak di-pause).
- Titik mulai ulang jika gagal di:
  - Misi 1 → kembali ke Start Zone
  - Misi 2 → kembali ke WP1 (titik mode otonom)
  - Misi 3 → kembali ke WP2 (Red Box boleh dipindah sementara untuk beri
    ruang lepas landas ulang)
  - Misi 4 → kembali ke WP2
  - Misi 5 → kembali ke WP4
- Boleh retry berkali-kali selama masih dalam 10 menit.

**Implikasi desain**: state machine misi harus punya "titik checkpoint"
eksplisit sesuai tabel di atas, bisa di-restart dari checkpoint tanpa
reset seluruh sistem, dan tetap menghitung waktu global.

## 6. Prosedur Lapangan

- Persiapan (set-up): tepat 5 menit di Start Zone (cek sensor, kompas,
  kesiapan wahana) sebelum aba-aba "GO".
- Durasi misi: 10 menit total sejak "GO", termasuk semua retry.

## 7. Keselamatan (wajib ada di software/hardware)

- **Emergency Stop** button yang terlihat jelas di wahana.
- **Emergency Landing System (ELS)**: aktif otomatis jika *lost contact*
  > 15 detik → pendaratan vertikal otomatis darurat.
- **Geofencing** wajib aktif.
- Navigation lights minimum merah & hijau (visibilitas visual).
- Anggota tim wajib pakai helm pengaman di lapangan.
- Sumber daya: motor elektrik saja, dilarang keras mesin berbahan bakar.
- Frekuensi yang diizinkan (radio, umum untuk semua divisi):
  - Telemetry: UHF 433MHz / S-Band 2.4 & 5.8GHz / 4G LTE, mode spread
    spectrum wajib.
  - Video: sama seperti di atas.
  - Daya pancar maks: UHF 433MHz ≤200mW; S-Band ≤1W.

## 8. Spesifikasi Wahana

- Berat lepas landas < 25 kg (mengacu Permenhub PM 37/2020).
- Tidak ada batas dimensi eksplisit, tapi harus proporsional agar bisa
  bermanuver aman di Triple Gate (lorong 2m).
- Wajib punya mekanisme release (gripper/pengait) untuk First Aid Kit,
  mampu lepas otonom saat wahana terdeteksi tepat di atas Drop Point.
- Wajib kamera depan dan/atau bawah untuk kenali objek, WP (ArUco), lokasi
  paket.
- Wajib sensor ketinggian (altimeter/rangefinder) + collision avoidance
  untuk terbang aman di area indoor/GPS-denied.

## 9. Ketentuan Tim & Tahapan

- 1 tim per PT, 3 mahasiswa aktif (bukan S2/S3) + 1 dosen pembimbing + maks
  3 pit crew tambahan.
- Tahapan: Seleksi Proposal → Seleksi Wilayah (daring, hanya Misi 1–3, area
  lapangan 15×15 m, ranking berdasar waktu tercepat + skor) → Final (luring
  di IKN, 2 tim per game, grup lalu sistem gugur).

## 10. Hal yang Perlu Dikonfirmasi ke Panitia/Juknis Final

Beberapa detail di dokumen ambigu/berpotensi berubah pada juknis final —
**jangan asumsikan, cross-check saat juknis final terbit**:
- Penomoran waypoint (WP3 tidak dijelaskan eksplisit muncul di antara
  WP2 dan WP4 pada garis besar pertandingan §3.3.4, tapi disebut di
  §3.3.11.2 "sampai misi 3 (WP3)").
- Definisi persis kriteria "berhasil" WP4 vs deteksi garis hitam di Misi 4.
- Dimensi pasti area GPS-denied indoor vs outdoor pada Final di lapangan
  60×80m (lihat Gambar 9–11 di dokumen asli untuk denah pasti).
