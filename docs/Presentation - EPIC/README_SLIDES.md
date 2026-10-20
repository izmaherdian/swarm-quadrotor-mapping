# Slide presentasi EPIC 2026

Deck ditulis di **Beamer**. Satu sumber (`main.tex`), dua keluaran:

| Berkas | Untuk apa |
|---|---|
| `main.pdf` (2,6 MB) | **satu berkas untuk semua**: disubmit ke panitia, dibaca di mana pun, DAN dipresentasikan lewat pdfpc |
| `main.pptx` (60 MB) | cadangan kalau harus pakai laptop panitia — MP4 asli tertanam di atas tiap halaman yang sudah dirender |

`main.pdf` membawa tiga hal sekaligus, dan dua di antaranya tidak terlihat di
pembaca PDF biasa:

* **video** — poster frame yang bisa diklik, sebagai tautan `run:`. Di pembaca
  biasa membuka pemutar sistem; di pdfpc diputar di tempat.
* **catatan pembicara** — 57 anotasi `/Subtype/Text` ber-flag `/F 6` (Hidden).
  pdfpc membacanya; pembaca lain tidak menggambarnya sama sekali.
* **hitung mundur 27 menit** (`duration=27, lastminutes=3`) di metadata XMP.

Semuanya dari paket LaTeX `pdfpc` dengan opsi `overridenote`, yang membuat
`\note{}` bawaan Beamer langsung jadi catatan pdfpc — tidak ada perintah baru.

```
 1 – 52   bicara, tepat 27,0 menit  (44 slide isi + 8 pembatas seksi)
53        pembatas Backup
54        "Questions we expect"  — paket tanya jawab, BUKAN bagian talk
55 – 65   satu slide jawaban per pertanyaan (Q1–Q11)
66 – 82   backup B1–B17: tabel gain penuh, detail, event upset, 7 misi penuh
```

### Narasi (`NARASI.md`)

Setiap slide bicara punya `\narasi{EN}{ID}{arahan}` di `main.tex` — satu sumber:

* **EN** — naskah yang diucapkan, tampil di layar presenter pdfpc; arahan lama
  (durasi, apa yang ditunjuk) menyusul dalam kurung siku.
* **ID** — terjemahannya, hanya di `NARASI.md`, tidak tercetak di PDF.

`make narasi` mengekspor keduanya ke `NARASI.md` dengan angka dari
`numbers.tex`, dan memperingatkan kalau naskah sebuah slide lebih panjang dari
jatah `\mins{}`-nya (patokan 140 kata/menit). Total ≈ 3600 kata untuk 27 menit.

### Kalau slot-mu ternyata 20 menit

Sembunyikan tujuh slide ini — semuanya pendukung, bukan rantai utamanya, dan
totalnya 7,0 menit:

| Slide | Menit | Isi |
|---|---|---|
| 11 | 0,7 | kontrak I/O low level (diagram kaskadenya pindah ke lisan) |
| 14 | 0,8 | Langkah 2, 12 state → 4 rantai |
| 20 | 0,6 | gain domain frekuensi (ada juga di Q5) |
| 30 | 0,5 | kontrak I/O high level |
| 32 | 0,6 | Langkah 14, jarak lajur dan sel Voronoi |
| 45 | 0,4 | coverage vs waktu |
| 48 | 0,6 | torque vs RMSE (ada juga di tabel hasil) |

Sisanya 37 slide isi, 20,0 menit, dan rantai Langkah 1–15 tetap utuh kecuali
Langkah 2 dan 14.

Talk berakhir di slide 52 (*Thank you*). Paket Q&A sengaja ditaruh **di belakang
pembatas Backup** — lompati ke sana hanya kalau ada yang bertanya.

## Alur: kontrak I/O per lapis, lalu rumus diturunkan satu per satu

Slide 1 (diagram `\diagarch` di kanan judul) dan slide 8–9 memberi gambaran
tiga lapis; slide 2 memutar montase empat misi sebagai gambaran umum. Setelah itu **tiap lapis dibuka dengan
slide kontraknya sendiri** — apa yang **masuk beserta satuannya**, apa yang
**dihitung**, apa yang **keluar beserta satuan dan efeknya**:

| Slide | Lapis | Masuk → Keluar |
|---|---|---|
| 11 | Low | $\mathbf{p}^\star$ [m] → empat kecepatan rotor $\Omega$ [rad/s] |
| 22 | Mid | $\mathbf{u}_\text{des}$ [m/s] + peta + tetangga → $\mathbf{u}^\star$ [m/s] |
| 30 | High | polygon [m²] + pose + grid → satu lajur, dua ujung [m] |

Keluaran satu lapis **persis** masukan lapis berikutnya, dan slide 11 juga
memuat diagram blok kaskade PID dengan satuan di tiap panah.

Di antara kontrak-kontrak itu, rantainya diturunkan sebagai **Langkah 1–13**,
naik dari low level ke high level:

| | Langkah | Yang diturunkan |
|---|---|---|
| **Low** | 1 | gaya horizontal hanya dari thrust yang dimiringkan → $a_{\max}=g\tan\theta_{\max}$ |
| | 2 | 12 state nonlinier → dilinierkan di hover → 4 rantai terdekopel |
| | 3 | tiap lapis = integrator ganda, diaugmentasi integrator masukan |
| | 4 | **ARE** (LQR) dan **HARE** (H∞) di atas plant teraugmentasi yang sama |
| | 5 | **Γ⁺** memproyeksikan $K_a$ jadi $K_p,K_i,K_d$ — plus hukum PID yang benar-benar jalan |
| | 6 | PID luar + referensi ber-lead → runtuh jadi plant kecepatan orde satu |
| | 7 | empat konstanta: $k_v$, $a_{\max}$, $T_\text{lead}$, $v_c$ — dari gain, bukan tuning |
| **Mid** | 8 | integrasikan plant → jarak berhenti eksak $D(s)$ |
| | 9 | tambah dead time → syarat keselamatan jadi satu pertidaksamaan |
| | 10 | selesaikan untuk kecepatan → barrier $\varphi(h)$ |
| | 11 | satu rintangan → satu baris linear pada kecepatan perintah |
| | 12 | tumpuk barisnya → kecepatan aman = proyeksi (QP) |
| **High** | 13 | dari $\varphi(h)$ → radius keep-out 1,30 m yang dipakai perencana |
| | 14 | dari radius sensor → jarak lajur; dari region → sel Voronoi |
| | 15 | saat satu agen mati, geometri yang sama dijalankan ulang |

Slide 20 menutup seksi low level dengan **bukti domain frekuensi** (‖G‖∞,
σ_x, ω_c, PM/GM), dan slide 48 di bagian hasil menampilkan **torque effort vs
RMS lateral** — scatter yang merangkum trade-off-nya. Tabel gain lengkap
(K_p/K_i/K_d keempat loop, Q, R, γ) ada di backup.

Langkah 3–5 memakai state space yang **benar-benar disolusikan kodenya**
(`solver_pid_lqr.py`), bukan model 12-state yang tidak dipakai: tiap lapis
adalah $A=\left[\begin{smallmatrix}0&1\\0&0\end{smallmatrix}\right]$,
$B=\left[\begin{smallmatrix}0\\b\end{smallmatrix}\right]$,
diaugmentasi $A_\text{aug}=\left[\begin{smallmatrix}A&B\\0&0\end{smallmatrix}\right]$
sehingga desainnya pada $\dot u$ — dan di situlah suku $K_i$ lahir.

Intinya satu kalimat: **tidak ada angka di lapis atas yang disetel tangan.**

Tiga persamaan kunci punya diagram TikZ-nya sendiri di `diagrams.tex`, ditulis
untuk audiens yang bukan orang kendali — tiap gambar menerjemahkan satu
persamaan jadi sesuatu yang bisa ditunjuk:

| Perintah | Dipakai di | Menjelaskan |
|---|---|---|
| `\diagfbd` | Langkah 1 | diagram benda bebas: thrust diurai jadi $T\cos\theta$ dan $T\sin\theta$ |
| `\diagbudget` | Langkah 9 | anggaran jarak: $sT_d$ + pengereman harus muat di dalam $h$ |
| `\diagrow` | Langkah 11 | hanya komponen **mendekat** yang dibatasi; berbelok tetap bebas |
| `\diagio` | slide 11, 22, 30 | strip kontrak: masuk (satuan) → dihitung → keluar (satuan) |
| `\diagcascade` | slide 11 | kaskade PID low level, satuan di tiap panah, 250 Hz |
| `\diagarch` | slide 1 | tiga lapis + plant dalam satu kolom, dengan umpan balik odometri/LiDAR |

Warnanya memakai palet tema, jadi tidak ada warna baru yang diperkenalkan.

## Angka berasal dari file yang sama dengan paper

```latex
\input{numbers}   % ../Full Paper - EPIC/generated/numbers.tex
```

274 makro, dihasilkan `swarm_ws/analysis/skema5_paper_analysis.py`. **Tidak ada
satu pun angka yang diketik di `main.tex`** — makro yang tidak ada menggagalkan
kompilasi. Jalankan ulang analisisnya, lalu `make`, dan seluruh deck ikut berubah.

`slowfactors.tex` (tiga faktor gerak lambat) dihasilkan `tools/build_media.py`.
Nomor slide di indeks Q&A memakai `\pageref`, jadi tetap benar kalau frame bergeser.

**Anggaran 27 menit ikut diverifikasi.** Tiap frame bicara punya `\mins{N}`
(N persepuluh menit). LaTeX menjumlahkannya ke sebuah counter dan mencetaknya ke
log; `make budget` gagal kalau totalnya bukan 270.

## Cara membangun

```bash
cd "docs/Presentation - EPIC"
make            # main.pdf  (sekaligus untuk pdfpc)
make pptx       # main.pptx
make all        # pdf + pptx + NARASI.md
make budget     # verifikasi anggaran 27,0 menit
make check      # ringkasan halaman/ukuran/anotasi
make media      # potong ulang klip + kecilkan figure beresolusi berlebih
make narasi     # NARASI.md dari \narasi{} di main.tex
make clean
```

`TEXINPUTS` diurus Makefile (menunjuk ke `generated/`, `figures/`,
`media/figures/`, `Progress/figures/`), jadi tidak ada berkas yang perlu disalin.

### Presentasi

```bash
pdfpc main.pdf
```

pdfpc 4.7.0 terpasang dan tertaut GStreamer (`avdec_h264` tersedia). Slide di
proyektor; catatan, timer hitung mundur dan pratinjau slide berikutnya di layar
laptop — semuanya dari metadata di dalam `main.pdf`, tanpa berkas tambahan.

> **Pemutaran video perlu satu klik darimu untuk dipastikan.** `main.pdf` dibuka
> pdfpc tanpa error dan berisi 15 tautan `run:`; manpage pdfpc menyebut
> *hyperlink launch* sebagai mekanisme yang didukung. Kalau ternyata video tidak
> mau diputar, ganti satu baris di awal `main.tex` ke `\def\videomode{movie}`
> dan `make` ulang: itu memakai anotasi `/Movie` dari paket `multimedia`.

Berkas `assets/clips/*.mp4` **harus ikut di sebelah PDF-nya** — tautan `run:`
merujuk berkas luar, tidak menanamnya. Kirim `main.pptx` kalau perlu satu berkas.

## Warna dan huruf

Palet coklat hangat, didefinisikan sekali di `epicbeamer.sty`:

| Peran | Warna |
|---|---|
| Blok judul, rel pembatas seksi | `#331106` Dark Brick Brown |
| Judul frame, angka besar | `#502611` Deep Rust Brown |
| Aksen, label seksi, garis | `#AA7158` Warm Clay Tan |
| Garis kotak, banding tabel | `#E7BAA5` Soft Peach Nude |
| Isi kotak callout | `#F6D0B9` Light Skin Beige |

**HEX yang dipakai, bukan RGB.** Dua warna yang kamu kirim tidak konsisten:
Deep Rust Brown (`#502611` = 80,38,17 vs RGB 93,38,17) dan Warm Clay Tan
(`#AA7158` = 170,113,88 vs RGB 170,118,66 — ini selisih yang terlihat).

**Biru/oranye pembeda kontroler sengaja TIDAK diubah** (`#1F77B4` / `#DD7A2E`):
warna itu ada di dalam figure paper, jadi legenda gambar dan teks slide harus
merujuk warna yang sama.

Huruf: tiga genre dari satu superfamily Source — **Source Serif** untuk judul,
angka besar dan pembatas; **Source Sans** untuk isi; **Source Code** untuk
besaran bernilai (`\num{...}`), supaya angka terukur terlihat beda dari simbol.

## Isi folder

| Berkas | Isinya |
|---|---|
| `main.tex` | **deck-nya** — klaim, bullet, penurunan, catatan pembicara, `\mins{}` |
| `epicbeamer.sty` | tema: warna, huruf, template frame, `\bignum`, `\callout`, `\videoposter`, crop panel |
| `diagrams.tex` | diagram TikZ: Langkah 1, 9, 11, kontrak I/O, kaskade, arsitektur slide judul |
| `Makefile` | target di atas, plus verifikasi anggaran waktu |
| `tools/build_media.py` | potong klip ffmpeg, kecilkan figure berlebih, tulis `slowfactors.tex` |
| `tools/pdf_to_pptx.py` | bungkus halaman PDF + tempel MP4 (hanya untuk target `pptx`) |
| `tools/export_narasi.py` | tulis `NARASI.md` (EN + ID per slide) dari `\narasi{}` di `main.tex` |
| `NARASI.md` | naskah bicara hasil ekspor — jangan diedit langsung |
| `tools/facts.py` | pemuat angka + peta waktu sim → video untuk pemotongan klip |

Python hanya menyentuh **media**, tidak pernah tata letak. Matematika dirender
Beamer; figure dipotong dari PDF vektor paper lewat `viewport`.

## Gaya, kalau mengedit

1. **Judul frame adalah KLAIM atau LANGKAH**, bukan topik.
2. **Bullet hanya pendukung** — maksimal empat, pendek. Uraian panjang ke
   `\note{}`, yang muncul di layar presenter pdfpc.
3. **Setiap frame bicara butuh `\mins{N}`**, dan totalnya harus 270.
4. Teks catatan masuk **apa adanya** ke string anotasi PDF: tulis `a max`,
   bukan `a\_max` (yang muncul sebagai `\protect\T1\textunderscore`), dan
   angka sebagai `\CovMin\space`, bukan `\CovMin{}` (yang ikut tercetak `{}`).
5. Node TikZ yang memuat `\\` butuh opsi `align`, dan `pic {angle = A--O--B}`
   menyapu berlawanan arah jarum jam — urutan titiknya menentukan apakah kamu
   dapat 26° atau 334°.

## Hal yang perlu kamu putuskan

* **Tenggat.** PDF guidelines menulis **15 Oktober 2026**; situs resmi
  menampilkan **22 Oktober**. Perlu dikonfirmasi ke panitia.
* **Commit.** Sumbernya kecil dan layak masuk git. `main.pptx` dan
  `assets/clips/` bisa dibangun ulang, jadi `.gitignore` tidak disentuh. Kalau
  klip ikut di-commit, tambahkan `*.mp4 binary` di `.gitattributes`.
* **Rekaman video** EPIC: rekam layar sambil menjalankan `pdfpc main.pdf`.

## Isi yang sengaja dipertahankan di bagian bicara

* run rect/PID-LQR yang gagal (slide 41, angka-besar 44, barisnya di tabel 43)
* N = 1 per konfigurasi (slide 36)
* tidak ada PX4 / tidak ada estimator (slide 9 dan 11, dan Q2)
* keunggulan H∞ sebagian besar penolakan offset angin, bukan atenuasi L2 murni
  (slide 46 dan angka-besar 47)
* pelindung rotor adalah asumsi, tabrakan antar-agen tidak pernah dideteksi —
  tidak ada klaim "nol tabrakan" (slide 49)
