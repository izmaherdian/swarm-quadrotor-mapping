# Preprint arXiv — EPIC 2026 (ABS-24)

Versi preprint dari paper *Fault-Tolerant Swarm Quadrotor Control for Non-Convex
Geodetic Mapping*. Isinya sama dengan naskah final ke panitia
(`../Full Paper - EPIC/EPIC2026_FullPaper_Izma.docx`, v2 22 Sep 2026). Bedanya hanya
tiga: tidak ada mode anonim, ada catatan preprint + lisensi di margin atas halaman 1
(syarat preprint AIP), dan link playlist ditulis lengkap.

Folder ini **salinan mandiri**. Kalau naskah EPIC berubah, perubahannya harus disalin
manual ke sini.

## Build dan paket upload

```bash
./make_arxiv.sh
```

Skrip ini menghasilkan `main.pdf` (pratinjau) dan `arxiv_upload.tar.gz`. Isi tarball:
`main.tex`, `main.bbl`, `.bst`, `generated/`, `figures/`, dan `00README.json`.
Setelah dikemas, skrip mengekstraknya ke folder kosong dan meng-compile dengan
`pdflatex` dua kali **tanpa bibtex**, sama seperti arXiv. Kalau ada referensi
undefined, skrip gagal.

## Upload ke arXiv

1. *Start New Submission*, lalu pilih lisensi **CC BY-NC-SA 4.0** (memenuhi syarat minimal NC yang diminta
   AIP untuk preprint).
2. Upload `arxiv_upload.tar.gz` dan cek pratinjau PDF dari arXiv.
3. Metadata:
   - Kategori utama **cs.RO**, cross-list **eess.SY**.
   - Comments: `8 pages, 9 figures, 2 tables. Presented at the 5th Engineering Physics
     International Conference (EPIC) 2026; submitted to AIP Conference Proceedings.
     Code: https://github.com/izmaherdian/swarm-quadrotor-mapping`
   - Penulis (urutan): Izma Alhazmi Herdian, Sulthan Naufal Affan, Estiyanti Ekawati,
     Faqihza Mukhlish.
4. Pastikan **semua co-author sudah setuju** sebelum submit.

## Setelah AIP terbit

- Isi metadata arXiv: *Journal-ref* `AIP Conf. Proc. <vol>, <no> (<tahun>)` dan
  *DOI* `10.1063/5.0…`. Cukup lewat *Replace*/metadata, PDF tidak perlu diganti.
- Kalau naskah berubah setelah review, upload v2 dari LaTeX sendiri (*accepted
  manuscript* boleh langsung).
- **Jangan** upload PDF hasil layout AIP sebelum 12 bulan setelah terbit.

Kebijakan AIP: https://publishing.aip.org/resources/researchers/rights-and-permissions/sharing-content-online/
