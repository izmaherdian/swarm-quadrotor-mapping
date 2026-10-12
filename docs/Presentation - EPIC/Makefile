# ===========================================================================
#  Slide EPIC 2026. Satu sumber (main.tex), tiga target.
#
#    make           main.pdf        SATU berkas untuk semua: disubmit,
#                                   dibaca di mana pun, dan dipresentasikan
#                                   lewat pdfpc. Video = poster yang bisa
#                                   diklik (run:); catatan pembicara dan
#                                   hitung mundur 20 menit ada di metadata.
#    make pptx      main.pptx       cadangan untuk laptop panitia. Tiap halaman
#                                   jadi gambar, MP4 asli ditempel di atasnya.
#    make media     potong ulang klip video + tulis slowfactors.tex
#    make all       ketiganya
#    make clean     buang berkas antara (bukan klip)
#
#  Angka berasal dari ../Full Paper - EPIC/generated/numbers.tex -- file yang
#  SAMA dengan paper. Makro yang hilang menggagalkan kompilasi.
# ===========================================================================
PY      := ../../.venv/bin/python
PAPER   := ../Full Paper - EPIC
MEDIA   := ../../media/figures
PROG    := ../Progress/figures
TEXINPUTS := .:$(PAPER)/generated:$(PAPER)/figures:$(MEDIA):$(PROG):
LATEXMK := TEXINPUTS="$(TEXINPUTS)" latexmk -pdf -interaction=nonstopmode -halt-on-error

SRC := main.tex epicbeamer.sty slowfactors.tex
CLIPS := assets/clips_manifest.json

.PHONY: all pdf pptx media clean distclean check budget
all: pdf pptx

pdf: main.pdf
main.pdf: $(SRC) $(CLIPS)
	$(LATEXMK) -jobname=main main.tex

pptx: main.pptx
main.pptx: main.pdf $(CLIPS)
	$(PY) tools/pdf_to_pptx.py main.pdf $@

media $(CLIPS):
	$(PY) tools/build_media.py

slowfactors.tex: $(CLIPS)

check: main.pdf
	@echo "halaman : $$(pdfinfo main.pdf | awk '/^Pages/{print $$2}')"
	@echo "ukuran  : $$(du -h main.pdf | cut -f1)"
	@grep -c '^\\begin{frame}\|^\\sectionframe' main.tex | xargs echo "frame   :"
	@echo "tautan video (run:) :" \
	  $$(mutool show main.pdf grep 2>/dev/null | grep -c '/Launch')
	@echo "catatan pdfpc       :" \
	  $$(mutool show main.pdf grep 2>/dev/null | grep -c '/Subtype/Text') "(anotasi tersembunyi, /F 6)"

clean:
	latexmk -C -jobname=main >/dev/null 2>&1 || true
	rm -f *.aux *.log *.nav *.out *.snm *.toc *.fls *.fdb_latexmk *.vrb

distclean: clean
	rm -f main.pdf main.pptx

# Angka anggaran ada di main.log, yang ikut terhapus 'make clean'. Jadi target
# ini mengompilasi sendiri kalau log-nya tidak ada, bukan gagal dengan pesan
# yang membingungkan.
budget:
	@test -f main.log || $(LATEXMK) -jobname=main main.tex >/dev/null 2>&1
	@t=$$(grep -o 'TALK-BUDGET-TENTHS=[0-9]*' main.log | tail -1 | cut -d= -f2); \
	 if [ -z "$$t" ]; then echo "anggaran: TIDAK TERCATAT (jalankan make pdf)"; exit 1; fi; \
	 printf "anggaran bicara : %d.%d menit" $$((t/10)) $$((t%10)); \
	 if [ "$$t" = "270" ]; then echo "  OK"; else echo "  SEHARUSNYA 27.0 -- perbaiki \\mins{} di main.tex"; exit 1; fi
