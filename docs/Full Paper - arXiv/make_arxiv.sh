#!/usr/bin/env bash
# Build preprint arXiv + kemas source siap upload.
#   ./make_arxiv.sh        -> main.pdf (pratinjau) + arxiv_upload.tar.gz
# arXiv tidak menjalankan bibtex, jadi main.bbl ikut dikemas; references.bib tidak.
set -euo pipefail
cd "$(dirname "$0")"

latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex >/dev/null
if grep -q -E "undefined|Citation .* undefined" main.log; then
    echo "[gagal] ada referensi/sitasi undefined, cek main.log" >&2; exit 1
fi

cat > 00README.json <<'JSON'
{
  "process": {"compiler": "pdflatex"},
  "sources": [{"filename": "main.tex", "usage": "toplevel"}]
}
JSON

tar -czf arxiv_upload.tar.gz 00README.json main.tex main.bbl aipnum4-1-epic.bst \
    generated/numbers.tex generated/concept_numbers.tex generated/table_results.tex \
    figures/*.pdf figures/*.png

# uji seperti arXiv: ekstrak ke folder kosong, pdflatex dua kali TANPA bibtex
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
tar -xzf arxiv_upload.tar.gz -C "$T"
( cd "$T" && pdflatex -interaction=nonstopmode -halt-on-error main.tex >/dev/null \
          && pdflatex -interaction=nonstopmode -halt-on-error main.tex >/dev/null )
grep -q -E "undefined" "$T/main.log" && { echo "[gagal] uji ekstrak: ada undefined" >&2; exit 1; }
echo "ok: arxiv_upload.tar.gz ($(du -h arxiv_upload.tar.gz | cut -f1)), $(pdfinfo "$T/main.pdf" | awk '/Pages/{print $2}') halaman"
