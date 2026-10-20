"""Ekspor naskah bicara deck ke NARASI.md.

Sumbernya \\narasi{EN}{ID}{arahan} di main.tex -- satu-satunya tempat narasi
ditulis. Makro angka (\\CovMin{} dst.) diganti nilainya dari numbers.tex lewat
facts.py, jadi naskah tidak pernah mengarang angka. Makro yang tidak dikenal
menggagalkan ekspor, persis seperti di LaTeX.

Nomor slide = nomor halaman PDF: tiap \\begin{frame} dan \\sectionframe di
main.tex adalah satu halaman (deck tidak memakai overlay). Hanya bagian bicara
yang diekspor; berhenti di pembatas Backup.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

from facts import Facts, HERE

DECK = HERE.parent
BACKUP = "% ======================================================== Backup"
WPM = 140   # laju bicara yang dipakai untuk memeriksa \mins{}


def group(s: str, i: int) -> tuple[str, int]:
    """Isi {...} yang dimulai di s[i] == '{', plus indeks sesudah '}'."""
    assert s[i] == "{", s[i:i + 40]
    d, j = 0, i
    while True:
        d += (s[j] == "{") - (s[j] == "}")
        j += 1
        if d == 0:
            return s[i + 1:j - 1], j


def clean(t: str, F: Facts) -> str:
    def num(m: re.Match) -> str:
        return F.n(m.group(1))
    t = t.replace(r"\space", "")
    t = re.sub(r"\\([A-Za-z]+)\{\}", num, t)
    for a, b in [(r"$H_\infty$", "H∞"), (r"\hinf", "PID-H∞"), (r"\lqr", "PID-LQR"),
                 (r"\%", "%"), (r"\_", "_"), (r"\&", "&"), (r"\,", " "),
                 (r"\textsuperscript{2}", "²"), (r"\textdegree", "°"),
                 (r"\varphi", "φ"), (r"\gamma", "γ"), (r"H_\infty", "H∞"),
                 (r"\textperiodcentered{}", "·"), ("$", ""), ("---", "—"), ("--", "–"), ("``", "“"),
                 ("''", "”"), ("\\\\", " ")]:
        t = t.replace(a, b)
    t = re.sub(r"\\([A-Za-z]+)", lambda m: F.n(m.group(1)), t)
    return re.sub(r"\s+", " ", t).strip()


def main() -> int:
    F = Facts()
    src = (DECK / "main.tex").read_text()
    talk = src[:src.index(BACKUP)]
    talk = "\n".join(l for l in talk.splitlines() if not l.lstrip().startswith("%"))

    items, page = [], 0
    for m in re.finditer(r"\\begin\{frame\}(\[[^\]]*\])?|\\sectionframe\{", talk):
        page += 1
        if m.group(0).startswith("\\sectionframe"):
            num, j = group(talk, m.end() - 1)
            title, _ = group(talk, j)
            items.append(dict(page=page, section=f"{num} · {clean(title, F)}"))
            continue
        end = talk.index("\\end{frame}", m.end())
        body = talk[m.end():end]
        if body.startswith("{"):
            title, _ = group(body, 0)
        elif "\\bignum{" in body:
            title, _ = group(body, body.index("\\bignum{") + 7)
        else:
            title = "Title" if page == 1 else "Thank you"
        mins = int(re.search(r"\\mins\{(\d+)\}", body).group(1))
        k = body.index("\\narasi{") + 7
        en, k = group(body, k)
        k = body.index("{", k)
        idn, k = group(body, k)
        k = body.index("{", k)
        cue, _ = group(body, k)
        items.append(dict(page=page, title=clean(title, F), mins=mins,
                          en=clean(en, F), id=clean(idn, F), cue=clean(cue, F)))

    frames = [x for x in items if "en" in x]
    total = sum(x["mins"] for x in frames)
    words = sum(len(x["en"].split()) for x in frames)
    out = ["# Narasi presentasi EPIC 2026",
           "",
           "Dihasilkan `tools/export_narasi.py` dari `\\narasi{}` di `main.tex` —"
           " **jangan diedit di sini**, edit `main.tex` lalu `make narasi`.",
           "Angka diambil dari `numbers.tex`, file yang sama dengan paper.",
           "",
           f"Slide 1–{items[-1]['page']} · {len(frames)} slide bicara · "
           f"{total / 10:.1f} menit · {words} kata Inggris "
           f"(≈ {words / (total / 10):.0f} kata/menit).",
           "",
           "Pembatas seksi tidak punya narasi: tahan sebentar, lalu klik lanjut.",
           ""]
    for x in items:
        if "section" in x:
            out += [f"## Slide {x['page']} — pembatas {x['section']}", ""]
            continue
        n = len(x["en"].split())
        out += [f"## Slide {x['page']} — {x['title']}",
                "",
                f"*{x['mins'] / 10:.1f} menit · {n} kata*",
                "",
                f"**EN** {x['en']}",
                "",
                f"**ID** {x['id']}",
                "",
                f"> Arahan: {x['cue']}",
                ""]
        if n > x["mins"] / 10 * WPM * 1.15:
            print(f"[warn] slide {x['page']}: {n} kata untuk {x['mins'] / 10} menit",
                  file=sys.stderr)
    (DECK / "NARASI.md").write_text("\n".join(out))
    print(f"ditulis NARASI.md: {len(frames)} slide, {total / 10:.1f} menit, {words} kata")
    return 0


if __name__ == "__main__":
    sys.exit(main())
