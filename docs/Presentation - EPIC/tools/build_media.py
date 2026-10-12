"""Bangun aset media deck: potong klip highlight dengan ffmpeg.

Dipanggil Makefile. Tidak menyentuh tata letak slide -- itu urusan
main.tex. Juga menulis slowfactors.tex, tiga makro faktor gerak lambat
yang diturunkan dari recording_rtf di media/scheme5/clips.json.

Dua rezim pemotongan highlight, dipilih dari rentang waktu SIMULASI yang mau
ditampilkan — bukan dari selera:

* Momen naratif (injeksi kegagalan, detour rintangan) mencakup 15-25 s waktu sim.
  Dipotong langsung dari klip 8x yang sudah dipublikasikan -> panjangnya pas,
  tanpa kehilangan kualitas.
* Momen rapat (tabrakan, serempetan) hanya 2-4 s waktu sim. Pada 8x itu cuma
  ~0,4 s dan tak terbaca, jadi dipotong dari screencast 1080p asli pada 2x
  -> gerak lambat ~8x relatif waktu sim, halus karena tiap frame nyata.

Memperlambat klip 8x yang sudah jadi BUKAN pilihan: hasilnya ~7,5 fps efektif.
"""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

from facts import Facts, HERE, REPO

DECK = HERE.parent
ASSETS = DECK / "assets"
CLIP_DIR = ASSETS / "clips"
POSTER_DIR = ASSETS / "posters"

# --------------------------------------------------------------------- klip
# src_window: jendela detik pada screencast 1080p asli, dari figures/frames/
#   index.json — jangkar eksak yang sudah dipakai paper untuk Gambar 7.
# sim_window: jendela waktu simulasi, dipetakan lewat jam dinding coordinator.log.
HIGHLIGHTS = [
    dict(id="h1_fault_kill1", run="rect_hinf", mode="clip", sim_window=(31.0, 56.0),
         title="Agent 4 lost -> cell merged, lanes handed to helpers"),
    dict(id="h2_fault_kill2", run="rect_hinf", mode="clip", sim_window=(52.0, 77.0),
         title="Agent 7 lost -> second reallocation, three helpers"),
    dict(id="h6_obstacle_detour", run="rect_hinf", mode="clip", sim_window=(82.0, 102.0),
         title="LiDAR confirms a cylinder -> remaining lanes re-planned"),
    dict(id="h7_v2v_closest", run="l_shape_lqr", mode="clip", sim_window=(37.0, 53.0),
         title="Closest drone-to-drone pass of the campaign"),
    dict(id="h3_crash_rect_lqr", run="rect_lqr", mode="source", speed=2.0,
         src_window=(846.0, 880.0), sim_window=(74.0, 80.0),
         title="Contact and loss: rect, PID-LQR, iris_2"),
    dict(id="h4_upset_plus_hinf", run="plus_hinf", mode="source", speed=2.0,
         src_window=(700.0, 722.0), sim_window=(77.5, 83.0),
         title="Graze and recovery: plus, PID-H-inf, iris_2"),
    dict(id="h5_upset_plus_lqr", run="plus_lqr", mode="source", speed=2.0,
         src_window=(617.0, 639.0), sim_window=(79.2, 84.7),
         title="Same cylinder, PID-LQR: smaller upset, also recovered"),
]

# Framing klip terbitan, disalin dari swarm_ws/analysis/skema5_video_clips.py
# (crop pada frame 1920x1080, lalu diskalakan ke lebar 1280 -> 1280x678).
PUBLISHED_CROP = "1870:990:50:60"
LABEL_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

FFMPEG_V = ["-c:v", "libx264", "-crf", "22", "-preset", "medium", "-pix_fmt", "yuv420p",
            "-profile:v", "high", "-level", "4.0", "-r", "30", "-fps_mode", "cfr",
            "-an", "-movflags", "+faststart"]


def _run(cmd: list[str]) -> None:
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError(f"ffmpeg gagal: {' '.join(cmd[:8])}...\n{p.stderr[-1500:]}")


def probe(path: Path) -> dict:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
         "stream=width,height,duration,nb_frames,pix_fmt", "-of", "json", str(path)],
        capture_output=True, text=True, check=True).stdout
    s = json.loads(out)["streams"][0]
    return dict(w=int(s["width"]), h=int(s["height"]), dur=float(s["duration"]),
                pix=s["pix_fmt"])


def poster_for(video: Path, dest: Path, at: float = 0.4) -> Path:
    _run(["ffmpeg", "-y", "-v", "error", "-ss", f"{at:.2f}", "-i", str(video),
          "-frames:v", "1", "-q:v", "3", str(dest)])
    return dest


def build_clips(F: Facts, force: bool = False) -> dict:
    """Hasilkan klip highlight + salin 8 klip misi penuh. Kembalikan manifest."""
    CLIP_DIR.mkdir(parents=True, exist_ok=True)
    POSTER_DIR.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, dict] = {}

    # 1) Delapan misi penuh: salin apa adanya (sudah H.264/yuv420p, siap PowerPoint).
    for run in F.clips:
        dest = CLIP_DIR / f"{run}.mp4"
        if force or not dest.exists():
            shutil.copy2(F.clip_path(run), dest)
        poster = POSTER_DIR / f"{run}.jpg"
        if force or not poster.exists():
            if F.thumb_path(run).exists():
                shutil.copy2(F.thumb_path(run), poster)
            else:
                poster_for(dest, poster)
        info = probe(dest)
        manifest[run] = dict(kind="full_mission", run=run, source=str(F.clip_path(run)),
                             speed_vs_sim="~1x (8x wall clock)", **info)

    # 2) Highlight.
    skipped = []
    for spec in HIGHLIGHTS:
        run, hid = spec["run"], spec["id"]
        dest = CLIP_DIR / f"{hid}.mp4"
        mode, note = spec["mode"], ""
        src = F.source_path(run)

        if mode == "source" and not src.exists():
            # Screencast asli (15,9 GB) ada di luar repo dan tidak masuk git.
            # Jangan diam-diam kehilangan slide: turunkan ke klip 8x yang ada.
            mode, note = "clip", "degraded: screencast 1080p tidak ada, dipotong dari klip 8x"
            skipped.append(hid)

        if force or not dest.exists():
            if mode == "source":
                a, b = spec["src_window"]
                speed = spec["speed"]
                # -t SEBELUM -i: membatasi input yang dibaca. Kalau ditaruh
                # setelah -i ia membatasi durasi OUTPUT, sehingga setpts menyedot
                # sumber sampai EOF dan klip tidak jadi gerak lambat.
                #
                # crop/scale persis seperti skema5_video_clips.py supaya framing
                # sama dengan 8 klip terbitan (dock, top bar dan judul jendela
                # hilang) — tanpa ini close-up terlihat seperti screenshot desktop.
                rtf = F.clips[run]["recording_rtf"]
                slow = (1.0 / speed) / rtf   # berapa kali lebih lambat dari waktu nyata
                label = (f"{run.replace('_', ' ')} | "
                         f"{slow:.1f}x slower than real time")
                vf = (f"crop={PUBLISHED_CROP},scale=1280:-2,setpts=PTS/{speed},"
                      f"drawtext=fontfile={LABEL_FONT}:text='{label}'"
                      f":x=16:y=14:fontsize=24:fontcolor=white"
                      f":box=1:boxcolor=black@0.55:boxborderw=9")
                _run(["ffmpeg", "-y", "-v", "error", "-ss", f"{a:.3f}",
                      "-t", f"{b - a:.3f}", "-i", str(src),
                      "-vf", vf, *FFMPEG_V, str(dest)])
            else:
                a = F.clip_s(run, spec["sim_window"][0])
                b = F.clip_s(run, spec["sim_window"][1])
                a = max(0.0, a)
                b = min(F.clip_len(run), b)
                _run(["ffmpeg", "-y", "-v", "error", "-ss", f"{a:.3f}",
                      "-t", f"{b - a:.3f}", "-i", str(F.clip_path(run)),
                      *FFMPEG_V, str(dest)])

        poster = POSTER_DIR / f"{hid}.jpg"
        if force or not poster.exists():
            poster_for(dest, poster)

        info = probe(dest)
        entry = dict(kind="highlight", run=run, title=spec["title"], cut_from=mode,
                     sim_window=spec["sim_window"], note=note, **info)
        if mode == "source":
            entry["source_file"] = str(src)
            entry["source_window_s"] = spec["src_window"]
            entry["speed"] = spec["speed"]
            entry["anchor"] = "figures/frames/index.json (exact)"
        else:
            entry["source_file"] = str(F.clip_path(run))
            entry["clip_window_s"] = [round(F.clip_s(run, spec["sim_window"][0]), 2),
                                      round(F.clip_s(run, spec["sim_window"][1]), 2)]
            entry["anchor"] = "coordinator.log wall clock (derived, ~0.4 s sim)"
        manifest[hid] = entry

    (ASSETS / "clips_manifest.json").write_text(json.dumps(manifest, indent=2))
    return dict(manifest=manifest, degraded=skipped)


# --------------------------------------------------------- makro ke LaTeX
# Faktor gerak lambat tiap close-up relatif WAKTU NYATA, bukan jam dinding
# rekaman: (1/speed) / recording_rtf. Ditulis sebagai makro supaya takarir
# slide tidak pernah mengarang angkanya.
SLOW_MACROS = {"h3_crash_rect_lqr": "SlowHthree",
               "h4_upset_plus_hinf": "SlowHfour",
               "h5_upset_plus_lqr": "SlowHfive"}


def write_slowfactors(F: Facts, manifest: dict, dest: Path) -> None:
    lines = ["% DIHASILKAN OTOMATIS oleh tools/build_media.py -- JANGAN DIEDIT."]
    for clip_id, macro in SLOW_MACROS.items():
        e = manifest[clip_id]
        if not e.get("speed"):
            raise RuntimeError(
                f"{clip_id} tidak dipotong dari sumber 1080p, jadi faktor gerak "
                f"lambatnya tidak terdefinisi. Pasang kembali ~/Videos/Screencasts "
                f"atau sunting takarirnya di main.tex.")
        slow = (1.0 / e["speed"]) / F.clips[e["run"]]["recording_rtf"]
        lines.append("\\newcommand{\\%s}{%.1f}" % (macro, slow))
    dest.write_text("\n".join(lines) + "\n")


# ------------------------------------------------- figure beresolusi berlebih
# Plot sintesis gain berukuran 4500x4200 px. Di slide ia menempati ~56 mm, jadi
# 1200 px sudah lebih dari cukup pada 300 dpi; menyertakan aslinya menambah
# ~1,8 MB ke KEDUA PDF tanpa menambah satu piksel pun yang terlihat.
OVERSIZED = {"grafik_pid_hinf_wind_turbulence":
             "docs/Progress/figures/grafik_pid_hinf_wind_turbulence.png"}


def shrink_figures(F: Facts, force: bool = False) -> None:
    from PIL import Image
    dest_dir = ASSETS / "fig"
    dest_dir.mkdir(parents=True, exist_ok=True)
    for name, rel in OVERSIZED.items():
        dest = dest_dir / f"{name}.png"
        if dest.exists() and not force:
            continue
        src = Path(F.__class__.__module__ and REPO) / rel
        if not src.is_file():
            raise FileNotFoundError(f"figure sumber tidak ada: {src}")
        im = Image.open(src)
        im.thumbnail((1200, 1200), Image.LANCZOS)
        im.save(dest, optimize=True)
        print(f"  {name}: {src.stat().st_size/1e6:.1f} MB -> "
              f"{dest.stat().st_size/1e6:.2f} MB")


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--force", action="store_true", help="potong ulang semua klip")
    args = ap.parse_args()

    F = Facts()
    shrink_figures(F, force=args.force)
    res = build_clips(F, force=args.force)
    if res["degraded"]:
        print(f"[warn] screencast 1080p tidak ada; close-up diturunkan: "
              f"{res['degraded']}")
    write_slowfactors(F, res["manifest"], DECK / "slowfactors.tex")

    full = sum(1 for v in res["manifest"].values() if v["kind"] == "full_mission")
    hi = sum(1 for v in res["manifest"].values() if v["kind"] == "highlight")
    mb_total = sum((CLIP_DIR / f"{k}.mp4").stat().st_size for k in res["manifest"]) / 1e6
    print(f"klip siap: {full} misi penuh + {hi} highlight, {mb_total:.0f} MB")
    print(f"ditulis  : assets/clips_manifest.json, slowfactors.tex")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
