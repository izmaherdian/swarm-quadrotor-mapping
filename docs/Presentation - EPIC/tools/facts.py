"""Fakta terverifikasi untuk deck EPIC 2026.

Semua angka dibaca dari artefak yang dihasilkan skema5_paper_analysis.py.
Tidak ada nilai default: makro yang hilang -> KeyError -> build gagal.
Itu disengaja; proyek ini pernah punya tooling yang menambal data.
"""
from __future__ import annotations

import bisect
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]          # tools/ -> Presentation - EPIC/ -> docs/ -> repo
PAPER = REPO / "docs" / "Full Paper - EPIC"
GEN = PAPER / "generated"
RUNS = REPO / "swarm_ws" / "src" / "swarm_sim" / "results" / "skema5_video_runs"
CLIPS = REPO / "media" / "scheme5"
SCREENCASTS = Path.home() / "Videos" / "Screencasts"

# Folder data per run, sesuai MANIFEST.json.
RUN_DIR = {
    "rect_hinf": "01_rect/pid_hinf",
    "rect_lqr": "01_rect/pid_lqr",
    "l_shape_hinf": "02_l-shape/pid_hinf",
    "l_shape_lqr": "02_l-shape/pid_lqr",
    "u_shape_hinf": "03_u-shape/pid_hinf",
    "u_shape_lqr": "03_u-shape/pid_lqr",
    "plus_hinf": "04_plus/pid_hinf",
    "plus_lqr": "04_plus/pid_lqr",
}

# Frame '*_end' dipakai sebagai jangkar jam-dinding tiap run.
ANCHOR = {k: f"{k}_end" for k in RUN_DIR}


class MissingNumber(KeyError):
    """Dilempar kalau teks slide meminta makro yang tidak ada di numbers.tex."""


class _StrictMap(dict):
    def __missing__(self, key):
        raise MissingNumber(
            f"makro '{key}' tidak ada di numbers.tex/metrics.json — "
            f"jalankan ulang swarm_ws/analysis/skema5_paper_analysis.py "
            f"atau perbaiki teks slide. Build sengaja digagalkan, bukan ditambal."
        )


class Facts:
    def __init__(self) -> None:
        self.numbers = self._load_numbers(GEN / "numbers.tex")
        self.numbers.update(self._load_numbers(GEN / "concept_numbers.tex"))
        metrics = json.loads((GEN / "metrics.json").read_text())
        self.runs = metrics["runs"]
        self.design = metrics["design"]
        self.clips = json.loads((CLIPS / "clips.json").read_text())
        self.frames = json.loads((PAPER / "figures" / "frames" / "index.json").read_text())
        self.manifest = json.loads((RUNS / "MANIFEST.json").read_text())
        self._walls: dict[str, list[float]] = {}

    # ---------------------------------------------------------------- numbers
    @staticmethod
    def _load_numbers(path: Path) -> dict[str, str]:
        if not path.is_file():
            raise FileNotFoundError(f"sumber angka tidak ada: {path}")
        pat = re.compile(r"\\newcommand\{\\([A-Za-z]+)\}\{([^}]*)\}")
        return {m.group(1): m.group(2) for m in pat.finditer(path.read_text())}

    def fmt(self, text: str) -> str:
        """Isi placeholder {Makro}; makro tak dikenal menggagalkan build."""
        if "{" not in text:
            return text
        return text.format_map(_StrictMap(self.numbers))

    def n(self, key: str) -> str:
        return _StrictMap(self.numbers)[key]

    # ------------------------------------------------------- jam sim -> video
    def _status_walls(self, run: str) -> list[float]:
        """Stempel wall-clock tiap baris [STATUS] di coordinator.log.

        Baris [STATUS] ke-i (0-based) = detik misi ke-(i+1); konvensi ini
        diverifikasi terhadap 'Durasi Misi' di tujuh log yang selesai.
        """
        if run not in self._walls:
            log = RUNS / RUN_DIR[run] / "coordinator.log"
            walls = []
            for line in log.read_text(errors="replace").splitlines():
                if "[STATUS]" in line:
                    m = re.search(r"\[(\d+\.\d+)\]", line)
                    if m:
                        walls.append(float(m.group(1)))
            if len(walls) < 2:
                raise RuntimeError(f"{log} tidak punya baris [STATUS] yang cukup")
            self._walls[run] = walls
        return self._walls[run]

    def wall_of_sim(self, run: str, t_sim: float) -> float:
        w = self._status_walls(run)
        x = t_sim - 1.0  # indeks pecahan ke dalam daftar STATUS
        if x <= 0:
            return w[0] + x * (w[1] - w[0])
        i = int(x)
        if i >= len(w) - 1:
            return w[-1] + (x - (len(w) - 1)) * (w[-1] - w[-2])
        return w[i] + (x - i) * (w[i + 1] - w[i])

    def video_s(self, run: str, t_sim: float) -> float:
        """Detik ke dalam screencast 1080p asli untuk waktu simulasi t_sim.

        Screencast berjalan pada jam dinding 1:1, jadi satu jangkar
        (video_s, log_wall_epoch) dari figures/frames/index.json sudah cukup.
        Akurasi terukur ~3 s video sumber (~0,4 s waktu sim) — jauh lebih baik
        daripada peta linear sim->klip, yang meleset ~25 s karena RTF Gazebo
        tidak konstan sepanjang misi.
        """
        a = self.frames[ANCHOR[run]]
        return a["video_s"] + (self.wall_of_sim(run, t_sim) - a["log_wall_epoch"])

    def clip_s(self, run: str, t_sim: float) -> float:
        """Detik ke dalam klip media/scheme5/<run>.mp4 yang sudah dipercepat 8x."""
        c = self.clips[run]
        return (self.video_s(run, t_sim) - c["trim_s"][0]) / c["speedup"]

    # ---------------------------------------------------------------- berkas
    def clip_path(self, run: str) -> Path:
        return CLIPS / f"{run}.mp4"

    def thumb_path(self, run: str) -> Path:
        return CLIPS / "thumbs" / f"{run}.jpg"

    def source_path(self, run: str) -> Path:
        return SCREENCASTS / self.clips[run]["source_video"]

    def clip_len(self, run: str) -> float:
        return float(self.clips[run]["clip_s"])
