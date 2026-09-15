#!/usr/bin/env python3
"""
Ekstrak frame video Skema 5 untuk verifikasi & figure paper.

Waktu video dipetakan dari stempel WALL-clock log coordinator (epoch UNIX, lihat
CLAUDE.md: stempel log = wall, bukan sim) dan waktu mulai rekaman yang tertera
di nama file screencast (zona Asia/Jakarta, UTC+7):

    posisi_video_ms = (epoch_baris_log - epoch_mulai_rekaman) * 1000

Keluaran: docs/Full Paper - EPIC/figures/frames/<run>_<tag>.png (+ index.json)

Pakai:
    cd swarm_ws && python3 analysis/skema5_video_frames.py [--montage-only]
"""
import json
import re
import sys
from datetime import datetime, timezone, timedelta
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / 'swarm_ws' / 'src' / 'swarm_sim' / 'results' / 'skema5_video_runs'
OUT = REPO / 'docs' / 'Full Paper - EPIC' / 'figures' / 'frames'
VIDEO_DIR = Path.home() / 'Videos' / 'Screencasts'
WIB = timezone(timedelta(hours=7))

# run -> (folder, video). Dicocokkan lewat mtime CSV = akhir log = akhir video (MANIFEST.json).
RUNS = {
    'rect_hinf': ('01_rect/pid_hinf', 'Screencast From 2026-09-11 19-20-53.mp4'),
    'rect_lqr': ('01_rect/pid_lqr', 'Screencast From 2026-09-11 20-35-13.mp4'),
    'l_shape_hinf': ('02_l-shape/pid_hinf', 'Screencast From 2026-09-11 21-17-17.mp4'),
    'l_shape_lqr': ('02_l-shape/pid_lqr', 'Screencast From 2026-09-11 22-34-02.mp4'),
    'u_shape_hinf': ('03_u-shape/pid_hinf', 'Screencast From 2026-09-12 11-18-20.mp4'),
    'u_shape_lqr': ('03_u-shape/pid_lqr', 'Screencast From 2026-09-12 12-53-45.mp4'),
    'plus_hinf': ('04_plus/pid_hinf', 'Screencast From 2026-09-12 15-29-13.mp4'),
    'plus_lqr': ('04_plus/pid_lqr', 'Screencast From 2026-09-12 16-06-51.mp4'),
}
RE_WALL = re.compile(r'^\[\w+\] \[(\d+\.\d+)\]')


def video_start_epoch(name):
    m = re.search(r'(\d{4}-\d{2}-\d{2}) (\d{2})-(\d{2})-(\d{2})', name)
    dt = datetime.strptime(f'{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}', '%Y-%m-%d %H:%M:%S')
    return dt.replace(tzinfo=WIB).timestamp()


def status_walls(log_path):
    """Epoch wall dari tiap baris STATUS (indeks = detik sim coordinator)."""
    walls, marks = [], {}
    for line in log_path.read_text(errors='replace').splitlines():
        m = RE_WALL.match(line)
        if not m:
            continue
        w = float(m.group(1))
        if '[STATUS]' in line:
            walls.append(w)
        elif 'AUTO-EXIT' in line and 'auto_exit' not in marks:
            marks['auto_exit'] = w
        elif 'SWARM SUCCESS' in line:
            marks['success'] = w
    return walls, marks


def grab(video, t_ms, dest):
    cap = cv2.VideoCapture(str(video))
    dur_ms = cap.get(cv2.CAP_PROP_FRAME_COUNT) / cap.get(cv2.CAP_PROP_FPS) * 1000.0
    for back in (0, 1000, 2000, 4000, 8000):
        cap.set(cv2.CAP_PROP_POS_MSEC, max(0.0, min(t_ms, dur_ms) - back))
        ok, fr = cap.read()
        if ok:
            cv2.imwrite(str(dest), fr)
            return True, (min(t_ms, dur_ms) - back) / 1000.0
    return False, None


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    index = {}
    for run, (folder, vname) in RUNS.items():
        video = VIDEO_DIR / vname
        if not video.is_file():
            print(f'[skip] video tidak ada: {video}', file=sys.stderr)
            continue
        walls, marks = status_walls(DATA / folder / 'coordinator.log')
        t0 = video_start_epoch(vname)
        wanted = {'end': marks.get('auto_exit', walls[-1])}
        # kejadian upset (detik sim coordinator; dari generated/audit.md, offset CSV-log ~1.5 s)
        if run == 'rect_lqr':
            wanted.update({'contact_m2': walls[74], 'contact': walls[75], 'fall': walls[76]})
        if run in ('plus_hinf', 'plus_lqr'):
            k = 79 if run == 'plus_hinf' else 80
            wanted.update({'upset_m1': walls[k - 1], 'upset': walls[k], 'upset_p1': walls[k + 1]})
        if run == 'rect_hinf':
            wanted['mid'] = walls[150]
        for tag, wall in wanted.items():
            dest = OUT / f'{run}_{tag}.png'
            ok, pos = grab(video, (wall - t0) * 1000.0, dest)
            index[f'{run}_{tag}'] = {'video': vname, 'video_s': pos, 'log_wall_epoch': wall, 'ok': ok}
            print(f'{run}_{tag}: {"ok" if ok else "GAGAL"} @ {pos}')
    (OUT / 'index.json').write_text(json.dumps(index, indent=2))


# ─────────────────────────────────────────────────────────────────────────
# Montase figure lingkungan: (a) Gazebo dari screenshot pengguna, (b-e) peta
# coverage RViz di akhir misi PID-H∞ untuk 4 region (frame video di atas).
# ─────────────────────────────────────────────────────────────────────────
def _green_bbox(img, pad=18):
    import numpy as np
    b, g, r = img[..., 0].astype(int), img[..., 1].astype(int), img[..., 2].astype(int)
    m = (g > 100) & (g > r + 40) & (g > b + 40)
    m[:, :990] = False                      # hanya panel RViz (kanan)
    ys, xs = np.where(m)
    y0, y1 = np.percentile(ys, [0.2, 99.8]).astype(int)
    x0, x1 = np.percentile(xs, [0.2, 99.8]).astype(int)
    h = max(y1 - y0, x1 - x0) + 2 * pad
    cy, cx = (y0 + y1) // 2, (x0 + x1) // 2
    y0, x0 = max(0, cy - h // 2), max(0, cx - h // 2)
    return img[y0:y0 + h, x0:x0 + h]


def montage():
    import numpy as np
    shot = cv2.imread(str(OUT.parent / 'Screenshot From 2026-09-14 11-19-34.png'))
    if shot is None:
        print('[skip] screenshot tidak ada', file=sys.stderr)
        return
    side = 640
    gz = cv2.resize(shot[140:915, 60:860], (side, side), interpolation=cv2.INTER_AREA)
    panels = [gz]
    for run in ('rect_hinf', 'l_shape_hinf', 'u_shape_hinf', 'plus_hinf'):
        img = cv2.imread(str(OUT / f'{run}_end.png'))
        panels.append(cv2.resize(_green_bbox(img), (side, side), interpolation=cv2.INTER_AREA))
    # label panel (a)-(e) di pojok kiri atas, sesuai rujukan di caption paper
    for k, p in enumerate(panels):
        lbl = f'({"abcde"[k]})'
        (tw, th), base = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 1.6, 3)
        cv2.rectangle(p, (8, 8), (8 + tw + 16, 8 + th + base + 14), (255, 255, 255), -1)
        cv2.putText(p, lbl, (16, 8 + th + 7), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 0), 3, cv2.LINE_AA)
    gap = np.full((side, 12, 3), 255, np.uint8)
    row = panels[0]
    for p in panels[1:]:
        row = np.hstack([row, gap, p])
    cv2.imwrite(str(OUT.parent / 'fig_env.png'), row)
    print('fig_env.png ok')


if __name__ == '__main__':
    if '--montage-only' not in sys.argv:
        main()
    montage()
