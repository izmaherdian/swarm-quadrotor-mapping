#!/usr/bin/env python3
"""
Klip video Skema 5 untuk README GitHub: crop (buang dock, top bar, judul jendela),
potong ke rentang log coordinator, percepat 8x, dan beri overlay teks yang
SELURUHNYA berasal dari log coordinator (waktu sim, coverage, kill, akhir misi).

Pemetaan waktu sama dengan skema5_video_frames.py:
    posisi_video_s = epoch_baris_log (WALL) - epoch_mulai_rekaman (nama file, WIB)
dan baris [STATUS] ke-i (dari 0) dicetak pada detik misi ke-(i+1): pesan SWARM SUCCESS
"Durasi Misi: 187.0s" muncul setelah tepat 186 baris STATUS (konsisten di 7 log).

Keluaran: media/scheme5/<run>.mp4, media/scheme5/thumbs/<run>.jpg, media/scheme5/clips.json

Pakai (butuh ffmpeg dengan libx264 + libass):
    cd swarm_ws && FFMPEG=/path/ffmpeg python3 analysis/skema5_video_clips.py [run ... | --thumbs-only]
"""
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from skema5_video_frames import DATA, REPO, RUNS, VIDEO_DIR, video_start_epoch  # noqa: E402

OUT = REPO / 'media' / 'scheme5'
SPEED = 8
CROP = '1870:990:50:60'          # w:h:x:y pada frame 1920x1080
WIDTH = 1280
PRE_S, POST_S = 2.0, 4.0         # margin sebelum baris log pertama / sesudah terakhir
EVENT_HOLD_S = 30.0              # lama teks kejadian di video ASLI (= 3.75 s setelah 8x)

REGION = {'rect': 'Rectangle', 'l_shape': 'L-shape', 'u_shape': 'U-shape', 'plus': 'Plus'}
CTRL = {'hinf': 'PID-H∞', 'lqr': 'PID-LQR'}
RE_LINE = re.compile(r'^\[\w+\] \[(\d+\.\d+)\] \[[^\]]+\]: (.*)$')
RE_COV = re.compile(r'Cov:\s*([\d.]+)%')
RE_KILL = re.compile(r'EMERGENCY KILL\] (iris_\d+)')
RE_Z = re.compile(r'(iris_\d+) jatuh tak terkendali ke Z=([\d.]+)m')


def parse_log(path):
    first = last = None
    status, events = [], []
    for line in path.read_text(errors='replace').splitlines():
        m = RE_LINE.match(line)
        if not m:
            continue
        wall, msg = float(m.group(1)), m.group(2)
        first = wall if first is None else first
        last = wall
        if '[STATUS]' in msg:
            c = RE_COV.search(msg)
            status.append((wall, float(c.group(1)) if c else None))
        elif 'EMERGENCY KILL' in msg:
            events.append((wall, 'kill', RE_KILL.search(msg).group(1)))
        elif 'SWARM SUCCESS' in msg:
            events.append((wall, 'target', None))
        elif 'AUTO-EXIT: SUCCESS' in msg and 'centroid' in msg:
            c = RE_COV.search(msg)
            events.append((wall, 'done', c.group(1) if c else None))
        elif 'AUTO-EXIT: FAILURE' in msg:
            z = RE_Z.search(msg)
            events.append((wall, 'abort', z.groups() if z else None))
    return first, last, status, events


def ass_time(s):
    s = max(0.0, s)
    h, rem = divmod(s, 3600)
    m, sec = divmod(rem, 60)
    return f'{int(h)}:{int(m):02d}:{sec:05.2f}'


def ass_file(run, t_start, t0, status, events, dest):
    region, ctrl = run.rsplit('_', 1)
    head = (
        '[Script Info]\nScriptType: v4.00+\nPlayResX: 1280\nPlayResY: 678\n\n'
        '[V4+ Styles]\nFormat: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, '
        'OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, '
        'Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding\n'
        'Style: Hud,DejaVu Sans,15,&H00FFFFFF,&H00FFFFFF,&H8C000000,&H8C000000,0,0,0,0,100,100,0,0,'
        '3,4,0,9,10,645,4,1\n'
        'Style: Event,DejaVu Sans,16,&H0000D7FF,&H00FFFFFF,&H8C000000,&H8C000000,1,0,0,0,100,100,0,0,'
        '3,5,0,9,10,645,52,1\n\n'
        '[Events]\nFormat: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n')
    rows = []
    title = f'{REGION[region]} · {CTRL[ctrl]} · Scheme 5 · video {SPEED}× faster'
    alive = 7
    kills = {w: who for w, kind, who in events if kind == 'kill'}
    walls = [w for w, _ in status]
    for i, (w, cov) in enumerate(status):
        alive -= sum(1 for kw in kills if (walls[i - 1] if i else 0) < kw <= w)
        end = walls[i + 1] if i + 1 < len(walls) else w + 1.0
        cov_txt = f'{cov:.1f}%' if cov is not None else 'n/a'
        txt = f'{title}\\Nsim t = {i + 1:3d} s  ·  coverage {cov_txt}  ·  alive {alive}/7'
        rows.append((w, end, 'Hud', txt))
    if walls:
        rows.insert(0, (t0 + t_start, walls[0], 'Hud', f'{title}\\Nstarting up'))
    for w, kind, info in events:
        k = sum(1 for x in walls if x <= w) + 1
        if kind == 'kill':
            txt = f'{info} killed at sim t = {k} s\\N→ cell merged, lanes reallocated'
        elif kind == 'target':
            txt = f'97% coverage reached at sim t = {k} s'
        elif kind == 'done':
            txt = f'mission complete at sim t = {k} s\\Nsurvivors back at centroids, coverage {info}%'
        else:
            who, z = info if info else ('agent', '?')
            txt = f'mission ABORTED at sim t = {k} s\\N{who} lost control and fell (Z = {z} m)'
        rows.append((w, w + EVENT_HOLD_S, 'Event', txt))
    lines = [f'Dialogue: 0,{ass_time(a - t0 - t_start)},{ass_time(b - t0 - t_start)},{st},,0,0,0,,{tx}'
             for a, b, st, tx in rows]
    dest.write_text(head + '\n'.join(lines) + '\n', encoding='utf-8')


def thumbnail(ffmpeg, clip):
    """Frame 0.5 s sebelum akhir klip (peta coverage final + teks akhir misi), lebar 640 px."""
    dest = OUT / 'thumbs' / f'{clip.stem}.jpg'
    dest.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-sseof', '-0.5', '-i', str(clip),
                    '-frames:v', '1', '-vf', 'scale=640:-2', '-q:v', '4', str(dest)], check=True)


def main():
    ffmpeg = os.environ.get('FFMPEG', 'ffmpeg')
    OUT.mkdir(parents=True, exist_ok=True)
    args = sys.argv[1:]
    if '--thumbs-only' in args:
        for clip in sorted(OUT.glob('*.mp4')):
            thumbnail(ffmpeg, clip)
        return
    wanted = args or list(RUNS)
    index_path = OUT / 'clips.json'
    index = json.loads(index_path.read_text()) if index_path.is_file() else {}
    for run in wanted:
        folder, vname = RUNS[run]
        video = VIDEO_DIR / vname
        if not video.is_file():
            print(f'[skip] video tidak ada: {video}', file=sys.stderr)
            continue
        cap = cv2.VideoCapture(str(video))
        dur = cap.get(cv2.CAP_PROP_FRAME_COUNT) / cap.get(cv2.CAP_PROP_FPS)
        cap.release()
        first, last, status, events = parse_log(DATA / folder / 'coordinator.log')
        t0 = video_start_epoch(vname)
        t_start = max(0.0, first - t0 - PRE_S)
        t_end = min(dur, last - t0 + POST_S)
        dest = OUT / f'{run}.mp4'
        with tempfile.TemporaryDirectory() as td:
            ass = Path(td) / 'hud.ass'
            ass_file(run, t_start, t0, status, events, ass)
            vf = (f'crop={CROP},scale={WIDTH}:-2,subtitles=filename={ass}:fontsdir=/usr/share/fonts,'
                  f'setpts=PTS/{SPEED},fps=30')
            cmd = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-ss', f'{t_start:.2f}',
                   '-t', f'{t_end - t_start:.2f}', '-i', str(video), '-an', '-vf', vf,
                   '-c:v', 'libx264', '-preset', 'slow', '-crf', '30', '-pix_fmt', 'yuv420p',
                   '-movflags', '+faststart', str(dest)]
            subprocess.run(cmd, check=True)
        thumbnail(ffmpeg, dest)
        sim_s = len(status)
        wall_s = (status[-1][0] - status[0][0]) if len(status) > 1 else float('nan')
        index[run] = {
            'source_video': vname, 'coordinator_log': f'{folder}/coordinator.log',
            'trim_s': [round(t_start, 1), round(t_end, 1)], 'speedup': SPEED,
            'clip_s': round((t_end - t_start) / SPEED, 1),
            'recording_rtf': round(sim_s / wall_s, 3) if wall_s == wall_s else None,
            'size_mb': round(dest.stat().st_size / 1e6, 2),
        }
        print(run, index[run], flush=True)
        index_path.write_text(json.dumps(index, indent=2, ensure_ascii=False) + '\n')


if __name__ == '__main__':
    main()
