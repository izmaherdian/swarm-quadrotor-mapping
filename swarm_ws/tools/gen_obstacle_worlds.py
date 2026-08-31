#!/usr/bin/env python3
"""Bangkitkan berkas world Skema 3 (rintangan STATIS saja) per wilayah.

Kenapa dibangkitkan, bukan ditulis tangan
-----------------------------------------
Koordinat rintangan hidup di dua tempat: tabel Python yang dipakai perencana
jalur, dan SDF yang dipakai Gazebo. Selama ini keduanya hanya dijaga oleh
sebuah komentar "harus tetap cocok" — dan tidak ada yang memeriksanya. Skrip
ini menjadikan `swarm_high_level.world.obstacles` sumber tunggal dan
menurunkan SDF darinya; `test_obstacle_paths.py` mem-parsing hasilnya dan
membandingkannya kembali ke tabel.

Beda dengan `obstacles.world`
-----------------------------
Berkas hasil TIDAK memuat kedua silinder dinamis. `obstacles.world` dibiarkan
apa adanya untuk Skema 4. Ini bukan sekadar kerapian: kedua silinder itu
membentang z 0.25-3.85 m sementara drone menjelajah di 2.0 m, jadi
membiarkannya ter-spawn tapi tidak digerakkan akan meninggalkan dua rintangan
diam di dalam arena yang tidak dilihat perencana maupun QP.

Pemakaian
---------
    python3 tools/gen_obstacle_worlds.py [--check]

``--check`` tidak menulis apa pun; keluar dengan kode 1 bila ada berkas yang
tidak sesuai dengan tabel (dipakai CI / tes).
"""
import argparse
import pathlib
import sys

WS = pathlib.Path(__file__).resolve().parent.parent
SRC = WS / 'src' / 'swarm_high_level'
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from swarm_high_level.world.obstacles import (          # noqa: E402
    OBSTACLES_BY_REGION, OBSTACLE_COLORS, OBSTACLE_HEIGHT, OBSTACLE_RADIUS)

WORLDS = WS / 'src' / 'swarm_sim' / 'worlds'
BASE_WORLD = WORLDS / 'obstacles.world'

# Penanda potong pada berkas basis: semua SEBELUM baris ini dipakai apa adanya
# (fisika, plugin, matahari, lantai, grid arena 30x30, border, staging pad).
CUT_MARKER = '<!-- 9 RINTANGAN STATIS'
FOOTER = '\n  </world>\n</sdf>\n'

MODEL_TEMPLATE = """\
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>
          <material>
            <ambient>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</ambient>
            <diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1.0</diffuse>
            <specular>0.5 0.5 0.5 1.0</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def _preamble():
    """Bagian atas berkas basis, sampai tepat sebelum blok rintangan."""
    text = BASE_WORLD.read_text()
    idx = text.find(CUT_MARKER)
    if idx < 0:
        raise SystemExit(f'Penanda {CUT_MARKER!r} tidak ditemukan di {BASE_WORLD}')
    head = text[:idx]
    return head[:head.rfind('<!-- ====')] if '<!-- ====' in head else head


def world_text(region):
    """Isi lengkap berkas world untuk sebuah wilayah."""
    table = OBSTACLES_BY_REGION[region]
    body = [
        '    <!-- ===================================================================== -->',
        f'    <!-- {len(table)} RINTANGAN STATIS — wilayah {region!r}'.ljust(74) + '-->',
        '    <!-- DIBANGKITKAN oleh tools/gen_obstacle_worlds.py — JANGAN diedit tangan. -->',
        '    <!-- Sumber: swarm_high_level/world/obstacles.py :: OBSTACLES_BY_REGION     -->',
        '    <!-- ===================================================================== -->',
        '',
    ]
    for i, (oid, ox, oy) in enumerate(table):
        cr, cg, cb = OBSTACLE_COLORS[i % len(OBSTACLE_COLORS)]
        body.append(MODEL_TEMPLATE.format(
            name=f'static_obs_{oid}', x=ox, y=oy, z=OBSTACLE_HEIGHT / 2.0,
            r=OBSTACLE_RADIUS, h=OBSTACLE_HEIGHT, cr=cr, cg=cg, cb=cb))
    return _preamble() + '\n'.join(body) + FOOTER


def path_for(region):
    return WORLDS / f'obstacles_{region}.world'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--check', action='store_true',
                    help='hanya periksa, jangan tulis; exit 1 bila melenceng')
    args = ap.parse_args()

    stale = []
    for region in OBSTACLES_BY_REGION:
        dest = path_for(region)
        text = world_text(region)
        if args.check:
            if not dest.exists() or dest.read_text() != text:
                stale.append(dest.name)
            continue
        dest.write_text(text)
        print(f'  ditulis {dest.relative_to(WS)} '
              f'({len(OBSTACLES_BY_REGION[region])} rintangan statis, tanpa dinamis)')

    if args.check:
        if stale:
            print('MELENCENG dari obstacles.py: ' + ', '.join(stale))
            print('Jalankan: python3 tools/gen_obstacle_worlds.py')
            return 1
        print('Seluruh berkas world sesuai dengan obstacles.py')
    return 0


if __name__ == '__main__':
    sys.exit(main())
