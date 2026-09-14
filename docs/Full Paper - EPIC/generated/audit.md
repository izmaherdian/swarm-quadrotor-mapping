# Audit data Skema 5 (dihasilkan otomatis)

## rect / hinf
- baris STATUS = 259; Total Waktu Misi log = 260.0; akhir CSV = 261.00 s
- kill iris_4: log t≈35 s (cov 18.8%), CSV onset 36.884
- kill iris_7: log t≈56 s (cov 32.0%), CSV onset 57.888
- offset CSV−log (median kill) = 1.8859999999999992
- coverage final log = 98.2 | STATUS terakhir = 98.2 | offline batas atas CSV = 98.9
- lateral: mean 1.6 cm, std 7.0 cm, jendela integrator LQR 87%
- d_min V2V: log = 1.71 | CSV = 1.71 m (pasangan [2, 3], t=8.977999999999975)
- clearance permukaan statis min = 0.492 m (iris_1, obs 105, t=92.2) → kontak: none
- PETA: 10 track terkonfirmasi-diam, id truth tercocok [101, 102, 103, 104, 105, 106, 107, 108, 109]; err posisi maks (cocok) 0.0 m, |err r| maks 0.010000000000000009, track spurious 1
- Tier-2 per pembatas: {'v2v_soft': 17, 'dynamic': 17, 'static': 6}; h_min log (semua baris) saat baris bergerak mengikat = -0.43

## rect / lqr
- baris STATUS = 77; Total Waktu Misi log = None; akhir CSV = 78.80 s
- kill iris_7: log t≈60 s (cov 19.2%), CSV onset 61.52
- offset CSV−log (median kill) = 1.5200000000000031
- coverage final log = None | STATUS terakhir = 29.0 | offline batas atas CSV = 42.6
- lateral: mean -7.0 cm, std 9.4 cm, jendela integrator LQR 64%
- d_min V2V: log = 1.52 | CSV = 1.52 m (pasangan [4, 7], t=4.787999999999991)
- clearance permukaan statis min = 0.055 m (iris_2, obs 108, t=77.3) → kontak: certain
- PETA: 9 track terkonfirmasi-diam, id truth tercocok [101, 102, 103, 104, 105, 106, 107, 108, 109]; err posisi maks (cocok) 0.0 m, |err r| maks 0.010000000000000009, track spurious 0
- Tier-2 per pembatas: {'v2v_soft': 46, 'dynamic': 15, 'static': 30}; h_min log (semua baris) saat baris bergerak mengikat = -0.57
- upset iris_2 t=77.1 tilt=155.9° pos=(0.07,1.95) zmin=0.11 surf_min=0.06 recovered=False
- upset iris_6 t=24.4 tilt=50.8° pos=(3.90,-3.38) zmin=1.66 surf_min=0.55 recovered=True
- ABORT: iris_2 Z=0.11 pada log t≈77 s

## l_shape / hinf
- baris STATUS = 203; Total Waktu Misi log = 204.0; akhir CSV = 204.61 s
- kill iris_4: log t≈35 s (cov 18.8%), CSV onset 36.44
- kill iris_7: log t≈51 s (cov 32.5%), CSV onset 52.404
- offset CSV−log (median kill) = 1.4220000000000006
- coverage final log = 98.4 | STATUS terakhir = 98.4 | offline batas atas CSV = 98.6
- lateral: mean 1.0 cm, std 8.4 cm, jendela integrator LQR 87%
- d_min V2V: log = 1.32 | CSV = 1.32 m (pasangan [1, 3], t=129.32799999999955)
- clearance permukaan statis min = 0.566 m (iris_3, obs 304, t=175.9) → kontak: none
- PETA: 9 track terkonfirmasi-diam, id truth tercocok [301, 302, 303, 304, 305, 306, 307, 308, 309]; err posisi maks (cocok) 0.0 m, |err r| maks 0.009999999999999953, track spurious 0
- Tier-2 per pembatas: {'v2v_soft': 17, 'dynamic': 38, 'static': 4}; h_min log (semua baris) saat baris bergerak mengikat = -0.68

## l_shape / lqr
- baris STATUS = 235; Total Waktu Misi log = 236.0; akhir CSV = 236.76 s
- kill iris_4: log t≈39 s (cov 18.3%), CSV onset 40.628
- kill iris_7: log t≈55 s (cov 32.7%), CSV onset 56.628
- offset CSV−log (median kill) = 1.6280000000000001
- coverage final log = 98.5 | STATUS terakhir = 98.5 | offline batas atas CSV = 98.7
- lateral: mean -5.7 cm, std 10.8 cm, jendela integrator LQR 56%
- d_min V2V: log = 0.79 | CSV = 0.79 m (pasangan [5, 6], t=45.029999999999845)
- clearance permukaan statis min = 0.287 m (iris_3, obs 306, t=94.9) → kontak: possible
- PETA: 11 track terkonfirmasi-diam, id truth tercocok [301, 302, 303, 304, 305, 306, 307, 308, 309]; err posisi maks (cocok) 0.0 m, |err r| maks 0.010000000000000009, track spurious 2
- Tier-2 per pembatas: {'v2v_soft': 25, 'dynamic': 12, 'static': 14}; h_min log (semua baris) saat baris bergerak mengikat = -0.58

## u_shape / hinf
- baris STATUS = 204; Total Waktu Misi log = 205.0; akhir CSV = 205.66 s
- kill iris_4: log t≈39 s (cov 18.5%), CSV onset 40.544
- kill iris_5: log t≈56 s (cov 33.0%), CSV onset 57.544
- offset CSV−log (median kill) = 1.543999999999997
- coverage final log = 98.5 | STATUS terakhir = 98.5 | offline batas atas CSV = 99.0
- lateral: mean 0.7 cm, std 8.3 cm, jendela integrator LQR 86%
- d_min V2V: log = 1.77 | CSV = 1.77 m (pasangan [3, 7], t=3.3779999999999957)
- clearance permukaan statis min = 0.633 m (iris_6, obs 302, t=8.2) → kontak: none
- PETA: 10 track terkonfirmasi-diam, id truth tercocok [301, 302, 303, 304, 305, 306, 307, 308, 309]; err posisi maks (cocok) 0.0 m, |err r| maks 0.010000000000000009, track spurious 1
- Tier-2 per pembatas: {'v2v_soft': 12, 'dynamic': 20, 'static': 3}; h_min log (semua baris) saat baris bergerak mengikat = -0.63

## u_shape / lqr
- baris STATUS = 239; Total Waktu Misi log = 240.0; akhir CSV = 240.48 s
- kill iris_4: log t≈51 s (cov 18.4%), CSV onset 52.304
- kill iris_5: log t≈69 s (cov 32.9%), CSV onset 70.336
- offset CSV−log (median kill) = 1.3200000000000003
- coverage final log = 96.9 | STATUS terakhir = 96.9 | offline batas atas CSV = 97.8
- lateral: mean -5.8 cm, std 9.8 cm, jendela integrator LQR 56%
- d_min V2V: log = 1.73 | CSV = 1.73 m (pasangan [5, 6], t=11.379999999999967)
- clearance permukaan statis min = 0.423 m (iris_7, obs 303, t=14.3) → kontak: none
- PETA: 10 track terkonfirmasi-diam, id truth tercocok [301, 302, 303, 304, 305, 306, 307, 308, 309]; err posisi maks (cocok) 0.0 m, |err r| maks 0.010000000000000009, track spurious 1
- Tier-2 per pembatas: {'static': 7, 'v2v_soft': 19, 'dynamic': 13}; h_min log (semua baris) saat baris bergerak mengikat = -0.29

## plus / hinf
- baris STATUS = 197; Total Waktu Misi log = 198.0; akhir CSV = 198.68 s
- kill iris_1: log t≈44 s (cov 18.5%), CSV onset 45.54
- kill iris_7: log t≈59 s (cov 33.0%), CSV onset 60.528
- offset CSV−log (median kill) = 1.533999999999999
- coverage final log = 98.4 | STATUS terakhir = 98.4 | offline batas atas CSV = 99.6
- lateral: mean 1.0 cm, std 9.6 cm, jendela integrator LQR 84%
- d_min V2V: log = 1.24 | CSV = 1.24 m (pasangan [4, 7], t=27.989999999999906)
- clearance permukaan statis min = 0.242 m (iris_3, obs 305, t=33.1) → kontak: possible
- PETA: 9 track terkonfirmasi-diam, id truth tercocok [301, 302, 303, 304, 305, 306, 307, 308, 309]; err posisi maks (cocok) 0.0 m, |err r| maks 0.0, track spurious 0
- Tier-2 per pembatas: {'v2v_soft': 29, 'dynamic': 13, 'static': 12}; h_min log (semua baris) saat baris bergerak mengikat = -0.55
- upset iris_2 t=80.4 tilt=82.5° pos=(0.33,0.64) zmin=0.64 surf_min=0.28 recovered=True

## plus / lqr
- baris STATUS = 221; Total Waktu Misi log = 222.0; akhir CSV = 222.62 s
- kill iris_1: log t≈46 s (cov 18.5%), CSV onset 47.236
- kill iris_7: log t≈62 s (cov 33.1%), CSV onset 63.5
- offset CSV−log (median kill) = 1.3679999999999986
- coverage final log = 99.0 | STATUS terakhir = 99.0 | offline batas atas CSV = 99.5
- lateral: mean -6.5 cm, std 9.6 cm, jendela integrator LQR 55%
- d_min V2V: log = 1.12 | CSV = 1.12 m (pasangan [4, 7], t=29.879999999999903)
- clearance permukaan statis min = 0.241 m (iris_2, obs 305, t=82.0) → kontak: possible
- PETA: 10 track terkonfirmasi-diam, id truth tercocok [301, 302, 303, 304, 305, 306, 307, 308, 309]; err posisi maks (cocok) 0.0 m, |err r| maks 0.0, track spurious 1
- Tier-2 per pembatas: {'dynamic': 20, 'v2v_soft': 42, 'static': 13}; h_min log (semua baris) saat baris bergerak mengikat = -0.64
- upset iris_2 t=82.1 tilt=40.1° pos=(0.20,0.63) zmin=1.87 surf_min=0.24 recovered=True

## Analisis desain linear (sumbu x)
- lqr: stabil=True ‖G‖∞=0.1583 @ 1.45 rad/s, PM=37.37424040588982, GM=11.49100971523168 dB, wc=4.093568075094219, σx/σa=0.1145, gains out={'Kp': 0.756947541151533, 'Ki': 0.2828427124746191, 'Kd': 0.3941582551386396} in={'Kp': 4.86787817830416, 'Kd': 0.5338577806656953}
- hinf: stabil=True ‖G‖∞=0.0395 @ 1.17 rad/s, PM=25.209744725201602, GM=9.017350793393346 dB, wc=9.078058955188782, σx/σa=0.0306, gains out={'Kp': 2.705236268535122, 'Ki': 1.0918833065479654, 'Kd': 1.1066100784713377} in={'Kp': 11.603859227342754, 'Kd': 1.1891807762349929}

**STATUS AUDIT: LOLOS**
