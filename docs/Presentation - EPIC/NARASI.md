# Narasi presentasi EPIC 2026

Dihasilkan `tools/export_narasi.py` dari `\narasi{}` di `main.tex` — **jangan diedit di sini**, edit `main.tex` lalu `make narasi`.
Angka diambil dari `numbers.tex`, file yang sama dengan paper.

Slide 1–52 · 44 slide bicara · 10.0 menit · 985 kata Inggris (≈ 98 kata/menit).

Pembatas seksi tidak punya narasi: tahan sebentar, lalu klik lanjut.

## Slide 1 — Title

*0.2 menit · 19 kata*

**EN** Hello everyone. I am Izma Alhazmi Herdian from Institut Teknologi Bandung, presenting fault-tolerant swarm quadrotor control for non-convex mapping.

**ID** Halo semuanya. Saya Izma Alhazmi Herdian dari Institut Teknologi Bandung, mempresentasikan kendali swarm quadrotor yang toleran kegagalan untuk pemetaan area non-konveks.

> Arahan: 10 min video. Open by naming the result out loud, then move on. Everything after this explains how — and what failed.

## Slide 2 — Seven quadrotors, four regions, every stressor at once

*0.3 menit · 38 kata*

**EN** Here is the system at work. Seven quadrotors split a region, sweep it lane by lane, avoid obstacles found with LiDAR, and survive losing two drones mid-flight. In every mission, the survivors take over the lost drones' work.

**ID** Inilah sistemnya bekerja. Tujuh quadrotor membagi area, menyapunya lajur demi lajur, menghindari rintangan yang ditemukan LiDAR, dan bertahan meski kehilangan dua drone saat terbang. Di setiap misi, drone yang tersisa mengambil alih pekerjaan drone yang hilang.

> Arahan: Let the clip run and say only what they are looking at: seven drones split the region, sweep lanes, avoid the cylinders, and two of them are killed mid-mission. Everything after this explains how.

## Slide 3 — Seven of eight missions finished after losing 28.6% of the fleet

*0.2 menit · 29 kata*

**EN** The answer first: seven of eight missions finished with 96.9 to 99.0 percent coverage. One crashed, and I will show it. H-infinity was faster, at a higher torque cost.

**ID** Jawabannya dulu: tujuh dari delapan misi selesai dengan cakupan 96.9 sampai 99.0 persen. Satu menabrak, dan akan saya tunjukkan. H-infinity lebih cepat, dengan biaya torsi lebih besar.

> Arahan: Tell them the answer first, then spend the talk earning it. Signalling the crash this early buys credibility for every number after.

## Slide 4 — pembatas 1 · The problem

## Slide 5 — The question is not whether a swarm is faster, but whether it still finishes

*0.3 menit · 40 kata*

**EN** Mapping means every point must pass under a sensor. We combine four difficulties at once: a non-convex region, strong wind, unseen obstacles, and two drones killed mid-mission. So the question is not only speed, but whether the swarm still finishes.

**ID** Pemetaan berarti setiap titik harus terlewati sensor. Kami menggabungkan empat kesulitan sekaligus: area non-konveks, angin kencang, rintangan yang belum dikenal, dan dua drone dimatikan di tengah misi. Jadi pertanyaannya bukan hanya kecepatan, tetapi apakah swarm tetap menyelesaikan misinya.

> Arahan: Survey mapping is a coverage problem — every point must pass under a sensor. The equation is a budget, not a result; say so. Name the scenario “Scheme 5” once and move on.

## Slide 6 — Four mechanisms, and a campaign honest enough to include a crash

*0.2 menit · 18 kata*

**EN** We contribute boundary-following lanes, lane-level reallocation, a physically derived safety filter, and a fair LQR versus H-infinity comparison.

**ID** Kontribusi kami: lajur yang mengikuti batas sel, realokasi tingkat lajur, filter keselamatan yang diturunkan secara fisik, dan perbandingan adil LQR dengan H-infinity.

> Arahan: The slide reviewers remember. One breath per bullet. Do not expand on any of them here — each has its own slide later.

## Slide 7 — pembatas 2 · The system as blocks

## Slide 8 — Three nested loops, each one handing a set-point to the next

*0.3 menit · 45 kata*

**EN** There are three layers. The high level plans lanes, the mid level at 20 hertz makes velocities safe, and the low level at 250 hertz drives the rotors. Each layer hands a set-point to the next, and we will walk it from the bottom up.

**ID** Ada tiga lapis. Lapis atas merencanakan lajur, lapis tengah pada 20 hertz membuat kecepatan aman, dan lapis bawah pada 250 hertz menggerakkan rotor. Tiap lapis menyerahkan set-point ke lapis berikutnya, dan kita telusuri dari bawah ke atas.

> Arahan: Read the three columns aloud left to right, then say the sentence that sets up the rest of the talk: we are going to walk this diagram BACKWARDS, because every limit upstairs is derived from what the rotors can actually do.

## Slide 9 — We derive the stack backwards: what the rotors allow decides what the planner may ask

*0.2 menit · 18 kata*

**EN** Every limit at the top is derived from what the rotors can do. Nothing is tuned by hand.

**ID** Setiap batas di lapis atas diturunkan dari kemampuan rotor. Tidak ada yang disetel tangan.

> Arahan: This slide is the promise of the talk. Say it plainly: no magic constants; every number upstairs has a derivation. Also mention what is NOT in the loop – ground-truth odometry, no PX4, no estimator; it is on the Limitations slide and in Q2.

## Slide 10 — pembatas 3 · Low level

## Slide 11 — Low level: a position in metres goes in, four rotor speeds come out

*0.1 menit · 11 kata*

**EN** The low level turns a position set-point into four rotor speeds.

**ID** Lapis bawah mengubah set-point posisi menjadi empat kecepatan rotor.

> Arahan: Read the three boxes left to right, then trace the chain. The two feedback paths are the point: the inner loop sees attitude, the outer loop sees position, and they run at the same 250 Hz.

## Slide 12 — Step 1 — the only horizontal force a quadrotor has is tilted thrust

*0.3 menit · 28 kata*

**EN** Step one: the only sideways force is tilted thrust, so acceleration depends only on tilt. Capping tilt at 15 degrees caps acceleration. That number drives everything that follows.

**ID** Langkah satu: satu-satunya gaya menyamping adalah thrust yang dimiringkan, jadi percepatan hanya bergantung pada kemiringan. Membatasi kemiringan 15 derajat berarti membatasi percepatan. Angka itu mendasari semua yang berikutnya.

> Arahan: The algebra is two lines; do not labour it. The point is that one design choice – 15 degrees – becomes a hard acceleration ceiling.

## Slide 13 — 2.631 m/s²

*0.1 menit · 7 kata*

**EN** That is 2.631 metres per second squared.

**ID** Yaitu 2.631 meter per detik kuadrat.

> Arahan: Say the number, pause, move on. It is the hinge of the whole talk.

## Slide 14 — Step 2 — twelve coupled states become four simple chains

*0.2 menit · 13 kata*

**EN** Step two: around hover, the twelve coupled states split into four simple chains.

**ID** Langkah dua: di sekitar kondisi melayang, dua belas state yang terkopel terpisah menjadi empat rantai sederhana.

> Arahan: For a non-control audience this is the bridge: do not let them wonder why a quadrotor became a 2x2 matrix. Say the word 'decoupled' once and point at the table.

## Slide 15 — Step 3 — augment with the input, and the integral term falls out

*0.2 menit · 26 kata*

**EN** Step three: augmenting each chain with its input makes the integral term fall out of the design, rather than being tuned. Both controllers share this structure.

**ID** Langkah tiga: mengaugmentasi tiap rantai dengan masukannya membuat suku integral muncul dari desain, bukan dari tuning. Kedua pengendali berbagi struktur ini.

> Arahan: The densest slide of the talk. Q for the outer loops is diag(0.08, 0.35) with R = 1; for the inner loops diag(6, 0.6) with R = 0.05. Gamma is found by bisection. Do not read those out.

## Slide 16 — Step 4 — same augmented plant, two Riccati equations

*0.3 menit · 19 kata*

**EN** Step four: LQR is optimal for the model, H-infinity against the worst disturbance. Same plant, same solver, different objective.

**ID** Langkah empat: LQR optimal untuk modelnya, H-infinity terhadap gangguan terburuk. Plant sama, solver sama, tujuan berbeda.

> Arahan: Do not derive either equation. The message is that the two designs pose the SAME augmented plant and differ only in which Riccati equation they solve – and in Q, R, gamma.

## Slide 17 — Step 5 — project the state feedback onto a PID, and that is what flies

*0.2 menit · 17 kata*

**EN** Step five: both become an ordinary PID. Note the LQR integrator is gated, and H-infinity's is not.

**ID** Langkah lima: keduanya menjadi PID biasa. Perhatikan integrator LQR diberi gerbang, sedangkan H-infinity tidak.

> Arahan: The callout is the single most important line of the low level: it is the actual control law, and it names the asymmetry that explains the headline result.

## Slide 18 — Step 6 — the outer PID plus a lead reference collapse into a velocity plant

*0.3 menit · 27 kata*

**EN** Step six: the coordinator sends a position that leads the drone, so the commanded speed is exactly the speed held. The closed loop becomes one first-order equation.

**ID** Langkah enam: koordinator mengirim posisi yang mendahului drone, sehingga kecepatan perintah tepat sama dengan kecepatan yang dipertahankan. Loop tertutupnya menjadi satu persamaan orde satu.

> Arahan: The longest derivation of the talk. Walk the three lines. The payoff is on the next slide: four numbers that everything else is built on.

## Slide 19 — Step 7 — four numbers, and they come from the gains, not from tuning

*0.2 menit · 23 kata*

**EN** Step seven: the whole low level reduces to four numbers, all taken from the gains. Change a gain, and all four move together.

**ID** Langkah tujuh: seluruh lapis bawah tereduksi menjadi empat angka, semuanya diambil dari gain. Ubah satu gain, maka keempatnya ikut berubah.

> Arahan: These four numbers are the interface between the low level and everything above. Table 1 of the paper has the full gain set; do not put it on screen unless asked.

## Slide 20 — What Q, R and γ actually bought, before any flight

*0.2 menit · 14 kata*

**EN** On paper, H-infinity rejects disturbances 4.0 times better. The flight data will qualify that.

**ID** Di atas kertas, H-infinity menolak gangguan 4.0 kali lebih baik. Data terbang akan memberi catatan untuk itu.

> Arahan: Set up the honest caveat early: the model predicts broadband attenuation, but the flight data will show the gain is mostly a rejected mean offset. Full gain table is in backup.

## Slide 21 — pembatas 4 · Mid level

## Slide 22 — Mid level: a wish in metres per second goes in, a safe one comes out

*0.2 menit · 14 kata*

**EN** The mid level adjusts the planner's velocity as little as needed to stay safe.

**ID** Lapis tengah mengubah kecepatan dari perencana seperlunya saja agar tetap aman.

> Arahan: The key sentence: the state machine proposes, the QP disposes, and the only thing traded away is how closely we honour the wish.

## Slide 23 — Step 8 — integrate the plant and you get an exact stopping distance

*0.2 menit · 11 kata*

**EN** Step eight: integrating the velocity model gives the exact stopping distance.

**ID** Langkah delapan: mengintegrasikan model kecepatan memberi jarak berhenti yang eksak.

> Arahan: Emphasise that nothing has been approximated yet beyond the small angle step.

## Slide 24 — Step 9 — add dead time, and safety becomes one inequality

*0.2 menit · 13 kata*

**EN** Step nine: add the 0.4 second reaction delay, and safety becomes one inequality.

**ID** Langkah sembilan: tambahkan jeda reaksi 0,4 detik, dan keselamatan menjadi satu pertidaksamaan.

> Arahan: The budget a is not quite a max: a = max(0.15 a max, mu a max minus obstacle and wind accelerations), mu = 0.75, 0.85 for walls. Mention only if asked.

## Slide 25 — Step 10 — solve for the speed and the barrier φ(h) falls out

*0.5 menit · 58 kata*

**EN** Step ten is the core. Solving it for speed gives the barrier phi of h: the fastest approach that can still be stopped. A linear barrier, shown by the pink band, allows approach where the drone cannot stop. Ours respects the real tilt limit and delay, with no free parameter. Below 0.16 metres, the allowed speed is zero.

**ID** Langkah sepuluh adalah intinya. Menyelesaikannya untuk kecepatan menghasilkan barrier phi dari h: kecepatan mendekat tercepat yang masih bisa dihentikan. Barrier linier, ditunjukkan pita merah muda, mengizinkan mendekat di tempat drone tidak bisa berhenti. Barrier kami menghormati batas kemiringan dan jeda yang sebenarnya, tanpa parameter bebas. Di bawah 0,16 meter, kecepatan yang diizinkan nol.

> Arahan: The core of the method. Point at the pink band in the figure: that is the region where the linear function lies above zero but the vehicle cannot stop.

## Slide 26 — 1.67 m

*0.1 menit · 9 kata*

**EN** At 1.6 metres per second, that is 1.67 metres.

**ID** Pada 1,6 meter per detik, itu 1,67 meter.

> Arahan: This is what makes the barrier concrete for the audience, and it is the number the planner inherits on the next section.

## Slide 27 — Step 11 — one hazard, one linear row on the commanded velocity

*0.2 menit · 14 kata*

**EN** Step eleven: every obstacle, wall or neighbour becomes one linear limit on closing speed.

**ID** Langkah sebelas: setiap rintangan, dinding, atau tetangga menjadi satu batas linier pada kecepatan mendekat.

> Arahan: Stress that the row shape is the same for every hazard class – that is what keeps the QP small and the code auditable.

## Slide 28 — Step 12 — stack the rows and the safe velocity is a projection

*0.2 menit · 14 kata*

**EN** Step twelve: the safe velocity is a projection, solved exactly in about 138 microseconds.

**ID** Langkah dua belas: kecepatan aman adalah sebuah proyeksi, diselesaikan secara eksak dalam sekitar 138 mikrodetik.

> Arahan: CBF rows only prevent approach, they never restore separation – that is why the recovery term exists at all.

## Slide 29 — pembatas 5 · High level

## Slide 30 — High level: a polygon goes in, a lane to fly comes out

*0.2 menit · 14 kata*

**EN** The high level splits the region into Voronoi cells and plans lanes inside each.

**ID** Lapis atas membagi area menjadi sel Voronoi dan merencanakan lajur di dalam tiap sel.

> Arahan: Close the loop out loud: high level output = mid level input, mid level output = low level input, and the limits travelled the other way.

## Slide 31 — Step 13 — the planner's keep-out radius is read off the barrier, not chosen

*0.3 menit · 23 kata*

**EN** Step thirteen: the planner's 1.30 metre keep-out radius is read from the barrier, and the detour still sweeps the ring around each obstacle.

**ID** Langkah tiga belas: radius keep-out perencana 1,30 meter dibaca dari barrier, dan jalan memutarnya tetap menyapu cincin di sekitar tiap rintangan.

> Arahan: The 70 degree cap also keeps chords long (1.49 m) so the 0.60 m minimum-segment merge cannot cut the corner back into the keep-out zone.

## Slide 32 — Step 14 — the sensor sets the lane pitch, the region sets the cells

*0.2 menit · 15 kata*

**EN** Step fourteen: the sensor sets the lane spacing, and the region shape sets the cells.

**ID** Langkah empat belas: sensor menentukan jarak antarlajur, dan bentuk area menentukan selnya.

> Arahan: Entry points are chosen combinatorially, not greedily: two neighbours can otherwise be assigned entries exactly 0.70 m apart – the hard separation – and then neither can arrive. Observed: 317 s at 0%.

## Slide 33 — Step 15 — when an agent dies, the same geometry is simply re-run

*0.3 menit · 32 kata*

**EN** Step fifteen: when a drone dies, its cell is merged with its neighbours, re-laned, and handed to helpers. Remaining lanes go out in contiguous blocks, so helpers do not criss-cross the arena.

**ID** Langkah lima belas: saat drone mati, selnya digabung dengan tetangganya, dibuatkan lajur ulang, dan diserahkan ke helper. Lajur sisanya dibagikan dalam blok bersambung, sehingga helper tidak bolak-balik melintasi arena.

> Arahan: On a loss: erase coverage inside the failed cell, merge it with its neighbours, re-lane the union with the same pitch, drop lanes already 80% covered, and hand contiguous serpentine blocks to 1-3 helpers. Contiguity is the key choice – a helper should not shuttle across the arena between lanes.

## Slide 34 — pembatas 6 · Experiment

## Slide 35 — Every stressor applied at once, to seven quadrotors

*0.2 menit · 25 kata*

**EN** We test seven drones in Gazebo, sweeping at 1.6 metres per second, with wind, static and moving obstacles, and two drone losses, all at once.

**ID** Kami menguji tujuh drone di Gazebo, menyapu pada 1,6 meter per detik, dengan angin, rintangan diam dan bergerak, serta dua drone hilang, semuanya sekaligus.

> Arahan: m = 1.50 kg, arm 0.22 m, rotor under 1100 rad/s. Correlation time 0.5 s, wind starts at t = 5 s. Let the results table carry the rest.

## Slide 36 — Every number here traces to a CSV, or it is not shown

*0.1 menit · 10 kata*

**EN** One run per configuration; every number traces to a CSV.

**ID** Satu run per konfigurasi; setiap angka bisa dilacak ke CSV.

> Arahan: This slide buys you the right to show a crash next. Q3 in the pack. These very slides read the same generated macro file as the paper.

## Slide 37 — pembatas 7 · The missions

## Slide 38 — 7 of 8

*0.1 menit · 4 kata*

**EN** Seven of eight finished.

**ID** Tujuh dari delapan selesai.

> Arahan: Then straight into the video section.

## Slide 39 — A full mission: partition, transit, sweep, two losses, return to centroid

*0.4 menit · 42 kata*

**EN** A full mission with H-infinity: partition, transit, then sweep. Green is live coverage. Drones 4 and 7 are killed, their cells are redistributed, and the survivors reach 98.2 percent coverage. It finished in 257 seconds, and the survivors return to their centroids.

**ID** Satu misi penuh dengan H-infinity: pembagian area, perpindahan, lalu menyapu. Hijau adalah cakupan secara langsung. Drone 4 dan 7 dimatikan, selnya dibagikan ulang, dan drone yang tersisa mencapai cakupan 98.2 persen. Misi selesai dalam 257 detik, dan drone yang tersisa kembali ke titik berat selnya.

> Arahan: Play about 30 s then stop — do not play all 263 s. Narrate the phases as they happen. The HUD in the clip carries sim time and live coverage.

## Slide 40 — The swarm never pauses — survivors keep sweeping through a reallocation

*0.2 menit · 13 kata*

**EN** At the first loss, the cell is re-laned and the swarm never pauses.

**ID** Pada kehilangan pertama, selnya dibuatkan lajur ulang dan swarm tidak pernah berhenti.

> Arahan: 24 s clip. Let it run once and talk over it. The coverage dip is visible on the curve two slides later.

## Slide 41 — One mission ended in an obstacle strike, and we report it as such

*0.3 menit · 38 kata*

**EN** The one failure: under LQR, agent 2 came within 0.06 metres of a cylinder and did not recover. We report it as a failure. We stopped the run at 78.8 seconds. It shows where the method runs out.

**ID** Satu-satunya kegagalan: dengan LQR, agen 2 mendekat sampai 0.06 meter dari silinder dan tidak pulih. Kami melaporkannya sebagai kegagalan. Kami menghentikan run pada 78.8 detik. Ini menunjukkan di mana batas metodenya.

> Arahan: Do not apologise for this slide. State it plainly — it is the most scientifically useful result in the set. The cause comes two slides later.

## Slide 42 — pembatas 8 · Results

## Slide 43 — H∞ finished every region; LQR finished three and struck a cylinder in the fourth

*0.5 menit · 42 kata*

**EN** Across all eight missions, H-infinity finished every region, and LQR finished three. Where both finished, H-infinity was 11 to 15 percent faster and tighter on the lane, but used several times more torque. LQR still had the best single coverage, 99.0 percent.

**ID** Di kedelapan misi, H-infinity menyelesaikan semua area, dan LQR menyelesaikan tiga. Di area yang diselesaikan keduanya, H-infinity 11 sampai 15 persen lebih cepat dan lebih rapat di lajur, tetapi memakai torsi beberapa kali lebih besar. LQR tetap mencatat cakupan tunggal terbaik, 99.0 persen.

> Arahan: A few seconds of silence so they can read it. Then name three things: the abort row, the 0.06 m clearance, and that Hinf finished every region.

## Slide 44 — 0.06 m

*0.1 menit · 11 kata*

**EN** The filter saw the crash coming, but ran out of authority.

**ID** Filter melihat tabrakan itu datang, tetapi kehabisan otoritas kendali.

> Arahan: This is the number a reviewer will remember; own it.

## Slide 45 — Coverage goes backwards at every loss — by design

*0.1 menit · 9 kata*

**EN** Coverage drops at each loss by design, then recovers.

**ID** Cakupan turun di setiap kehilangan secara sengaja, lalu pulih.

> Arahan: The backwards step is a design choice, not a plotting artefact. Say so explicitly or someone will think it is a bug.

## Slide 46 — Most of the H∞ advantage is a rejected mean offset, not broadband attenuation

*0.4 menit · 44 kata*

**EN** Why is H-infinity tighter? Mostly not noise rejection. LQR flies with a constant offset of six to seven centimetres, while H-infinity stays within two. The main cause is H-infinity's 3.9 times larger integral gain. The spread of the error is actually similar for both.

**ID** Mengapa H-infinity lebih rapat? Sebagian besar bukan karena peredaman gangguan. LQR terbang dengan offset tetap enam sampai tujuh sentimeter, sedangkan H-infinity tetap dalam dua sentimeter. Penyebab utamanya gain integral H-infinity yang 3.9 kali lebih besar. Sebaran galatnya sebenarnya mirip untuk keduanya.

> Arahan: The slide that separates a careful paper from a marketing one. Median fell from 8.5–8.8 to 2.7–3.8 cm.

## Slide 47 — 55–64%

*0.2 menit · 21 kata*

**EN** The LQR integrator was open only 55 to 64 percent of the time: a tuning choice, not a flaw of LQR.

**ID** Integrator LQR hanya terbuka 55 sampai 64 persen dari waktu: pilihan tuning, bukan kelemahan LQR.

> Arahan: The most interesting finding in the paper and the one that stops an unfair conclusion. Expect Q5 here.

## Slide 48 — Holding the lane tighter costs torque, and the trade is one scatter

*0.2 menit · 20 kata*

**EN** The cost is 3.9 to 5.6 times more torque. Neither controller wins outright. On a battery-limited drone, that cost matters.

**ID** Harganya torsi 3.9 sampai 5.6 kali lebih besar. Tidak ada pengendali yang menang mutlak. Pada drone yang dibatasi baterai, biaya itu penting.

> Arahan: Point at the gap between the blue and orange clusters on the x axis – that is the whole cost argument in one picture.

## Slide 49 — No pair went below 0.70 m — which is not the same as no collisions

*0.2 menit · 24 kata*

**EN** Drones never came closer than 0.79 metres, but we do not claim zero collisions. Rotor guards are assumed, and drone contact is never detected.

**ID** Jarak antar-drone tidak pernah di bawah 0.79 meter, tetapi kami tidak mengklaim nol tabrakan. Pelindung rotor hanya diasumsikan, dan kontak antar-drone tidak pernah dideteksi.

> Arahan: All eight runs logged a negative moving-obstacle barrier, to -0.68 m — Q7 covers it. Q6 and Q7 live here. Saying it first is cheaper than defending it later. Torque effort 3.9–5.6 times higher for Hinf; motor saturation under 0.2% of samples for both.

## Slide 50 — What this work does not establish

*0.1 menit · 13 kata*

**EN** These are single runs with one wind seed, ideal sensing, and centralised planning.

**ID** Ini run tunggal dengan satu seed angin, penginderaan ideal, dan perencanaan tersentralisasi.

> Arahan: Read quickly and without hedging. A short slide that earns trust. Runs are stopped at unscheduled crashes rather than continued.

## Slide 51 — Seven of eight, under every stressor at once

*0.3 menit · 42 kata*

**EN** To conclude: the swarm kept 96.9 to 99.0 percent coverage while losing two of seven drones. Next come more wind seeds, distributed planning, and flight tests. Where both finished, H-infinity held lanes 16 to 39 percent tighter, at several times the torque.

**ID** Sebagai kesimpulan: swarm mempertahankan cakupan 96.9 sampai 99.0 persen meski kehilangan dua dari tujuh drone. Selanjutnya lebih banyak seed angin, perencanaan terdistribusi, dan uji terbang. Di area yang diselesaikan keduanya, H-infinity menjaga lajur 16 sampai 39 persen lebih rapat, dengan torsi beberapa kali lebih besar.

> Arahan: Land on the future work, then go straight to the thank-you slide.

## Slide 52 — Thank you

*0.2 menit · 18 kata*

**EN** Thank you. Code and recordings are public at the link shown. We thank CITA ITB for its support.

**ID** Terima kasih. Kode dan rekaman bisa diakses publik lewat tautan ini. Kami berterima kasih kepada CITA ITB atas dukungannya.

> Arahan: End here. Do NOT advance – the backup section behind this slide holds the anticipated-question pack; jump to it only if a question lands.
