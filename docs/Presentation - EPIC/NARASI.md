# Narasi presentasi EPIC 2026

Dihasilkan `tools/export_narasi.py` dari `\narasi{}` di `main.tex` — **jangan diedit di sini**, edit `main.tex` lalu `make narasi`.
Angka diambil dari `numbers.tex`, file yang sama dengan paper.

Slide 1–52 · 44 slide bicara · 27.0 menit · 3582 kata Inggris (≈ 133 kata/menit).

Pembatas seksi tidak punya narasi: tahan sebentar, lalu klik lanjut.

## Slide 1 — Title

*0.3 menit · 41 kata*

**EN** Hello everyone. I am Izma Alhazmi Herdian, from Institut Teknologi Bandung. Today I present fault-tolerant swarm quadrotor control for mapping non-convex areas. On the right is the whole system in one picture: three control layers on top of a simulated swarm.

**ID** Halo semuanya. Saya Izma Alhazmi Herdian, dari Institut Teknologi Bandung. Hari ini saya mempresentasikan kendali swarm quadrotor yang toleran terhadap kegagalan untuk memetakan area non-konveks. Di sebelah kanan adalah keseluruhan sistem dalam satu gambar: tiga lapis kendali di atas swarm yang disimulasikan.

> Arahan: 20 min + 10 min Q&A. Open by naming the result out loud, then move on. Everything after this explains how — and what failed.

## Slide 2 — Seven quadrotors, four regions, every stressor at once

*0.4 menit · 58 kata*

**EN** Before any equations, this is what the system does. Seven quadrotors split a region into cells, sweep their cells lane by lane, and steer around cylinders they discover with LiDAR, all under strong wind. In each of these four missions, two drones are killed in flight, and the survivors take over their work. Everything after this explains how.

**ID** Sebelum masuk ke persamaan, inilah yang dilakukan sistemnya. Tujuh quadrotor membagi area menjadi sel, menyapu selnya lajur demi lajur, dan menghindari silinder yang mereka temukan dengan LiDAR, semuanya di bawah angin kencang. Di keempat misi ini, dua drone dimatikan saat terbang, dan drone yang tersisa mengambil alih pekerjaannya. Seluruh sisa presentasi menjelaskan bagaimana caranya.

> Arahan: 25 s. Let the clip run and say only what they are looking at: seven drones split the region, sweep lanes, avoid the cylinders, and two of them are killed mid-mission. Everything after this explains how.

## Slide 3 — Seven of eight missions finished after losing 28.6% of the fleet

*0.5 menit · 68 kata*

**EN** So here is the answer first. Seven of our eight missions finished after losing two of seven drones, which is 28.6 percent of the fleet. Coverage was 96.9 to 99.0 percent. The PID H-infinity controller finished 11 to 15 percent sooner than PID-LQR, but at 3.9 to 5.6 times the torque effort. And one mission ended in an obstacle strike. I will show you that one as well.

**ID** Jadi saya sampaikan jawabannya lebih dulu. Tujuh dari delapan misi kami selesai meskipun kehilangan dua dari tujuh drone, yaitu 28,6 persen armada. Cakupannya 96.9 sampai 99.0 persen. Pengendali PID H-infinity selesai 11 sampai 15 persen lebih cepat daripada PID-LQR, tetapi dengan upaya torsi 3.9 sampai 5.6 kali lebih besar. Dan satu misi berakhir dengan menabrak rintangan. Misi itu juga akan saya tunjukkan.

> Arahan: 30 s. Tell them the answer first, then spend the talk earning it. Signalling the crash this early buys credibility for every number after.

## Slide 4 — pembatas 1 · The problem

## Slide 5 — The question is not whether a swarm is faster, but whether it still finishes

*0.7 menit · 112 kata*

**EN** Survey mapping is a coverage problem: every point of the area must pass under a sensor. The rough budget on the right says that sweep time falls as we add drones, but coordination cost rises. Our scenario combines four difficulties at once. The region is non-convex, with notches and arms that break a simple raster. The wind has a mean of about 3.0 metres per second, with gusts up to 12.0. There are nine cylinders the swarm has never seen, plus two moving ones. And two quadrotors are killed mid-mission, at 18 and 32 percent coverage. So the question is not only whether a swarm is faster, but whether it still finishes.

**ID** Pemetaan survei adalah masalah cakupan: setiap titik area harus terlewati sensor. Perkiraan kasar di sebelah kanan menunjukkan bahwa waktu sapuan turun saat drone ditambah, tetapi biaya koordinasinya naik. Skenario kami menggabungkan empat kesulitan sekaligus. Areanya non-konveks, dengan lekukan dan lengan yang merusak pola raster sederhana. Anginnya rata-rata sekitar 3.0 meter per detik, dengan hembusan sampai 12.0. Ada sembilan silinder yang belum pernah dilihat swarm, ditambah dua yang bergerak. Dan dua quadrotor dimatikan di tengah misi, pada cakupan 18 dan 32 persen. Jadi pertanyaannya bukan hanya apakah swarm lebih cepat, tetapi apakah swarm tetap bisa menyelesaikan misinya.

> Arahan: 45 s. Survey mapping is a coverage problem — every point must pass under a sensor. The equation is a budget, not a result; say so. Name the scenario “Scheme 5” once and move on.

## Slide 6 — Four mechanisms, and a campaign honest enough to include a crash

*0.5 menit · 76 kata*

**EN** Our contribution has four parts. First, lanes that follow the cell boundary, so tapered corners are still swept. Second, lane-level reallocation when an agent dies: merge, re-lane, and hand over. Third, a safety filter whose speed limit comes from the real tilt limit of the drone. Fourth, a fair comparison of PID-LQR and PID H-infinity on an identical structure, where only the gains differ. All of it is backed by eight logged missions, including one failure.

**ID** Kontribusi kami ada empat. Pertama, lajur yang mengikuti batas sel, sehingga sudut yang meruncing tetap tersapu. Kedua, realokasi tingkat lajur saat sebuah agen mati: gabungkan, buat lajur ulang, lalu serahkan. Ketiga, filter keselamatan yang batas kecepatannya berasal dari batas kemiringan drone yang sebenarnya. Keempat, perbandingan yang adil antara PID-LQR dan PID H-infinity pada struktur yang identik, yang berbeda hanya gain-nya. Semuanya didukung delapan misi yang terekam, termasuk satu kegagalan.

> Arahan: The slide reviewers remember. 30 s, one breath per bullet. Do not expand on any of them here — each has its own slide later.

## Slide 7 — pembatas 2 · The system as blocks

## Slide 8 — Three nested loops, each one handing a set-point to the next

*0.8 menit · 110 kata*

**EN** Here is the system as blocks. It has three nested layers, and each one hands a set-point to the next. The high level is event-driven. It takes the region polygon, the drone positions and any drone failures, and outputs a lane to follow. The mid level runs at 20 hertz. It takes the desired velocity, the LiDAR obstacle map and the neighbours, and outputs a safe velocity. The low level runs at 250 hertz. It takes a position set-point and the measured state, and outputs four rotor speeds. We will walk through this diagram backwards, because every limit at the top is derived from what the rotors can actually do.

**ID** Ini sistemnya dalam bentuk blok. Ada tiga lapis bertingkat, dan tiap lapis menyerahkan set-point ke lapis berikutnya. Lapis atas berbasis kejadian. Masukannya poligon area, posisi drone, dan kegagalan drone, dan keluarannya satu lajur untuk diikuti. Lapis tengah berjalan pada 20 hertz. Masukannya kecepatan yang diinginkan, peta rintangan dari LiDAR, dan drone tetangga, dan keluarannya kecepatan yang aman. Lapis bawah berjalan pada 250 hertz. Masukannya set-point posisi dan state terukur, dan keluarannya empat kecepatan rotor. Kita akan menelusuri diagram ini dari belakang, karena setiap batas di lapis atas diturunkan dari kemampuan rotor yang sebenarnya.

> Arahan: Spend 45 s. Read the three columns aloud left to right, then say the sentence that sets up the rest of the talk: we are going to walk this diagram BACKWARDS, because every limit upstairs is derived from what the rotors can actually do.

## Slide 9 — We derive the stack backwards: what the rotors allow decides what the planner may ask

*0.6 menit · 87 kata*

**EN** This is the promise of the talk. At the low level, the tilt limit gives us a simple velocity model of the drone. At the mid level, that model gives a stopping distance, and from it a barrier function that becomes one constraint per hazard. At the high level, the same barrier and the sensor footprint set the keep-out radius and the lane spacing. Nothing at the top is tuned by hand. One honest note: the simulation uses ground-truth odometry, with no PX4 and no state estimator.

**ID** Inilah janji presentasi ini. Di lapis bawah, batas kemiringan memberi kita model kecepatan drone yang sederhana. Di lapis tengah, model itu memberi jarak berhenti, dan darinya fungsi barrier yang menjadi satu kendala untuk tiap bahaya. Di lapis atas, barrier yang sama dan jangkauan sensor menentukan radius keep-out dan jarak antarlajur. Tidak ada yang disetel tangan di lapis atas. Satu catatan jujur: simulasinya memakai odometri ground-truth, tanpa PX4 dan tanpa estimator state.

> Arahan: This slide is the promise of the talk. Say it plainly: no magic constants; every number upstairs has a derivation. Also mention what is NOT in the loop – ground-truth odometry, no PX4, no estimator; it is on the Limitations slide and in Q2.

## Slide 10 — pembatas 3 · Low level

## Slide 11 — Low level: a position in metres goes in, four rotor speeds come out

*0.7 menit · 87 kata*

**EN** We start at the bottom. The low level receives a position set-point in metres from the mid level, plus the measured position, velocity, attitude and body rates. It runs a fixed chain: a pre-filter, an outer PID, an inner PD and a motor mixer, every four milliseconds, with no optimisation at run time. It outputs four rotor speeds, bounded between 250 and 1050 radians per second. Notice the two feedback paths: the inner loop sees attitude, the outer loop sees position, and both run at 250 hertz.

**ID** Kita mulai dari bawah. Lapis bawah menerima set-point posisi dalam meter dari lapis tengah, ditambah posisi, kecepatan, sikap, dan laju sudut yang terukur. Ia menjalankan rantai yang tetap: pre-filter, PID luar, PD dalam, dan mixer motor, setiap empat milidetik, tanpa optimasi saat berjalan. Keluarannya empat kecepatan rotor, dibatasi antara 250 dan 1050 radian per detik. Perhatikan dua jalur umpan baliknya: loop dalam melihat sikap, loop luar melihat posisi, dan keduanya berjalan pada 250 hertz.

> Arahan: 45 s. Read the three boxes left to right, then trace the chain. The two feedback paths are the point: the inner loop sees attitude, the outer loop sees position, and they run at the same 250 Hz.

## Slide 12 — Step 1 — the only horizontal force a quadrotor has is tilted thrust

*0.8 menit · 112 kata*

**EN** Step one. A quadrotor has only one force, the thrust along its body axis. To hover, the vertical part of the thrust, T cos theta, must equal the weight. The only horizontal force left is T sin theta. Divide the two, and the mass drops out: horizontal acceleration depends only on the tilt angle, a max equals g times tan theta max. We cap the tilt at 15 degrees, because the mapping sensor needs a steady attitude. That is a design choice, not physics, but once it is made it becomes a hard ceiling on acceleration. Note that it caps the attitude reference, so measured tilt can exceed it at lane-end turns.

**ID** Langkah satu. Quadrotor hanya punya satu gaya, yaitu thrust sepanjang sumbu badannya. Agar melayang, komponen vertikal thrust, T cos theta, harus sama dengan berat. Satu-satunya gaya horizontal yang tersisa adalah T sin theta. Bagi keduanya, dan massanya hilang: percepatan horizontal hanya bergantung pada sudut kemiringan, a max sama dengan g kali tan theta max. Kami membatasi kemiringan di 15 derajat, karena sensor pemetaan butuh sikap yang stabil. Itu pilihan desain, bukan hukum fisika, tetapi begitu dipilih, ia menjadi batas keras percepatan. Perlu dicatat, yang dibatasi adalah referensi sikap, sehingga kemiringan terukur bisa melebihinya saat berbelok di ujung lajur.

> Arahan: 50 s. The algebra is two lines; do not labour it. The point is that one design choice – 15 degrees – becomes a hard acceleration ceiling.

## Slide 13 — 2.631 m/s²

*0.2 menit · 24 kata*

**EN** So the drone has 2.631 metres per second squared of horizontal acceleration, about a quarter of g. Every slide that follows uses this number.

**ID** Jadi drone ini punya percepatan horizontal 2.631 meter per detik kuadrat, sekitar seperempat g. Setiap slide berikutnya memakai angka ini.

> Arahan: 12 s. Say the number, pause, move on. It is the hinge of the whole talk.

## Slide 14 — Step 2 — twelve coupled states become four simple chains

*0.8 menit · 98 kata*

**EN** Step two. A quadrotor has twelve states: position, velocity, attitude and body rates. They are nonlinear and coupled, because you tilt to move, and the thrust direction tilts with you. If we linearise around hover with small angles, those couplings vanish to first order. What remains is four independent chains: X through pitch, Y through roll, Z through collective thrust, and yaw through the yaw torque. This is also why the 15 degree cap matters. The upsets we measured, 82.5 and 155.9 degrees, lie far outside this model, so we report them as events, not as tracking error.

**ID** Langkah dua. Quadrotor punya dua belas state: posisi, kecepatan, sikap, dan laju sudut. Semuanya nonlinier dan saling terkopel, karena untuk bergerak drone harus miring, dan arah thrust ikut miring. Jika kita linierkan di sekitar kondisi melayang dengan sudut kecil, kopling itu hilang pada orde pertama. Yang tersisa adalah empat rantai independen: X lewat pitch, Y lewat roll, Z lewat thrust kolektif, dan yaw lewat torsi yaw. Ini juga alasan batas 15 derajat penting. Upset yang kami ukur, 82.5 dan 155.9 derajat, jauh di luar model ini, sehingga kami melaporkannya sebagai kejadian, bukan sebagai galat tracking.

> Arahan: 50 s. For a non-control audience this is the bridge: do not let them wonder why a quadrotor became a 2x2 matrix. Say the word 'decoupled' once and point at the table.

## Slide 15 — Step 3 — augment with the input, and the integral term falls out

*1.2 menit · 159 kata*

**EN** Step three. Each chain is a double integrator: A is zero one, zero zero, and B carries a single gain b, which is g for the outer loops and one over the inertia for the inner loops. We then augment the state with the input itself. The design variable becomes the rate of change of the input, u dot. Feeding back a rate means we have already integrated once, and that is where the integral term comes from. On this augmented plant, LQR solves the Riccati equation with weights Q and R, and H-infinity solves the H-infinity Riccati equation with gamma set to 1.3 times its minimum, plus a stability check. The resulting state-feedback gain is then projected onto a PID structure, which gives us Kp, Ki and Kd. So the integral gain is not a tuning knob. It is a consequence of the design. And the two controllers differ only in Q, R and gamma, never in structure.

**ID** Langkah tiga. Tiap rantai adalah integrator ganda: A bernilai nol satu, nol nol, dan B membawa satu gain b, yaitu g untuk loop luar dan satu per inersia untuk loop dalam. Lalu state-nya kami augmentasi dengan masukan itu sendiri. Variabel desainnya menjadi laju perubahan masukan, u dot. Mengumpanbalikkan laju berarti kita sudah mengintegrasikan satu kali, dan dari situlah suku integral berasal. Pada plant teraugmentasi ini, LQR menyelesaikan persamaan Riccati dengan bobot Q dan R, dan H-infinity menyelesaikan persamaan Riccati H-infinity dengan gamma 1,3 kali nilai minimumnya, ditambah pengecekan kestabilan. Gain umpan balik state yang dihasilkan lalu diproyeksikan ke struktur PID, sehingga kita mendapat Kp, Ki, dan Kd. Jadi gain integral bukan tombol tuning. Ia konsekuensi dari desain. Dan kedua pengendali hanya berbeda pada Q, R, dan gamma, tidak pernah pada strukturnya.

> Arahan: 60 s, the densest slide of the talk. Q for the outer loops is diag(0.08, 0.35) with R = 1; for the inner loops diag(6, 0.6) with R = 0.05. Gamma is found by bisection. Do not read those out.

## Slide 16 — Step 4 — same augmented plant, two Riccati equations

*1.0 menit · 125 kata*

**EN** Step four puts the two designs side by side. Both start from the same augmented plant. PID-LQR minimises a quadratic cost, so it is optimal for the model you wrote down. PID H-infinity looks for a controller that keeps the worst-case gain from disturbance to output below gamma, so it is optimal against the worst disturbance. In the code, both are solved by the same Riccati routine. For H-infinity we stack the disturbance input next to the control input and give it a negative weight, minus gamma squared. That negative block is what turns cheapest into safest against the worst case. Every solution is checked for stability before it is accepted. So the comparison later is fair: same plant, same structure, same solver, different objective.

**ID** Langkah empat menyandingkan kedua desain. Keduanya berangkat dari plant teraugmentasi yang sama. PID-LQR meminimalkan biaya kuadratik, sehingga optimal untuk model yang kita tuliskan. PID H-infinity mencari pengendali yang menjaga gain terburuk dari gangguan ke keluaran tetap di bawah gamma, sehingga optimal terhadap gangguan terburuk. Di kode, keduanya diselesaikan oleh rutin Riccati yang sama. Untuk H-infinity, masukan gangguan ditumpuk di samping masukan kendali dan diberi bobot negatif, minus gamma kuadrat. Blok negatif itulah yang mengubah paling murah menjadi paling aman terhadap kasus terburuk. Setiap solusi dicek kestabilannya sebelum diterima. Jadi perbandingan nanti adil: plant sama, struktur sama, solver sama, hanya tujuannya yang berbeda.

> Arahan: 55 s. Do not derive either equation. The message is that the two designs pose the SAME augmented plant and differ only in which Riccati equation they solve – and in Q, R, gamma.

## Slide 17 — Step 5 — project the state feedback onto a PID, and that is what flies

*0.9 menit · 123 kata*

**EN** Step five. The optimal controller is a state feedback, but what flies is a PID. We bridge the two with a pseudo-inverse, gamma plus, which projects the optimal gain onto what a PID can express. So the gains are not fitted by hand. They are the closest PID to an optimal controller. What runs on the drone, 250 times a second, is the law at the bottom: the attitude reference is Kp times the error, plus the integral, minus Kd times velocity, plus a velocity feedforward, saturated at 15 degrees. One detail matters a lot later: the LQR integrator is only active when the error is below 25 centimetres, while the H-infinity integrator is always active. That one asymmetry returns in the results.

**ID** Langkah lima. Pengendali optimalnya berupa umpan balik state, tetapi yang terbang adalah PID. Keduanya kami jembatani dengan pseudo-invers, gamma plus, yang memproyeksikan gain optimal ke bentuk yang bisa dinyatakan PID. Jadi gain-nya tidak dicocokkan secara manual. Ia adalah PID yang paling dekat dengan pengendali optimal. Yang berjalan di drone, 250 kali per detik, adalah hukum kendali di bagian bawah: referensi sikap sama dengan Kp kali galat, ditambah integral, dikurangi Kd kali kecepatan, ditambah feedforward kecepatan, disaturasi di 15 derajat. Satu detail sangat penting nanti: integrator LQR hanya aktif saat galat di bawah 25 sentimeter, sedangkan integrator H-infinity selalu aktif. Asimetri inilah yang muncul kembali di hasil.

> Arahan: 50 s. The callout is the single most important line of the low level: it is the actual control law, and it names the asymmetry that explains the headline result.

## Slide 18 — Step 6 — the outer PID plus a lead reference collapse into a velocity plant

*1.2 menit · 142 kata*

**EN** Step six turns this controller into a simple velocity model. For small angles, horizontal acceleration is roughly g times the attitude reference. Now, the coordinator never sends a velocity. It sends a position that leads the drone by T lead times the commanded velocity, so the position error is the command. Substitute that into the control law, and the acceleration becomes a gain on the commanded velocity, minus k v times the actual velocity, where k v equals g times Kd. If we choose T lead so that the two gains are equal, T lead equals Kd minus k ff, divided by Kp, then the DC gain is exactly one. In plain words: the speed we command is the speed the drone actually holds, and the whole closed loop becomes one first-order equation. That is what the safety filter will build on.

**ID** Langkah enam mengubah pengendali ini menjadi model kecepatan yang sederhana. Untuk sudut kecil, percepatan horizontal kira-kira g kali referensi sikap. Nah, koordinator tidak pernah mengirim kecepatan. Ia mengirim posisi yang mendahului drone sejauh T lead kali kecepatan perintah, sehingga galat posisi itulah perintahnya. Substitusikan ke hukum kendali, dan percepatannya menjadi sebuah gain pada kecepatan perintah, dikurangi k v kali kecepatan aktual, dengan k v sama dengan g kali Kd. Jika T lead dipilih agar kedua gain sama, yaitu T lead sama dengan Kd dikurangi k ff, dibagi Kp, maka gain DC-nya tepat satu. Dengan kata lain: kecepatan yang kita perintahkan adalah kecepatan yang benar-benar dipertahankan drone, dan seluruh loop tertutup menjadi satu persamaan orde satu. Di atas inilah filter keselamatan dibangun.

> Arahan: The longest slide of the talk, 60 s. Walk the three lines. The payoff is on the next slide: four numbers that everything else is built on.

## Slide 19 — Step 7 — four numbers, and they come from the gains, not from tuning

*0.5 menit · 69 kata*

**EN** Step seven. The low level is now summarised by four numbers, and all four come from the gains, not from tuning: k v is 3.867 per second, a max is 2.631, T lead is 0.323 seconds, and v c, where saturation stops binding, is 0.680 metres per second. Change a gain or the tilt cap, and all four move together. These four numbers are the interface to everything above.

**ID** Langkah tujuh. Lapis bawah sekarang diringkas oleh empat angka, dan keempatnya berasal dari gain, bukan dari tuning: k v sebesar 3.867 per detik, a max 2.631, T lead 0.323 detik, dan v c, titik saat saturasi berhenti membatasi, 0.680 meter per detik. Ubah satu gain atau batas kemiringan, maka keempatnya ikut berubah. Empat angka inilah antarmuka ke semua lapis di atasnya.

> Arahan: 40 s. These four numbers are the interface between the low level and everything above. Table 1 of the paper has the full gain set; do not put it on screen unless asked.

## Slide 20 — What Q, R and γ actually bought, before any flight

*0.6 menit · 81 kata*

**EN** What did the H-infinity weights buy, before any flight? The worst-case gain from disturbance to position drops 4.0 times, from 0.158 to 0.040. The predicted deviation under this wind falls 3.7 times. It is paid for with bandwidth, as crossover rises from 4.09 to 9.08 radians per second, and with margin, as phase margin drops from 37.4 to 25.2 degrees. But this is a prediction from the linear model, not a result. The flight data will tell a more nuanced story.

**ID** Apa yang didapat dari bobot H-infinity, sebelum terbang sama sekali? Gain terburuk dari gangguan ke posisi turun 4.0 kali, dari 0.158 menjadi 0.040. Simpangan yang diprediksi di bawah angin ini turun 3.7 kali. Harganya dibayar dengan bandwidth, frekuensi crossover naik dari 4.09 ke 9.08 radian per detik, dan dengan margin, phase margin turun dari 37.4 ke 25.2 derajat. Tetapi ini prediksi dari model linier, bukan hasil. Data terbang akan menunjukkan cerita yang lebih bernuansa.

> Arahan: 35 s. Set up the honest caveat early: the model predicts broadband attenuation, but the flight data will show the gain is mostly a rejected mean offset. Full gain table is in backup.

## Slide 21 — pembatas 4 · Mid level

## Slide 22 — Mid level: a wish in metres per second goes in, a safe one comes out

*0.7 menit · 91 kata*

**EN** Now the mid level. It receives a desired velocity from the lane follower, the confirmed obstacle map from LiDAR, and the positions and velocities of the neighbours. It builds one linear constraint per hazard from a barrier function, and solves a small quadratic program, the only optimisation in the whole stack. It outputs a safe velocity, which becomes the next position set-point. What is optimised is only how far we deviate from what the planner asked. The safety constraints are hard. In short: the state machine proposes, and the QP disposes.

**ID** Sekarang lapis tengah. Masukannya kecepatan yang diinginkan dari pengikut lajur, peta rintangan terkonfirmasi dari LiDAR, serta posisi dan kecepatan drone tetangga. Ia membangun satu kendala linier untuk tiap bahaya dari fungsi barrier, lalu menyelesaikan program kuadratik kecil, satu-satunya optimasi di seluruh tumpukan kendali. Keluarannya kecepatan aman, yang menjadi set-point posisi berikutnya. Yang dioptimasi hanyalah seberapa jauh kita menyimpang dari permintaan perencana. Kendala keselamatannya bersifat keras. Singkatnya: state machine mengusulkan, QP yang memutuskan.

> Arahan: 45 s. The key sentence: the state machine proposes, the QP disposes, and the only thing traded away is how closely we honour the wish.

## Slide 23 — Step 8 — integrate the plant and you get an exact stopping distance

*0.7 menit · 90 kata*

**EN** Step eight. Take the velocity model from step seven and command zero. The drone decelerates at the smaller of k v times v and a max. At high speed it is acceleration-limited, so the stopping distance is a parabola. At low speed it decays exponentially. The two regimes meet exactly at v c. Integrating gives the exact stopping distance, D of s, on the right. This is not a bound. It is the integral of the identified plant, so the safety argument adds no extra conservatism, and no optimism either.

**ID** Langkah delapan. Ambil model kecepatan dari langkah tujuh dan perintahkan nol. Drone melambat dengan nilai yang lebih kecil antara k v kali v dan a max. Pada kecepatan tinggi ia dibatasi percepatan, sehingga jarak berhentinya berbentuk parabola. Pada kecepatan rendah ia meluruh secara eksponensial. Kedua rezim bertemu tepat di v c. Integrasinya memberi jarak berhenti yang eksak, D dari s, di sebelah kanan. Ini bukan batas atas. Ini integral dari plant yang teridentifikasi, sehingga argumen keselamatannya tidak menambah konservatisme, dan juga tidak terlalu optimis.

> Arahan: 45 s. Emphasise that nothing has been approximated yet beyond the small angle step.

## Slide 24 — Step 9 — add dead time, and safety becomes one inequality

*0.6 menit · 85 kata*

**EN** Step nine adds reality: the drone reacts late. Sensing, the QP, the set-point hand-off and actuator lag add up to a dead time of 0.4 seconds. During that window the drone keeps closing at speed s, and only afterwards does braking begin. So the clearance h must cover both the dead-time distance and the stopping distance. That gives one inequality, linear in h and quadratic in s. Read it as a speed limit instead of a distance check, and the controller can act on it.

**ID** Langkah sembilan menambahkan kenyataan: drone bereaksi terlambat. Penginderaan, QP, penyerahan set-point, dan jeda aktuator berjumlah dead time 0,4 detik. Selama jeda itu drone terus mendekat dengan kecepatan s, dan baru sesudahnya pengereman dimulai. Jadi jarak bebas h harus mencakup jarak selama dead time dan jarak berhenti. Hasilnya satu pertidaksamaan, linier terhadap h dan kuadratik terhadap s. Baca ia sebagai batas kecepatan, bukan pengecekan jarak, dan pengendali bisa bertindak berdasarkan itu.

> Arahan: 45 s. The budget a is not quite a max: a = max(0.15 a max, mu a max minus obstacle and wind accelerations), mu = 0.75, 0.85 for walls. Mention only if asked.

## Slide 25 — Step 10 — solve for the speed and the barrier φ(h) falls out

*1.3 menit · 153 kata*

**EN** Step ten is the core of the method. Solve that inequality for the speed, and the barrier function phi of h falls out. Phi gives the fastest approach speed that can still be stopped in time at clearance h. It has units of speed, and it is an extended class-K function, which is exactly what a control barrier function needs. Below a clearance of 0.16 metres, phi is zero: at that distance the drone must already be stopped. Below zero clearance it becomes negative and actively commands separation. Why not the usual linear function, gamma times h? Look at the pink band in the figure. A linear function matched at 1.6 metres per second still allows approach inside 0.16 metres, where the drone physically cannot stop. It fails exactly where it matters. Our barrier respects the real tilt limit and the real dead time, so there is no free parameter left to tune.

**ID** Langkah sepuluh adalah inti metodenya. Selesaikan pertidaksamaan tadi untuk kecepatan, maka fungsi barrier phi dari h muncul dengan sendirinya. Phi memberi kecepatan mendekat tercepat yang masih bisa dihentikan tepat waktu pada jarak bebas h. Satuannya kecepatan, dan ia fungsi kelas-K yang diperluas, persis yang dibutuhkan control barrier function. Di bawah jarak bebas 0,16 meter, phi bernilai nol: pada jarak itu drone harus sudah berhenti. Di bawah nol ia menjadi negatif dan secara aktif memerintahkan drone menjauh. Mengapa tidak memakai fungsi linier biasa, gamma kali h? Lihat pita merah muda di gambar. Fungsi linier yang disamakan pada 1,6 meter per detik masih mengizinkan drone mendekat di dalam 0,16 meter, padahal di situ drone secara fisik tidak bisa berhenti. Ia gagal tepat di tempat yang paling penting. Barrier kami menghormati batas kemiringan dan dead time yang sebenarnya, sehingga tidak ada parameter bebas yang perlu disetel.

> Arahan: The core of the method, 65 s. Point at the pink band in the figure: that is the region where the linear function lies above zero but the vehicle cannot stop.

## Slide 26 — 1.67 m

*0.2 menit · 32 kata*

**EN** To make it concrete: approaching at 1.6 metres per second needs 1.67 metres of clearance. At one metre per second it needs 0.9 metres. At 0.16 metres, the allowed speed is zero.

**ID** Supaya konkret: mendekat dengan 1,6 meter per detik butuh jarak bebas 1,67 meter. Pada satu meter per detik butuh 0,9 meter. Pada 0,16 meter, kecepatan yang diizinkan adalah nol.

> Arahan: 12 s. This is what makes the barrier concrete for the audience, and it is the number the planner inherits on the next section.

## Slide 27 — Step 11 — one hazard, one linear row on the commanded velocity

*0.5 menit · 76 kata*

**EN** Step eleven. Each hazard gives one linear row on the commanded velocity: the speed toward the hazard must stay below lambda times phi of h. Only the closing component is limited, so moving sideways is still free. Static cylinders, moving cylinders, walls and neighbours all produce rows of this same shape. For pairs of drones the burden is shared, so the two lambdas add up to one, which is why all seven drones are solved together.

**ID** Langkah sebelas. Tiap bahaya menghasilkan satu baris linier pada kecepatan perintah: kecepatan menuju bahaya harus tetap di bawah lambda kali phi dari h. Hanya komponen yang mendekat yang dibatasi, jadi bergerak ke samping tetap bebas. Silinder diam, silinder bergerak, dinding, dan tetangga semuanya menghasilkan baris dengan bentuk yang sama. Untuk pasangan drone, bebannya dibagi sehingga kedua lambda berjumlah satu, dan karena itulah ketujuh drone diselesaikan bersamaan.

> Arahan: 40 s. Stress that the row shape is the same for every hazard class – that is what keeps the QP small and the code auditable.

## Slide 28 — Step 12 — stack the rows and the safe velocity is a projection

*0.7 menit · 88 kata*

**EN** Step twelve. Stack all the rows, and the safe velocity is simply the admissible velocity closest to what the planner asked: a projection. With only two variables, we solve it exactly by enumerating active sets, in about 138 microseconds for 24 rows, with no external solver. On 3000 random test cases, SLSQP failed 320 times and this solver none. A separate recovery term breaks deadlocks, but it only touches the objective, never the constraints, so liveness is never traded against safety. Each mission ran 19,500 to 25,900 solves.

**ID** Langkah dua belas. Tumpuk semua baris, maka kecepatan aman adalah kecepatan yang diizinkan dan paling dekat dengan permintaan perencana: sebuah proyeksi. Karena hanya dua variabel, kami menyelesaikannya secara eksak dengan enumerasi active set, sekitar 138 mikrodetik untuk 24 baris, tanpa solver eksternal. Pada 3000 kasus uji acak, SLSQP gagal 320 kali dan solver ini tidak sekali pun. Suku pemulihan terpisah memecah deadlock, tetapi hanya menyentuh fungsi tujuan, tidak pernah kendalanya, sehingga kelancaran gerak tidak pernah ditukar dengan keselamatan. Tiap misi menjalankan 19.500 sampai 25.900 kali penyelesaian.

> Arahan: 40 s. CBF rows only prevent approach, they never restore separation – that is why the recovery term exists at all.

## Slide 29 — pembatas 5 · High level

## Slide 30 — High level: a polygon goes in, a lane to fly comes out

*0.5 menit · 69 kata*

**EN** Now the high level. It takes the region polygon, the drone positions, a coverage grid and failure events. It partitions the region into Voronoi cells, plans boustrophedon lanes, and routes them around known obstacles. It outputs one lane at a time, which becomes the desired velocity for the mid level. It has no clock: it only runs when a lane ends, an obstacle is confirmed, or a drone dies.

**ID** Sekarang lapis atas. Masukannya poligon area, posisi drone, grid cakupan, dan kejadian kegagalan. Ia membagi area menjadi sel Voronoi, merencanakan lajur boustrophedon, dan membelokkan lajur itu di sekitar rintangan yang sudah diketahui. Keluarannya satu lajur setiap kali, yang menjadi kecepatan yang diinginkan untuk lapis tengah. Ia tidak punya jam: ia hanya berjalan saat sebuah lajur selesai, rintangan terkonfirmasi, atau sebuah drone mati.

> Arahan: 40 s. Close the loop out loud: high level output = mid level input, mid level output = low level input, and the limits travelled the other way.

## Slide 31 — Step 13 — the planner's keep-out radius is read off the barrier, not chosen

*0.8 menit · 114 kata*

**EN** Step thirteen. The planner's keep-out radius is read from the barrier, not chosen. The physical need is the cylinder radius plus the airframe plus a margin: 0.87 metres. But a detour is flown as straight chords of an arc, and a chord cuts inside the arc. With the chord angle capped at 70 degrees, routing at 1.30 metres keeps the path at least 1.065 metres away, which is still clear. And with a 0.95 metre sensor, the ring around each cylinder is still swept, so the detour costs no coverage. The planner handles what is known. Between a LiDAR discovery and the re-plan, the safety filter is the only thing keeping the drone safe.

**ID** Langkah tiga belas. Radius keep-out perencana dibaca dari barrier, bukan dipilih. Kebutuhan fisiknya adalah radius silinder ditambah badan drone ditambah margin: 0,87 meter. Tetapi jalan memutar diterbangkan sebagai tali busur lurus dari sebuah lingkaran, dan tali busur memotong ke dalam lingkaran. Dengan sudut tali busur dibatasi 70 derajat, merutekan pada 1,30 meter menjaga lintasan minimal 1,065 meter dari pusat, yang masih aman. Dan dengan sensor 0,95 meter, cincin di sekitar tiap silinder tetap tersapu, sehingga jalan memutar tidak mengurangi cakupan. Perencana menangani yang sudah diketahui. Di antara penemuan oleh LiDAR dan perencanaan ulang, filter keselamatan adalah satu-satunya yang menjaga drone tetap aman.

> Arahan: 50 s. The 70 degree cap also keeps chords long (1.49 m) so the 0.60 m minimum-segment merge cannot cut the corner back into the keep-out zone.

## Slide 32 — Step 14 — the sensor sets the lane pitch, the region sets the cells

*0.6 menit · 91 kata*

**EN** Step fourteen. The sensor sets the lane spacing: a 0.95 metre radius gives a 1.9 metre swath, and lanes at no more than 1.45 metres leave overlap, so small heading errors open no gaps. The region sets the cells: Lloyd's algorithm moves each generator to its cell centroid, 25 times, and then a Hungarian assignment matches drones to cells. Each cell is clipped to the region polygon first, so nothing assumes a rectangle. And the first and last lanes follow the cell's own boundary, to sweep the wedges at tapered corners.

**ID** Langkah empat belas. Sensor menentukan jarak antarlajur: radius 0,95 meter memberi lebar sapuan 1,9 meter, dan lajur dengan jarak paling jauh 1,45 meter menyisakan tumpang tindih, sehingga galat arah kecil tidak menimbulkan celah. Area menentukan selnya: algoritma Lloyd memindahkan tiap generator ke titik berat selnya, 25 kali, lalu penugasan Hungarian memasangkan drone dengan sel. Tiap sel dipotong dulu ke poligon area, jadi tidak ada yang mengasumsikan persegi panjang. Dan lajur pertama dan terakhir mengikuti batas sel itu sendiri, untuk menyapu irisan di sudut yang meruncing.

> Arahan: 40 s. Entry points are chosen combinatorially, not greedily: two neighbours can otherwise be assigned entries exactly 0.70 m apart – the hard separation – and then neither can arrive. Observed: 317 s at 0%.

## Slide 33 — Step 15 — when an agent dies, the same geometry is simply re-run

*0.6 menit · 89 kata*

**EN** Step fifteen is the fault tolerance. When a drone dies, the same geometry is simply run again. Coverage inside the failed cell is erased, the cell is merged with its neighbours, and the union is re-laned with the same spacing. Lanes that are already 80 percent covered are dropped. The remaining lanes are handed out in contiguous blocks to one to three helpers, so a helper never shuttles back and forth across the arena. The figure shows seven agents with 54 lanes, then the first loss, then the second.

**ID** Langkah lima belas adalah toleransi kegagalannya. Saat sebuah drone mati, geometri yang sama cukup dijalankan ulang. Cakupan di dalam sel yang gagal dihapus, sel itu digabung dengan tetangganya, dan gabungannya dibuatkan lajur ulang dengan jarak yang sama. Lajur yang sudah 80 persen tercakup dibuang. Lajur sisanya dibagikan dalam blok yang bersambung kepada satu sampai tiga helper, sehingga helper tidak bolak-balik melintasi arena. Gambar ini menunjukkan tujuh agen dengan 54 lajur, lalu kehilangan pertama, lalu yang kedua.

> Arahan: On a loss: erase coverage inside the failed cell, merge it with its neighbours, re-lane the union with the same pitch, drop lanes already 80% covered, and hand contiguous serpentine blocks to 1-3 helpers. Contiguity is the key choice – a helper should not shuttle across the arena between lanes.

## Slide 34 — pembatas 6 · Experiment

## Slide 35 — Every stressor applied at once, to seven quadrotors

*0.6 menit · 83 kata*

**EN** The experiment applies every stressor at once to seven 3DR Iris quadrotors in Gazebo with ROS 2. They cruise at two metres and sweep at 1.6 metres per second. The wind is a Gauss-Markov process with 2.5 metres per second of turbulence plus a constant component. The recorded mean was about -2.34 and -1.95 metres per second, with peaks of 12.0. Two cylinders move diagonally at up to 2.5 metres per second, and two drones are killed at 18 and 32 percent coverage.

**ID** Eksperimennya menerapkan semua gangguan sekaligus pada tujuh quadrotor 3DR Iris di Gazebo dengan ROS 2. Drone terbang di ketinggian dua meter dan menyapu dengan 1,6 meter per detik. Anginnya proses Gauss-Markov dengan turbulensi 2,5 meter per detik ditambah komponen konstan. Rata-rata terekam sekitar -2.34 dan -1.95 meter per detik, dengan puncak 12.0. Dua silinder bergerak diagonal hingga 2,5 meter per detik, dan dua drone dimatikan pada cakupan 18 dan 32 persen.

> Arahan: 45 s. m = 1.50 kg, arm 0.22 m, rotor under 1100 rad/s. Correlation time 0.5 s, wind starts at t = 5 s. Let the results table carry the rest.

## Slide 36 — Every number here traces to a CSV, or it is not shown

*0.4 menit · 56 kata*

**EN** Every number in this talk traces to a CSV file. We ran four regions times two controllers. Each run has a manifest, and one analysis script generates every figure. To be clear, that is one run per configuration with one wind seed, so these are single-run observations. Missing data prints n/a and is never filled in.

**ID** Setiap angka di presentasi ini bisa dilacak ke sebuah file CSV. Kami menjalankan empat area dikali dua pengendali. Tiap run punya manifest, dan satu skrip analisis menghasilkan semua gambar. Supaya jelas, ini satu run per konfigurasi dengan satu seed angin, jadi ini pengamatan run tunggal. Data yang hilang ditulis n/a dan tidak pernah diisi dengan nilai pengganti.

> Arahan: This slide buys you the right to show a crash next. Q3 in the pack. These very slides read the same generated macro file as the paper.

## Slide 37 — pembatas 7 · The missions

## Slide 38 — 7 of 8

*0.2 menit · 19 kata*

**EN** Seven of eight missions finished, after losing two of seven quadrotors. The eighth one, we are going to watch.

**ID** Tujuh dari delapan misi selesai, setelah kehilangan dua dari tujuh quadrotor. Misi kedelapan akan kita tonton bersama.

> Arahan: 12 s. Then straight into the video section.

## Slide 39 — A full mission: partition, transit, sweep, two losses, return to centroid

*0.8 menit · 94 kata*

**EN** This is a full mission in the rectangular region with PID H-infinity. First, the region is partitioned into seven cells, and each drone transits to the start of its first lane. Then they sweep, lane by lane. The green area on the right is the live coverage map. Watch the counter at the top: drone 4 is killed, and its cell is redistributed. Later drone 7 is killed as well, and the survivors still finish. This mission reached 98.2 percent coverage in 257 seconds, and at the end the survivors return to their centroids.

**ID** Ini satu misi penuh di area persegi dengan PID H-infinity. Pertama, area dibagi menjadi tujuh sel, dan tiap drone berpindah ke awal lajur pertamanya. Lalu mereka menyapu, lajur demi lajur. Area hijau di sebelah kanan adalah peta cakupan secara langsung. Perhatikan penghitung di atas: drone 4 dimatikan, dan selnya dibagikan ulang. Kemudian drone 7 juga dimatikan, dan drone yang tersisa tetap menyelesaikan misi. Misi ini mencapai cakupan 98.2 persen dalam 257 detik, dan di akhir drone yang tersisa kembali ke titik berat selnya.

> Arahan: Play about 30 s then stop — do not play all 263 s. Narrate the phases as they happen. The HUD in the clip carries sim time and live coverage.

## Slide 40 — The swarm never pauses — survivors keep sweeping through a reallocation

*0.5 menit · 58 kata*

**EN** Here is a closer look at the first loss. Agent 4 is killed at 18 percent coverage. Coverage inside its cell is erased, so the counter actually drops. Its cell is merged and re-laned, and the lanes are handed to helpers. The important point is that the swarm never pauses: the survivors keep sweeping while the reallocation happens.

**ID** Ini tampilan lebih dekat dari kehilangan pertama. Agen 4 dimatikan pada cakupan 18 persen. Cakupan di dalam selnya dihapus, jadi penghitungnya benar-benar turun. Selnya digabung dan dibuatkan lajur ulang, lalu lajurnya diserahkan ke helper. Poin pentingnya, swarm tidak pernah berhenti: drone yang tersisa terus menyapu selama realokasi berlangsung.

> Arahan: 24 s clip. Let it run once and talk over it. The coverage dip is visible on the curve two slides later.

## Slide 41 — One mission ended in an obstacle strike, and we report it as such

*0.6 menit · 92 kata*

**EN** And this is the one mission that did not finish: the rectangle with PID-LQR, slowed down so you can see it. Agent 2 comes within 0.06 metres of a cylinder surface. That is inside the airframe itself: the body half-width is 0.235 metres and the rotors reach 0.348. It tilts to about 155.9 degrees, drops to 0.11 metres, and does not recover. We stopped the run at 78.8 seconds and report its coverage as n/a. It is the most useful result in the set, because it shows where the method runs out.

**ID** Dan inilah satu-satunya misi yang tidak selesai: area persegi dengan PID-LQR, diperlambat supaya terlihat. Agen 2 mendekat sampai 0.06 meter dari permukaan silinder. Itu sudah di dalam badan drone sendiri: setengah lebar badannya 0.235 meter dan jangkauan rotornya 0.348. Ia miring sampai sekitar 155.9 derajat, turun ke 0.11 meter, dan tidak pulih. Kami menghentikan run pada 78.8 detik dan melaporkan cakupannya sebagai n/a. Ini hasil paling berguna di antara semuanya, karena menunjukkan di mana batas metodenya.

> Arahan: Do not apologise for this slide. State it plainly — it is the most scientifically useful result in the set. The cause comes two slides later.

## Slide 42 — pembatas 8 · Results

## Slide 43 — H∞ finished every region; LQR finished three and struck a cylinder in the fourth

*1.0 menit · 103 kata*

**EN** Here are all eight missions. H-infinity finished every region. LQR finished three, and in the rectangle it struck a cylinder. Coverage in the finished runs is between 96.9 and 99.0 percent. In every region both completed, H-infinity was faster and held the lane more tightly, but used several times more torque. LQR is not uniformly worse, though. It has the best coverage of the campaign, 99.0 percent in the plus region, and the lowest peak tilt, 16.6 degrees in the U shape. During sweeping, tilt sits at the 15 degree cap in every run. The large maxima come from lane-end turns and upsets.

**ID** Ini kedelapan misi. H-infinity menyelesaikan semua area. LQR menyelesaikan tiga, dan di area persegi ia menabrak silinder. Cakupan pada run yang selesai antara 96.9 dan 99.0 persen. Di setiap area yang diselesaikan keduanya, H-infinity lebih cepat dan menjaga lajur lebih rapat, tetapi memakai torsi beberapa kali lebih besar. Meski begitu, LQR tidak selalu lebih buruk. LQR punya cakupan terbaik di seluruh kampanye, 99.0 persen di area plus, dan kemiringan puncak terendah, 16.6 derajat di area U. Selama menyapu, kemiringan berada di batas 15 derajat pada setiap run. Nilai maksimum yang besar berasal dari belokan di ujung lajur dan upset.

> Arahan: Ten seconds of silence so they can read it. Then name three things: the abort row, the 0.06 m clearance, and that Hinf finished every region.

## Slide 44 — 0.06 m

*0.2 menit · 32 kata*

**EN** This is the clearance where the failure happened: 0.06 metres. Before every upset, the safety filter was already straining against that cylinder. It saw it coming, but ran out of control authority.

**ID** Inilah jarak bebas saat kegagalan terjadi: 0.06 meter. Sebelum setiap upset, filter keselamatan sudah bekerja keras menahan silinder itu. Ia melihatnya datang, tetapi kehabisan otoritas kendali.

> Arahan: 12 s. This is the number a reviewer will remember; own it.

## Slide 45 — Coverage goes backwards at every loss — by design

*0.4 menit · 54 kata*

**EN** This plot shows coverage over time. Notice that it goes backwards at every loss. That is by design, not a bug: coverage inside a failed cell is erased and swept again. The cross marks the aborted LQR run at about 79 seconds. In every region it completed, H-infinity reached the 97 percent target sooner.

**ID** Grafik ini menunjukkan cakupan terhadap waktu. Perhatikan kurvanya turun setiap kali ada kehilangan. Itu disengaja, bukan bug: cakupan di dalam sel yang gagal dihapus lalu disapu ulang. Tanda silang menandai run LQR yang dihentikan sekitar detik ke-79. Di setiap area yang diselesaikannya, H-infinity mencapai target 97 persen lebih cepat.

> Arahan: The backwards step is a design choice, not a plotting artefact. Say so explicitly or someone will think it is a bug.

## Slide 46 — Most of the H∞ advantage is a rejected mean offset, not broadband attenuation

*1.0 menit · 106 kata*

**EN** Now, why is H-infinity tighter on the lane? The lateral RMS error drops from 11.4 to 12.2 centimetres for LQR, to 7.2 to 9.7 for H-infinity, a reduction of 16 to 39 percent. But look at the spread: the standard deviations are close, 9.4 to 10.8 against 7.0 to 9.6 centimetres. What really moved is the mean. LQR flies with a constant offset of six to seven centimetres to one side, while H-infinity stays within two centimetres of the line. So there are two real effects. H-infinity does attenuate disturbances better, and it also has a 3.9 times larger integral gain. Here, the second effect dominates.

**ID** Lalu, mengapa H-infinity lebih rapat di lajur? Galat lateral RMS turun dari 11.4 sampai 12.2 sentimeter untuk LQR, menjadi 7.2 sampai 9.7 untuk H-infinity, penurunan 16 sampai 39 persen. Tetapi lihat sebarannya: simpangan bakunya mirip, 9.4 sampai 10.8 dibanding 7.0 sampai 9.6 sentimeter. Yang benar-benar bergeser adalah rata-ratanya. LQR terbang dengan offset tetap enam sampai tujuh sentimeter ke satu sisi, sedangkan H-infinity tetap dalam dua sentimeter dari garis. Jadi ada dua efek yang nyata. H-infinity memang meredam gangguan lebih baik, dan ia juga punya gain integral 3.9 kali lebih besar. Di sini, efek kedua yang dominan.

> Arahan: The slide that separates a careful paper from a marketing one. Take the full 50 s. Median fell from 8.5–8.8 to 2.7–3.8 cm.

## Slide 47 — 55–64%

*0.3 menit · 44 kata*

**EN** The mechanism: the LQR integrator was open in only 55 to 64 percent of sweep samples, against 84 to 87 for H-infinity. Its gate is 25 centimetres, but the set-point leads by about 40. That is a tuning choice, not a property of LQR.

**ID** Mekanismenya: integrator LQR hanya terbuka pada 55 sampai 64 persen sampel sapuan, dibanding 84 sampai 87 untuk H-infinity. Gerbangnya 25 sentimeter, tetapi set-point mendahului sekitar 40 sentimeter. Itu pilihan tuning, bukan sifat LQR.

> Arahan: 18 s. The most interesting finding in the paper and the one that stops an unfair conclusion. Expect Q5 here.

## Slide 48 — Holding the lane tighter costs torque, and the trade is one scatter

*0.6 menit · 76 kata*

**EN** And the cost. Each point is one run, lateral error against torque effort. Every H-infinity run sits lower and further right: tighter lanes, more effort, 3.9 to 5.6 times more torque. Motor saturation stayed under 0.2 percent for both, so effort was spent, not exhausted. On a battery-limited drone, this ratio may matter more than the lateral error. So neither design wins outright: H-infinity is tighter and harder on the airframe, LQR is gentler and looser.

**ID** Dan harganya. Tiap titik adalah satu run, galat lateral terhadap upaya torsi. Setiap run H-infinity berada lebih rendah dan lebih ke kanan: lajur lebih rapat, upaya lebih besar, torsi 3.9 sampai 5.6 kali lebih besar. Saturasi motor tetap di bawah 0,2 persen untuk keduanya, jadi upayanya terpakai, bukan habis. Pada drone yang dibatasi baterai, rasio ini bisa lebih menentukan daripada galat lateral. Jadi tidak ada desain yang menang mutlak: H-infinity lebih rapat dan lebih berat bagi badan drone, LQR lebih lembut dan lebih longgar.

> Arahan: 35 s. Point at the gap between the blue and orange clusters on the x axis – that is the whole cost argument in one picture.

## Slide 49 — No pair went below 0.70 m — which is not the same as no collisions

*0.5 menit · 69 kata*

**EN** On safety between drones: the closest pair in the whole campaign came to 0.79 metres, against a hard limit of 0.70. But we are careful about what that means. Three missions passed within rotor reach of an obstacle without contact being provable. Rotor guards are an assumption, not simulated, and contact between drones is never detected; only the minimum distance is tracked. So we do not claim zero collisions.

**ID** Soal keselamatan antar-drone: pasangan terdekat di seluruh kampanye berjarak 0.79 meter, dibanding batas keras 0,70. Tetapi kami berhati-hati soal artinya. Tiga misi lewat dalam jangkauan rotor dari rintangan tanpa bisa dibuktikan ada kontak atau tidak. Pelindung rotor adalah asumsi, tidak disimulasikan, dan kontak antar-drone tidak pernah dideteksi; hanya jarak minimum yang dicatat. Jadi kami tidak mengklaim nol tabrakan.

> Arahan: All eight runs logged a negative moving-obstacle barrier, to -0.68 m — Q7 covers it. Q6 and Q7 live here. Saying it first is cheaper than defending it later. Torque effort 3.9–5.6 times higher for Hinf; motor saturation under 0.2% of samples for both.

## Slide 50 — What this work does not establish

*0.3 menit · 46 kata*

**EN** To be clear about what this work does not establish: these are single runs with one wind seed. State and obstacle feedback are perfect, failure detection is instant, and coverage is geometric. The upper layers are centralised, and there is no ablation or re-implemented baseline yet.

**ID** Supaya jelas apa yang belum dibuktikan penelitian ini: ini run tunggal dengan satu seed angin. Umpan balik state dan rintangan sempurna, deteksi kegagalan instan, dan cakupannya geometris. Lapis atasnya tersentralisasi, dan belum ada ablasi atau baseline yang diimplementasikan ulang.

> Arahan: Read quickly and without hedging. A short slide that earns trust. Runs are stopped at unscheduled crashes rather than continued.

## Slide 51 — Seven of eight, under every stressor at once

*0.4 menit · 55 kata*

**EN** To conclude. The swarm reached 96.9 to 99.0 percent coverage while losing two of seven quadrotors. Where both finished, H-infinity held lanes 16 to 39 percent tighter and finished 11 to 15 percent sooner, at several times the torque effort. Next, we plan several wind seeds, a gust-aware barrier, distributed upper layers, and flight tests.

**ID** Sebagai kesimpulan. Swarm mencapai cakupan 96.9 sampai 99.0 persen meskipun kehilangan dua dari tujuh quadrotor. Di area yang diselesaikan keduanya, H-infinity menjaga lajur 16 sampai 39 persen lebih rapat dan selesai 11 sampai 15 persen lebih cepat, dengan upaya torsi beberapa kali lebih besar. Selanjutnya, kami merencanakan beberapa seed angin, barrier yang memperhitungkan hembusan, lapis atas yang terdistribusi, dan uji terbang.

> Arahan: Land on the future work, then go straight to the thank-you slide.

## Slide 52 — Thank you

*0.3 menit · 45 kata*

**EN** Thank you for your attention. The code is open source under the MIT licence, and all eight mission recordings are public; the link and QR code are on this slide. We thank CITA ITB for supporting this work. I am happy to take your questions.

**ID** Terima kasih atas perhatiannya. Kodenya open source dengan lisensi MIT, dan kedelapan rekaman misi bisa diakses publik; tautan dan kode QR ada di slide ini. Kami berterima kasih kepada CITA ITB yang telah mendukung penelitian ini. Saya dengan senang hati menerima pertanyaan.

> Arahan: End here. Do NOT advance – the backup section behind this slide holds the anticipated-question pack; jump to it only if a question lands.
