# Narasi presentasi EPIC 2026

Dihasilkan `tools/export_narasi.py` dari `\narasi{}` di `talk.tex` — **jangan diedit di sini**, edit `talk.tex` lalu `make`.
Angka diambil dari `numbers.tex`, file yang sama dengan paper.

Slide 1–16 · 16 slide bicara · 10.0 menit · 923 kata Inggris (≈ 92 kata/menit).


## Slide 1 — Title

*0.3 menit · 38 kata*

**EN** Hi everyone. I'm Izma from Institut Teknologi Bandung. Today I'll show you how a team of drones can map an area together, even when some of them fail. On the right is the whole system in one picture.

**ID** Halo semuanya. Saya Izma dari Institut Teknologi Bandung. Hari ini saya akan menunjukkan bagaimana sekelompok drone bisa memetakan area bersama-sama, bahkan saat beberapa di antaranya gagal. Di sebelah kanan adalah seluruh sistemnya dalam satu gambar.

> Arahan: Smile, speak slowly. Point at the diagram on the last sentence.

## Slide 2 — Seven quadrotors, four regions, every stressor at once

*0.5 menit · 58 kata*

**EN** Let's start with a video. Here, seven drones share one area. Each drone gets its own part and flies back and forth to cover it. There is strong wind, and there are obstacles they only see when they get close. In the middle of each mission, we turn off two drones, and the other drones finish their work.

**ID** Kita mulai dengan video. Di sini, tujuh drone berbagi satu area. Tiap drone mendapat bagiannya sendiri dan terbang bolak-balik untuk menyapunya. Anginnya kencang, dan ada rintangan yang baru terlihat saat drone sudah dekat. Di tengah setiap misi, kami mematikan dua drone, dan drone lainnya menyelesaikan pekerjaan mereka.

> Arahan: Click the frame to play. Move on after about 25 s, even if the clip is still running.

## Slide 3 — The question is not whether a swarm is faster, but whether it still finishes

*0.6 menit · 70 kata*

**EN** So what's the problem? To map an area, every spot has to pass under a sensor. More drones make it faster, but things can go wrong. In our test, four things go wrong at the same time. The area is non-convex. The wind is strong, about 3.0 metres per second, with gusts up to 12.0. The obstacles are not known in advance. And two of the seven drones stop working.

**ID** Jadi apa masalahnya? Untuk memetakan area, setiap titik harus terlewati sensor. Lebih banyak drone membuatnya lebih cepat, tetapi banyak hal bisa salah. Dalam pengujian kami, empat hal salah sekaligus. Areanya non-konveks. Anginnya kencang, sekitar 3.0 meter per detik, dengan hembusan sampai 12.0. Rintangannya tidak diketahui sebelumnya. Dan dua dari tujuh drone berhenti bekerja.

> Arahan: Read the four bullets as you name them.

## Slide 4 — Four contributions

*0.4 menit · 39 kata*

**EN** We bring four ideas. Lanes that follow the shape of each cell. Reallocating lanes when a drone is lost. A safety filter based on the real actuator limits. And a fair comparison of two controllers, PID-LQR and PID H-infinity.

**ID** Kami membawa empat ide. Lajur yang mengikuti bentuk tiap sel. Realokasi lajur saat sebuah drone hilang. Filter keselamatan yang berdasar pada batas aktuator sebenarnya. Dan perbandingan yang adil antara dua pengendali, PID-LQR dan PID H-infinity.

> Arahan: One breath per bullet.

## Slide 5 — Three layers, each one handing a target to the next

*0.8 menit · 78 kata*

**EN** The system has three layers, and each one passes a setpoint to the next. The high level is the planner: it partitions the area and gives each drone a lane. The mid level is a safety filter at 20 hertz: it takes the desired velocity and corrects it if the drone is approaching an obstacle or another drone. The low level is the PID flight controller at 250 hertz: it turns a position setpoint into four rotor speeds.

**ID** Sistemnya punya tiga lapis, dan tiap lapis memberikan setpoint ke lapis berikutnya. Lapis atas adalah perencana: ia mempartisi area dan memberi tiap drone satu lajur. Lapis tengah adalah filter keselamatan pada 20 hertz: ia menerima kecepatan yang diinginkan dan mengoreksinya kalau drone sedang mendekati rintangan atau drone lain. Lapis bawah adalah pengendali terbang PID pada 250 hertz: ia mengubah setpoint posisi menjadi empat kecepatan rotor.

> Arahan: Read the three columns left to right.

## Slide 6 — Low level: one tilt limit, one structure, two controllers

*0.8 menit · 71 kata*

**EN** At the low level, a quadrotor can only move sideways by tilting. We limit the tilt to 15 degrees, so the maximum horizontal acceleration is 2.631 metres per second squared. We design two controllers on the same linear model. LQR minimises a quadratic cost, and H-infinity minimises the effect of the worst-case disturbance. Both are then turned into a standard PID with the same structure, so only the gains are different.

**ID** Di lapis bawah, quadrotor hanya bisa bergerak ke samping dengan cara miring. Kami membatasi kemiringan 15 derajat, sehingga percepatan horizontal maksimumnya 2.631 meter per detik kuadrat. Kami merancang dua pengendali pada model linier yang sama. LQR meminimalkan biaya kuadratik, dan H-infinity meminimalkan efek gangguan terburuk. Keduanya lalu diubah menjadi PID standar dengan struktur yang sama, jadi yang berbeda hanya gain-nya.

> Arahan: Point at the tilted thrust in the figure on the first sentence.

## Slide 7 — Mid level: a speed limit the drone can actually obey

*0.9 menit · 96 kata*

**EN** Next, the safety layer. The drone has a limited braking acceleration and a reaction delay of about 0.4 seconds. From these, we compute a barrier function: for each distance, the maximum approach speed that still lets the drone stop in time. That's the blue curve. Each obstacle, wall or neighbour becomes one linear constraint on the approach speed, and a small QP finds the safe velocity closest to the planned one. A linear barrier, the pink area, would allow approach where the drone can no longer stop. Ours doesn't, and it has no parameter to tune.

**ID** Berikutnya, lapis keselamatan. Drone punya percepatan pengereman yang terbatas dan jeda reaksi sekitar 0,4 detik. Dari keduanya, kami menghitung barrier function: untuk setiap jarak, kecepatan mendekat maksimum yang masih memungkinkan drone berhenti tepat waktu. Itulah kurva biru. Setiap rintangan, dinding, atau tetangga menjadi satu kendala linier pada kecepatan mendekat, dan QP kecil mencari kecepatan aman yang paling dekat dengan kecepatan yang direncanakan. Barrier linier, area merah muda, masih mengizinkan drone mendekat padahal sudah tidak bisa berhenti. Barrier kami tidak, dan tidak punya parameter yang perlu disetel.

> Arahan: Point at the curve, then at the pink band on the last sentence.

## Slide 8 — High level: cells, lanes, and re-planning when an agent is lost

*0.8 menit · 71 kata*

**EN** The high level plans the coverage. It partitions the area into Voronoi cells, one per drone, and covers each cell with back-and-forth lanes, called boustrophedon lanes. The lanes go around known obstacles. When a drone is lost, its cell is merged with the neighbouring cells, the remaining lanes are re-planned, and nearby drones take them over. The figure shows the plan at the start, and after each of the two losses.

**ID** Lapis atas merencanakan cakupan. Ia mempartisi area menjadi sel Voronoi, satu per drone, dan menyapu tiap sel dengan lajur bolak-balik yang disebut lajur boustrophedon. Lajurnya memutari rintangan yang sudah diketahui. Saat sebuah drone hilang, selnya digabung dengan sel tetangga, lajur yang tersisa direncanakan ulang, lalu drone terdekat mengambil alih. Gambar ini menunjukkan rencana di awal, dan setelah masing-masing dari dua kehilangan.

> Arahan: Walk panels a, b, c left to right.

## Slide 9 — Every stressor applied at once, to seven quadrotors

*0.5 menit · 45 kata*

**EN** We tested everything in a simulator called Gazebo, with seven drones flying at 1.6 metres per second. Everything happens at once: wind, hidden obstacles, two moving obstacles, and two drones turned off. We tried four area shapes with both controllers, so eight missions in total.

**ID** Kami menguji semuanya di simulator bernama Gazebo, dengan tujuh drone yang terbang 1,6 meter per detik. Semuanya terjadi sekaligus: angin, rintangan tersembunyi, dua rintangan bergerak, dan dua drone dimatikan. Kami mencoba empat bentuk area dengan kedua pengendali, jadi totalnya delapan misi.

> Arahan: Keep it brisk; the videos come next.

## Slide 10 — A full mission: partition, transit, sweep, two losses, return

*0.8 menit · 65 kata*

**EN** Here is one full mission. First the area is split, then the drones fly to their lanes and start covering. The green on the right shows the area that's already mapped. Now drone 4 is turned off, and the others take over its lanes. Later, drone 7 is turned off too. The rest still finish, with 98.2 percent of the area covered in 257 seconds.

**ID** Ini satu misi penuh. Pertama areanya dibagi, lalu drone terbang ke lajurnya dan mulai menyapu. Warna hijau di sebelah kanan menunjukkan area yang sudah terpetakan. Sekarang drone 4 dimatikan, dan drone lain mengambil alih lajurnya. Kemudian drone 7 juga dimatikan. Sisanya tetap selesai, dengan 98.2 persen area tercakup dalam 257 detik.

> Arahan: Click to play. Stop the clip after about 40 s, when your sentence ends.

## Slide 11 — The swarm never pauses — survivors keep sweeping through a reallocation

*0.4 menit · 33 kata*

**EN** Here's a closer look at the first loss. The lost drone's area is reset, planned again, and shared with the helpers. The other drones keep working the whole time. Nobody stops and waits.

**ID** Ini tampilan lebih dekat dari kehilangan pertama. Area milik drone yang hilang direset, direncanakan ulang, lalu dibagikan ke para helper. Drone lainnya terus bekerja sepanjang waktu. Tidak ada yang berhenti dan menunggu.

> Arahan: Click to play; talk over the clip.

## Slide 12 — Seven of eight missions finished; PID-H∞ finished every region

*0.9 menit · 80 kata*

**EN** Here are all eight missions. Seven of them finished, covering 96.9 to 99.0 percent of the area. H-infinity finished all four. LQR finished three. In one case, an LQR drone hit an obstacle, and we stopped the run. We count it as a failure, and we show it instead of hiding it. Where both finished, H-infinity was 11 to 15 percent faster and stayed closer to its lanes. LQR used less effort and got the best single coverage, 99.0 percent.

**ID** Ini kedelapan misi. Tujuh di antaranya selesai, mencakup 96.9 sampai 99.0 persen area. H-infinity menyelesaikan keempatnya. LQR menyelesaikan tiga. Dalam satu kasus, drone LQR menabrak rintangan, dan kami menghentikan run-nya. Kami menghitungnya sebagai kegagalan, dan kami menunjukkannya, bukan menyembunyikannya. Di area yang diselesaikan keduanya, H-infinity 11 sampai 15 persen lebih cepat dan lebih dekat ke lajurnya. LQR memakai tenaga lebih sedikit dan mencatat cakupan tunggal terbaik, 99.0 persen.

> Arahan: Pause two seconds so they can read the table, then start.

## Slide 13 — Coverage drops at every loss, then recovers

*0.5 menit · 38 kata*

**EN** This plot shows how much area is covered over time. Each dip is a lost drone: its area is reset and mapped again. After every dip, the swarm recovers. And wherever both finished, H-infinity reached 97 percent first.

**ID** Grafik ini menunjukkan berapa banyak area yang tercakup seiring waktu. Setiap penurunan adalah drone yang hilang: areanya direset dan dipetakan ulang. Setelah setiap penurunan, swarm pulih. Dan di semua area yang diselesaikan keduanya, H-infinity lebih dulu mencapai 97 persen.

> Arahan: Point at one of the drops on the first sentence.

## Slide 14 — PID-H∞ holds the lane tighter, at a torque cost

*0.9 menit · 64 kata*

**EN** So why is H-infinity closer to its lanes? Its lateral error is 16 to 39 percent smaller. But this is mostly not better disturbance rejection. It mostly removes a steady-state offset, because its integral gain is about 3.9 times larger. The cost is 3.9 to 5.6 times more control effort. So there's no clear winner: H-infinity is more precise, and LQR uses less effort.

**ID** Jadi kenapa H-infinity lebih dekat ke lajurnya? Galat lateralnya 16 sampai 39 persen lebih kecil. Tapi sebagian besar bukan karena penolakan gangguan yang lebih baik. Sebagian besar karena ia menghilangkan offset steady-state, sebab gain integralnya sekitar 3.9 kali lebih besar. Harganya, control effort-nya 3.9 sampai 5.6 kali lebih besar. Jadi tidak ada pemenang yang jelas: H-infinity lebih presisi, dan LQR memakai effort lebih kecil.

> Arahan: Point at the gap between the blue and orange clusters.

## Slide 15 — Seven of eight, under every stressor at once

*0.6 menit · 48 kata*

**EN** To sum up. Seven drones mapped non-convex areas, covering 96.9 to 99.0 percent, even after losing two drones. Seven out of eight missions finished. These are simulation results, with one run each. Next, we want to test more wind conditions, make the planning distributed, and fly real drones.

**ID** Sebagai ringkasan. Tujuh drone memetakan area non-konveks, mencakup 96.9 sampai 99.0 persen, bahkan setelah kehilangan dua drone. Tujuh dari delapan misi selesai. Ini hasil simulasi, dengan satu run untuk masing-masing. Selanjutnya, kami ingin menguji lebih banyak kondisi angin, membuat perencanaannya terdistribusi, dan menerbangkan drone sungguhan.

> Arahan: Slow down here; this is what they remember.

## Slide 16 — Thank you

*0.3 menit · 29 kata*

**EN** Thank you for watching. The code and all the videos are free to see at the link on this slide. Thanks also to CITA ITB for supporting this work.

**ID** Terima kasih sudah menonton. Kode dan semua videonya bisa dilihat gratis di tautan pada slide ini. Terima kasih juga kepada CITA ITB yang telah mendukung penelitian ini.

> Arahan: Hold this slide for two seconds before stopping the recording.
