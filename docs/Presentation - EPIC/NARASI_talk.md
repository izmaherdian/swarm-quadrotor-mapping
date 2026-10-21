# Narasi presentasi EPIC 2026

Dihasilkan `tools/export_narasi.py` dari `\narasi{}` di `talk.tex` — **jangan diedit di sini**, edit `talk.tex` lalu `make`.
Angka diambil dari `numbers.tex`, file yang sama dengan paper.

Slide 1–16 · 16 slide bicara · 10.0 menit · 940 kata Inggris (≈ 94 kata/menit).


## Slide 1 — Title

*0.3 menit · 37 kata*

**EN** Hello everyone. I am Izma Alhazmi Herdian from Institut Teknologi Bandung. Today I present fault-tolerant swarm quadrotor control for mapping non-convex areas. On the right is the whole system: three control layers on top of the swarm.

**ID** Halo semuanya. Saya Izma Alhazmi Herdian dari Institut Teknologi Bandung. Hari ini saya mempresentasikan kendali swarm quadrotor yang toleran kegagalan untuk memetakan area non-konveks. Di sebelah kanan adalah keseluruhan sistemnya: tiga lapis kendali di atas swarm.

> Arahan: Smile, speak slowly. Point at the diagram on the last sentence.

## Slide 2 — Seven quadrotors, four regions, every stressor at once

*0.5 menit · 51 kata*

**EN** This is what the system does. Seven quadrotors split a region into cells and sweep them lane by lane, in four differently shaped regions. They fly through strong wind, avoid cylinders they discover with LiDAR, and in every mission two drones are switched off mid-flight. The survivors take over their work.

**ID** Inilah yang dilakukan sistemnya. Tujuh quadrotor membagi area menjadi sel dan menyapunya lajur demi lajur, di empat area dengan bentuk berbeda. Mereka terbang menembus angin kencang, menghindari silinder yang ditemukan dengan LiDAR, dan di setiap misi dua drone dimatikan saat terbang. Drone yang tersisa mengambil alih pekerjaannya.

> Arahan: Click the frame to play. Move on after about 25 s, even if the clip is still running.

## Slide 3 — The question is not whether a swarm is faster, but whether it still finishes

*0.6 menit · 65 kata*

**EN** Mapping an area is a coverage problem: every point has to pass under a sensor. A swarm can cover it faster, but only if it copes with what goes wrong. We combine four difficulties at once: a non-convex region with notches and arms, wind averaging 3.0 metres per second, obstacles that are not known in advance, and the loss of two of the seven drones.

**ID** Memetakan sebuah area adalah masalah cakupan: setiap titik harus terlewati sensor. Swarm bisa mencakupnya lebih cepat, tetapi hanya jika mampu menghadapi hal yang tidak berjalan sesuai rencana. Kami menggabungkan empat kesulitan sekaligus: area non-konveks dengan lekukan dan lengan, angin rata-rata 3.0 meter per detik, rintangan yang tidak diketahui sebelumnya, dan hilangnya dua dari tujuh drone.

> Arahan: Read the four bullets as you name them.

## Slide 4 — Four contributions

*0.4 menit · 35 kata*

**EN** We contribute four parts: lanes that follow each cell's shape, lane reallocation when a drone is lost, a safety filter based on the drone's physical limits, and a fair comparison of PID-LQR and PID H-infinity.

**ID** Kontribusi kami ada empat: lajur yang mengikuti bentuk tiap sel, realokasi lajur saat drone hilang, filter keselamatan yang berdasar pada batas fisik drone, dan perbandingan adil antara PID-LQR dan PID H-infinity.

> Arahan: One breath per bullet.

## Slide 5 — Three layers, each one handing a target to the next

*0.8 menit · 77 kata*

**EN** The system has three layers, and each one hands a target to the next. The high level is event-driven: it takes the region and the drone positions, and gives each drone a lane to follow. The mid level runs at 20 hertz: it takes the desired velocity, the obstacle map from LiDAR and the neighbouring drones, and returns a safe velocity. The low level runs at 250 hertz and turns a position target into four rotor speeds.

**ID** Sistemnya punya tiga lapis, dan masing-masing menyerahkan target ke lapis berikutnya. Lapis atas berbasis kejadian: masukannya area dan posisi drone, dan keluarannya satu lajur untuk diikuti tiap drone. Lapis tengah berjalan pada 20 hertz: masukannya kecepatan yang diinginkan, peta rintangan dari LiDAR, dan drone tetangga, lalu mengembalikan kecepatan yang aman. Lapis bawah berjalan pada 250 hertz dan mengubah target posisi menjadi empat kecepatan rotor.

> Arahan: Read the three columns left to right.

## Slide 6 — Low level: one tilt limit, one structure, two controllers

*0.8 menit · 75 kata*

**EN** At the low level, a quadrotor can only move sideways by tilting its thrust. With the tilt limited to 15 degrees for stable mapping, its acceleration is capped at 2.631 metres per second squared. We design two controllers on the same model: PID-LQR, which is optimal for the model, and PID H-infinity, which is designed against the worst-case disturbance. Both end up as an ordinary PID with the same structure, so only the gains differ.

**ID** Di lapis bawah, quadrotor hanya bisa bergerak ke samping dengan memiringkan thrust-nya. Dengan kemiringan dibatasi 15 derajat agar pemetaan stabil, percepatannya terbatas pada 2.631 meter per detik kuadrat. Kami merancang dua pengendali pada model yang sama: PID-LQR, yang optimal untuk modelnya, dan PID H-infinity, yang dirancang terhadap gangguan terburuk. Keduanya berakhir sebagai PID biasa dengan struktur yang sama, jadi yang berbeda hanya gain-nya.

> Arahan: Point at the tilted thrust in the figure on the first sentence.

## Slide 7 — Mid level: a speed limit the drone can actually obey

*0.9 menit · 89 kata*

**EN** The mid level keeps the drones safe. From the tilt limit and the drone's reaction delay of 0.4 seconds, we compute how fast a drone may approach an obstacle at a given distance and still stop in time. That is the curve on the right. Every obstacle, wall or neighbouring drone then becomes one limit on approach speed, and a small optimisation picks the safe velocity closest to what the planner asked. A simple linear rule, shown by the pink band, would allow approaches the drone cannot stop from.

**ID** Lapis tengah menjaga drone tetap aman. Dari batas kemiringan dan jeda reaksi drone 0,4 detik, kami menghitung seberapa cepat drone boleh mendekati rintangan pada jarak tertentu dan masih bisa berhenti tepat waktu. Itulah kurva di sebelah kanan. Setiap rintangan, dinding, atau drone tetangga lalu menjadi satu batas kecepatan mendekat, dan optimasi kecil memilih kecepatan aman yang paling dekat dengan permintaan perencana. Aturan linier sederhana, ditunjukkan pita merah muda, masih mengizinkan drone mendekat padahal sudah tidak bisa berhenti.

> Arahan: Point at the curve, then at the pink band on the last sentence.

## Slide 8 — High level: cells, lanes, and re-planning when an agent is lost

*0.8 menit · 74 kata*

**EN** The high level plans the coverage. The region is divided into Voronoi cells, one per drone, and each cell is swept in parallel lanes spaced to the sensor footprint, with detours around known obstacles. When a drone is lost, its cell is merged with its neighbours, the remaining lanes are re-planned, and they are handed to nearby drones in contiguous blocks. The figure shows the plan before, and after each of the two losses.

**ID** Lapis atas merencanakan cakupan. Area dibagi menjadi sel Voronoi, satu per drone, dan tiap sel disapu dengan lajur sejajar yang jaraknya disesuaikan dengan jangkauan sensor, dengan jalan memutar di sekitar rintangan yang diketahui. Saat drone hilang, selnya digabung dengan tetangganya, lajur yang tersisa direncanakan ulang, lalu diserahkan ke drone terdekat dalam blok yang bersambung. Gambar ini menunjukkan rencana sebelum, dan sesudah masing-masing dari dua kehilangan.

> Arahan: Walk panels a, b, c left to right.

## Slide 9 — Every stressor applied at once, to seven quadrotors

*0.5 menit · 61 kata*

**EN** We tested this in Gazebo with ROS 2, using seven 3DR Iris quadrotors sweeping at 1.6 metres per second. All stressors are applied at once: turbulent wind, static cylinders the swarm does not know in advance, two moving cylinders, and two drones switched off at 18 and 32 percent coverage. We ran four regions with both controllers, eight missions in total.

**ID** Kami mengujinya di Gazebo dengan ROS 2, memakai tujuh quadrotor 3DR Iris yang menyapu pada 1,6 meter per detik. Semua gangguan diterapkan sekaligus: angin turbulen, silinder diam yang tidak diketahui swarm sebelumnya, dua silinder bergerak, dan dua drone dimatikan pada cakupan 18 dan 32 persen. Kami menjalankan empat area dengan kedua pengendali, total delapan misi.

> Arahan: Keep it brisk; the videos come next.

## Slide 10 — A full mission: partition, transit, sweep, two losses, return

*0.8 menit · 68 kata*

**EN** Here is a full mission in the rectangular region with PID H-infinity. The region is divided, the drones fly to their first lanes, and start sweeping. The green area on the right is the coverage map building up. Drone 4 is switched off, and its lanes are taken over; later drone 7 is switched off as well. The survivors still finish, with 98.2 percent coverage in 257 seconds.

**ID** Ini satu misi penuh di area persegi dengan PID H-infinity. Area dibagi, drone terbang ke lajur pertamanya, lalu mulai menyapu. Area hijau di sebelah kanan adalah peta cakupan yang terus bertambah. Drone 4 dimatikan, dan lajurnya diambil alih; kemudian drone 7 juga dimatikan. Drone yang tersisa tetap menyelesaikan misi, dengan cakupan 98.2 persen dalam 257 detik.

> Arahan: Click to play. Stop the clip after about 40 s, when your sentence ends.

## Slide 11 — The swarm never pauses — survivors keep sweeping through a reallocation

*0.4 menit · 36 kata*

**EN** This is a closer look at the first loss. Coverage inside the lost cell is reset, the cell is merged and re-planned, and the helpers take over. The rest of the swarm keeps sweeping without pausing.

**ID** Lebih dekat pada kehilangan pertama. Cakupan di dalam sel yang hilang direset, sel itu digabung dan direncanakan ulang, lalu para helper mengambil alih. Bagian swarm lainnya terus menyapu tanpa berhenti.

> Arahan: Click to play; talk over the clip.

## Slide 12 — Seven of eight missions finished; PID-H∞ finished every region

*0.9 menit · 78 kata*

**EN** Here are all eight missions. Seven finished, with coverage between 96.9 and 99.0 percent. H-infinity completed all four regions. LQR completed three; in the rectangle, one LQR drone hit a cylinder and the run was stopped, so we report it as a failure rather than leave it out. Where both finished, H-infinity was 11 to 15 percent faster and held the lanes more tightly, while LQR used less control effort and achieved the single best coverage, 99.0 percent.

**ID** Inilah kedelapan misi. Tujuh selesai, dengan cakupan antara 96.9 dan 99.0 persen. H-infinity menyelesaikan keempat area. LQR menyelesaikan tiga; di area persegi, satu drone LQR menabrak silinder dan run dihentikan, jadi kami melaporkannya sebagai kegagalan, bukan menghilangkannya. Di area yang diselesaikan keduanya, H-infinity 11 sampai 15 persen lebih cepat dan menjaga lajur lebih rapat, sedangkan LQR memakai upaya kendali lebih kecil dan mencatat cakupan tunggal terbaik, 99.0 persen.

> Arahan: Pause two seconds so they can read the table, then start.

## Slide 13 — Coverage drops at every loss, then recovers

*0.5 menit · 46 kata*

**EN** This plot shows coverage over time. Each drop happens when a drone is lost: coverage inside its cell is reset and swept again by the helpers. After each drop the swarm recovers, and in every region both controllers completed, H-infinity reached the 97 percent target sooner.

**ID** Grafik ini menunjukkan cakupan terhadap waktu. Setiap penurunan terjadi saat drone hilang: cakupan di dalam selnya direset lalu disapu ulang oleh para helper. Setelah tiap penurunan swarm pulih, dan di setiap area yang diselesaikan kedua pengendali, H-infinity mencapai target 97 persen lebih cepat.

> Arahan: Point at one of the drops on the first sentence.

## Slide 14 — PID-H∞ holds the lane tighter, at a torque cost

*0.9 menit · 76 kata*

**EN** Why is H-infinity tighter? Its lateral error is 16 to 39 percent lower. Most of that comes from removing a steady offset, because its integral action is about 3.9 times stronger, rather than from filtering out the wind. The price is effort: H-infinity uses 3.9 to 5.6 times more torque, although motor saturation stays below 0.2 percent. So there is a trade-off: H-infinity is tighter, LQR is gentler, and the right choice depends on the platform.

**ID** Mengapa H-infinity lebih rapat? Galat lateralnya 16 sampai 39 persen lebih kecil. Sebagian besar berasal dari hilangnya offset yang tetap, karena aksi integralnya sekitar 3.9 kali lebih kuat, bukan dari penyaringan angin. Harganya adalah upaya: H-infinity memakai torsi 3.9 sampai 5.6 kali lebih besar, meski saturasi motor tetap di bawah 0,2 persen. Jadi ada trade-off: H-infinity lebih rapat, LQR lebih lembut, dan pilihan yang tepat bergantung pada platformnya.

> Arahan: Point at the gap between the blue and orange clusters.

## Slide 15 — Seven of eight, under every stressor at once

*0.6 menit · 45 kata*

**EN** To conclude: a seven-drone swarm mapped non-convex regions with 96.9 to 99.0 percent coverage while losing two drones, and seven of eight missions finished. These are simulation results with one run per configuration. Next, we plan more wind conditions, distributed planning, and real flight tests.

**ID** Sebagai kesimpulan: swarm tujuh drone memetakan area non-konveks dengan cakupan 96.9 sampai 99.0 persen meski kehilangan dua drone, dan tujuh dari delapan misi selesai. Ini hasil simulasi dengan satu run per konfigurasi. Selanjutnya, kami merencanakan lebih banyak kondisi angin, perencanaan terdistribusi, dan uji terbang nyata.

> Arahan: Slow down here; this is what they remember.

## Slide 16 — Thank you

*0.3 menit · 27 kata*

**EN** Thank you for your attention. The code and all mission recordings are available at the link on this slide. We thank CITA ITB for supporting this work.

**ID** Terima kasih atas perhatiannya. Kode dan semua rekaman misi tersedia di tautan pada slide ini. Kami berterima kasih kepada CITA ITB yang telah mendukung penelitian ini.

> Arahan: Hold this slide for two seconds before stopping the recording.
