# Contekan Q&A — EPIC 2026, ABS-24

Jawaban singkat, tidak terlalu teknis, siap diucapkan.
Kalau tidak tahu: *"That's a good question. We haven't tested that yet, but it's part of our future work."*

---

## Paling mungkin ditanya

**1. Why did the LQR mission crash? Is the comparison fair?**
Yes, it's fair: both controllers use the same structure and the same safety layer, only the tuning is different.
The LQR drone drifted slightly to one side, came too close to an obstacle, and couldn't recover. We report it as a failure instead of hiding it.

**2. Is H-infinity better than LQR?**
It's more precise and finishes a bit faster, but it works the motors several times harder.
So it depends on the drone: if precision matters, H-infinity; if energy matters, LQR.

**3. Only one run per test — is that enough?**
No, it's not enough for statistics, and we say that clearly.
It shows the behaviour and the main trend. More runs with different wind are our next step.

**4. Have you tested this on real drones?**
Not yet, this is simulation only.
Real flight tests are our next step.

**5. How do the drones know one has failed?**
In this work, we assume the failure is detected immediately.
Detecting it from real sensor data is future work.

---

## Metode

**6. Why did you use this safety method instead of ORCA?** *(abstrak menyebut ORCA)*
ORCA assumes the drone can change speed instantly, but a real quadrotor needs time to tilt and brake.
Our method uses the drone's real braking limit, so the safe speed it gives is one the drone can actually follow.

**7. Is it fast enough to run in real time?**
Yes, the safety calculation takes less than a millisecond per drone, so it's very light.

**8. Does it work with more than seven drones?**
The safety layer can scale, but the planner currently runs in one central computer.
Making the planner distributed is part of our future work.

**9. How do the drones find obstacles?**
Each drone has a LiDAR. When it sees an obstacle a few times, the obstacle is added to the shared map, and the remaining lanes are re-planned around it.

**10. Why not use a GPS or a real state estimator?**
In this simulation we used the true position, so the controller comparison stays clean.
Adding a real estimator is planned for the flight tests.

---

## Hasil

**11. Why is coverage not 100 percent?**
Even a perfect flight of our planned lanes doesn't reach 100 percent because of the grid and the lane spacing.
Our results, about 97 to 99 percent, are very close to that limit.

**12. Did the drones collide with each other?**
No pair came closer than our safety limit. But we only measure the distance, not real contact, so we don't claim zero collisions.

**13. What happens to coverage when a drone is lost?**
The coverage of its area is reset and mapped again by the other drones.
That's why the coverage curve drops a little, then catches up.

---

## Umum

**14. What is the real-world application?**
Mapping irregular areas like farmland, mines, or disaster zones, where one drone is too slow and drones can fail.

**15. Why seven drones?**
Seven matched the size of our test area. It's not a hard limit.
