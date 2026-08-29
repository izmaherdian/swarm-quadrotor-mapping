"""Penghindaran tabrakan berbasis Control Barrier Function (CBF-QP).

Satu QP per drone per tick menggabungkan rintangan statis, rintangan bergerak,
dan jarak antar-drone resiprokal sebagai constraint linear pada kecepatan yang
dikomandokan — menggantikan tumpukan lapisan if/else hand-tuned.

Kunci desainnya: fungsi kelas-K phi(h) diturunkan dari plant loop-tertutup yang
teridentifikasi (lihat plant_model.py), bukan disetel manual. Dengan begitu
himpunan aman benar-benar invariant di bawah batas kemiringan 15 derajat yang
nyata — persoalan yang membuat pendekatan lama gagal.
"""
from .avoidance import CBFAvoidance, priority_for_state
from .barrier import phi, phi_inverse, phi_zero_h
from .plant_model import PlantModel
from .qp2d import solve_projection
from .types import (
    AgentState,
    AvoidanceResult,
    Bounds,
    CBFConfig,
    Obstacle,
    Task,
    CLASS_DYNAMIC,
    CLASS_STATIC,
)

__all__ = [
    'CBFAvoidance', 'priority_for_state',
    'PlantModel', 'phi', 'phi_inverse', 'phi_zero_h', 'solve_projection',
    'AgentState', 'Obstacle', 'Task', 'Bounds', 'CBFConfig', 'AvoidanceResult',
    'CLASS_STATIC', 'CLASS_DYNAMIC',
]
