import os
import sys

# Agar `import swarm_high_level...` bekerja saat pytest dijalankan dari src/.
sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
