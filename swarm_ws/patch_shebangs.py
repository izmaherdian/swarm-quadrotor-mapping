import os

paths = [
    "install/swarm_low_level/lib/swarm_low_level/pid_lqr_node",
    "install/swarm_low_level/lib/swarm_low_level/pid_hinf_node",
    "install/swarm_mid_level/lib/swarm_mid_level/collision_avoidance_node"
]

for p in paths:
    if os.path.exists(p):
        with open(p, 'r') as f:
            lines = f.readlines()
        if lines[0].startswith("#!"):
            lines[0] = "#!/usr/bin/env python3\n"
            with open(p, 'w') as f:
                f.writelines(lines)
            print(f"Patched shebang for {p}")
        else:
            print(f"No shebang found for {p}")
    else:
        print(f"File not found: {p}")
