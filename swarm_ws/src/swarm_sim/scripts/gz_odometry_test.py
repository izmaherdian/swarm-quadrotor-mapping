#!/usr/bin/env python3
"""
GZ-only multi-drone odometry test.
Subscribes to /model/iris_{i}/odometry for i=1..N using pure GZ transport.
Prints frame_id and Y position to verify each drone's topic delivers correct data.
Usage:
  python3 gz_odometry_test.py [num_drones=7]
"""
import sys
import signal
from gz.transport import Node
from gz.msgs.odometry_pb2 import Odometry

subscribers = []
running = True

def make_callback(drone_id):
    def cb(msg):
        y = msg.pose.pose.position.y
        frame = msg.header.frame_id
        print(f"[iris_{drone_id}] frame={frame}  Y={y:.3f}m")
    return cb

def main():
    global running
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 7

    node = Node()
    for i in range(1, n + 1):
        topic = f"/model/iris_{i}/odometry"
        cb = make_callback(i)
        if node.subscribe(Odometry, topic, cb):
            print(f"Subscribed to {topic}")
            subscribers.append((topic, cb))
        else:
            print(f"FAILED to subscribe to {topic}")

    def shutdown(sig, frame):
        global running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    print("\nListening (Ctrl+C to stop)...\n")
    while running:
        signal.pause()

if __name__ == "__main__":
    main()
