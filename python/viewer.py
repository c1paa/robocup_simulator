#!/usr/bin/env python3
"""Minimal demo client: streams the robot's mirror-camera feed over gRPC and
shows it with OpenCV. Run the simulator first, then:

    python viewer.py [grpc_port]
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "generated"))

import grpc
import numpy as np
import cv2

import simulator_pb2
import simulator_pb2_grpc


def main():
    port = 50051
    if len(sys.argv) > 1:
        port = int(sys.argv[1])

    channel = grpc.insecure_channel(f"localhost:{port}")
    stub = simulator_pb2_grpc.SimulatorStub(channel)

    request = simulator_pb2.SensorRequest(robot_id=0)
    for data in stub.SensorStream(request):
        img = np.frombuffer(data.image_data, dtype=np.uint8)
        expected = data.image_width * data.image_height * 3
        if img.size != expected:
            continue
        img = img.reshape((data.image_height, data.image_width, 3))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow("Robot camera", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
