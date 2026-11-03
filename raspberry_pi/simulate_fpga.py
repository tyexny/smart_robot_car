#!/usr/bin/env python3
"""Send color bars or a real test image using the FPGA's exact UDP wire format."""

import argparse
import random
import socket
import sys
import threading
import time
from pathlib import Path

from protocol import VIDEO_HEIGHT, VIDEO_WIDTH, decode_box_packet, encode_video_line


def make_image(path):
    if path is None:
        # RGB565 color bars: white, yellow, cyan, green, magenta, red, blue, black.
        palette = (0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x0000)
        row = b"".join(word.to_bytes(2, "big") * (VIDEO_WIDTH // 8) for word in palette)
        return row * VIDEO_HEIGHT
    try:
        import cv2
        import numpy as np
    except ImportError as error:
        raise RuntimeError("--image needs numpy and OpenCV from requirements.txt") from error
    image = cv2.imread(str(path))
    if image is None:
        raise RuntimeError(f"Cannot read image {path}")
    image = cv2.resize(image, (VIDEO_WIDTH, VIDEO_HEIGHT), interpolation=cv2.INTER_AREA).astype(np.uint16)
    words = ((image[:, :, 2] >> 3) << 11) | ((image[:, :, 1] >> 2) << 5) | (image[:, :, 0] >> 3)
    return words.astype(">u2").tobytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-ip", default="192.168.10.2", help="Simulator's FPGA IPv4 address")
    parser.add_argument("--pi-ip", default="192.168.10.1")
    parser.add_argument("--video-port", type=int, default=5000)
    parser.add_argument("--box-port", type=int, default=5001)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--fps", type=float, default=10)
    parser.add_argument("--image", type=Path)
    parser.add_argument("--wire-version", type=int, choices=(1, 2), default=1)
    parser.add_argument("--lock-session", type=int, default=0)
    parser.add_argument("--lock-id", type=int, default=0,
                        help="With wire-version 2, prioritize this already known session/ID")
    parser.add_argument("--shuffle", action="store_true")
    parser.add_argument("--duplicate", action="store_true")
    parser.add_argument("--drop-line", type=int, default=-1,
                        help="Drop this line of every frame; there should be no replies")
    args = parser.parse_args()
    if args.frames < 1 or not 0 < args.fps <= 60 or not -1 <= args.drop_line < VIDEO_HEIGHT:
        parser.error("invalid frame count, fps, or drop-line")
    if not (0 <= args.lock_session <= 0xFFFFFFFF and 0 <= args.lock_id <= 0xFFFFFFFF):
        parser.error("lock session/ID must fit u32")
    if bool(args.lock_id) != bool(args.lock_session) or (args.wire_version == 1 and args.lock_id):
        parser.error("nonzero lock ID and session must both be supplied with wire-version 2")
    data = make_image(args.image)
    stop = threading.Event()
    replies = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.bind_ip, args.box_port))
        sock.settimeout(0.1)

        def receive():
            while not stop.is_set():
                try:
                    packet, address = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                try:
                    decoded = decode_box_packet(packet)
                except ValueError as error:
                    print(f"Invalid reply from {address}: {error}", file=sys.stderr)
                    continue
                replies.append(decoded.frame_id)
                print(f"reply frame={decoded.frame_id} {decoded.width}x{decoded.height} "
                      f"version={decoded.version} session={decoded.tracker_session:08x} "
                      f"objects={len(decoded.boxes)} from={address} boxes={decoded.boxes}", flush=True)

        worker = threading.Thread(target=receive, daemon=True)
        worker.start()
        try:
            destination = (args.pi_ip, args.video_port)
            interval = 1 / (args.fps * VIDEO_HEIGHT)
            for frame_id in range(args.frames):
                indexes = list(range(VIDEO_HEIGHT))
                if args.shuffle:
                    random.shuffle(indexes)
                next_row = time.monotonic()
                for y in indexes:
                    if y == args.drop_line:
                        continue
                    packet = encode_video_line(frame_id, y, data[y * 1280:(y + 1) * 1280],
                                               version=args.wire_version,
                                               lock_session=args.lock_session,
                                               lock_id=args.lock_id,
                                               lock_flags=1 if args.lock_id else 0)
                    sock.sendto(packet, destination)
                    if args.duplicate and y % 40 == 0:
                        sock.sendto(packet, destination)
                    next_row += interval
                    time.sleep(max(0, next_row - time.monotonic()))
            time.sleep(0.5)
        finally:
            stop.set()
            worker.join(timeout=1)
    print(f"Sent {args.frames} frames, received {len(replies)} replies")
    if args.drop_line >= 0:
        return 1 if replies else 0
    return 0 if replies else 1


if __name__ == "__main__":
    raise SystemExit(main())
