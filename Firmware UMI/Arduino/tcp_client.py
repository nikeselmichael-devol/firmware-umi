#!/usr/bin/env python3
"""
TCP client for the ESP32-C3 dual-encoder + IMU firmware.

Connects to the ESP32's TCP server and prints/parses the newline-delimited
JSON stream it publishes at 100 Hz.

Usage:
    python3 tcp_client.py <ESP32_IP> [--port 5000] [--csv out.csv]

Example:
    python3 tcp_client.py 192.168.1.42
    python3 tcp_client.py 192.168.1.42 --csv log.csv
"""

import argparse
import csv
import json
import socket
import sys
import time


def main():
    parser = argparse.ArgumentParser(description="ESP32 sensor TCP client")
    parser.add_argument("host", help="ESP32 IP address (see its Serial Monitor output)")
    parser.add_argument("--port", type=int, default=5000, help="TCP port (default 5000)")
    parser.add_argument("--csv", default=None, help="Optional path to also log samples as CSV")
    parser.add_argument("--print-every", type=int, default=1,
                         help="Print every Nth sample to avoid flooding the terminal (default 1 = print all)")
    args = parser.parse_args()

    csv_writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["t_ms", "enc1_deg", "enc1_err", "enc2_deg", "enc2_err",
                              "ax", "ay", "az", "gx", "gy", "gz", "recv_time"])

    print(f"Connecting to {args.host}:{args.port} ...")
    sock = socket.create_connection((args.host, args.port), timeout=5)
    sock.settimeout(2.0)
    print("Connected. Streaming... (Ctrl+C to stop)\n")

    buf = ""
    count = 0
    last_report = time.time()
    samples_since_report = 0

    try:
        while True:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                print("No data received in 2s — check firmware is still running / connected to WiFi.")
                continue

            if not chunk:
                print("Connection closed by device.")
                break

            buf += chunk.decode("utf-8", errors="replace")

            # The firmware sends one JSON object per line (newline-delimited)
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                try:
                    sample = json.loads(line)
                except json.JSONDecodeError:
                    print(f"Skipping malformed line: {line!r}")
                    continue

                count += 1
                samples_since_report += 1

                if csv_writer:
                    csv_writer.writerow([
                        sample.get("t"),
                        sample.get("enc1_deg"), sample.get("enc1_err"),
                        sample.get("enc2_deg"), sample.get("enc2_err"),
                        sample.get("ax"), sample.get("ay"), sample.get("az"),
                        sample.get("gx"), sample.get("gy"), sample.get("gz"),
                        time.time(),
                    ])

                if args.print_every > 0 and count % args.print_every == 0:
                    print(
                        f"[{sample.get('t')}] "
                        f"enc1={sample.get('enc1_deg'):7.2f}deg "
                        f"enc2={sample.get('enc2_deg'):7.2f}deg | "
                        f"acc=({sample.get('ax'):+.2f},{sample.get('ay'):+.2f},{sample.get('az'):+.2f}) m/s^2 "
                        f"gyro=({sample.get('gx'):+.2f},{sample.get('gy'):+.2f},{sample.get('gz'):+.2f}) deg/s"
                    )

                # Report actual received rate every second
                now = time.time()
                if now - last_report >= 1.0:
                    print(f"--- receiving ~{samples_since_report} samples/sec ---")
                    samples_since_report = 0
                    last_report = now

    except KeyboardInterrupt:
        print(f"\nStopped. Received {count} samples total.")
    finally:
        sock.close()
        if csv_file:
            csv_file.close()
            print(f"CSV log saved to {args.csv}")


if __name__ == "__main__":
    main()
