"""Bridge: node sentences ($UAVDOA) -> Cursor-on-Target -> ATAK / WinTAK / TAK Server.

    # one node on USB, CoT to the default ATAK SA multicast group
    uv run python scripts/cot_bridge.py --nodes nodes.json --serial /dev/ttyUSB0

    # several nodes, fused fixes when >= 2 of them hear the same thing
    uv run python scripts/cot_bridge.py --nodes nodes.json --serial /dev/ttyUSB0 --serial /dev/ttyUSB1

    # no hardware: synthetic 3-node scenario, print the XML instead of sending
    uv run python scripts/cot_bridge.py --demo --dry-run

nodes.json maps node id -> surveyed position:
    {"1": {"lat": 50.4501, "lon": 30.5234}, "2": {"lat": 50.4512, "lon": 30.5341}}

Output goes to UDP 239.2.3.1:6969 (ATAK's default SA multicast) unless
--dest host:port is given (e.g. a TAK Server's plain CoT input).

Everything here is receive-side host software; the nodes themselves stay
passive. Transmitting CoT over a radio link is a separate, regulated matter
under martial law.
"""

from __future__ import annotations

import argparse
import json
import queue
import socket
import sys
import threading
import time

import numpy as np

from uavdoa import cot, nmea
from uavdoa.fusion import enu_from_latlon, latlon_from_enu, triangulate

FRESH_S = 1.5  # bearings older than this are not fused
FOV_DEG = 10.0  # wedge width drawn for a single-node bearing


def reader(stream, q: queue.Queue):
    for raw in stream:
        line = raw.decode(errors="replace") if isinstance(raw, bytes) else raw
        r = nmea.parse(line)
        if isinstance(r, nmea.DoaReport):
            q.put((time.monotonic(), r))


def open_serial(port: str, baud: int):
    try:
        import serial  # pyserial, optional
    except ImportError:
        sys.exit("pyserial is needed for --serial: uv pip install pyserial")
    return serial.Serial(port, baud, timeout=1)


def demo_stream(q: queue.Queue, nodes: dict):
    """Three synthetic nodes watching a drone fly across."""
    ids = sorted(nodes, key=int)
    lat0, lon0 = nodes[ids[0]]["lat"], nodes[ids[0]]["lon"]
    xy = {i: enu_from_latlon(nodes[i]["lat"], nodes[i]["lon"], lat0, lon0) for i in ids}
    rng = np.random.default_rng(0)
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        target = np.array([-600 + 40 * (t % 50), 900.0])
        for i in ids:
            d = target - xy[i]
            b = float(np.degrees(np.arctan2(d[0], d[1])) % 360 + rng.normal(0, 2))
            body = f"UAVDOA,{i},{int(t * 1000)},1,{b:.1f},{b:.1f},20.0,0.80,12.0,0.60,1,{b:.1f}"
            q.put((time.monotonic(), nmea.parse(f"${body}*{nmea.checksum(body):02X}")))
        time.sleep(0.256)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nodes", help="JSON: node id -> {lat, lon}")
    ap.add_argument("--serial", action="append", default=[], help="serial port of a node (repeatable)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dest", default="239.2.3.1:6969", help="host:port for CoT over UDP")
    ap.add_argument("--dry-run", action="store_true", help="print CoT XML instead of sending")
    ap.add_argument("--demo", action="store_true", help="synthetic 3-node scenario")
    ap.add_argument("--sigma", type=float, default=2.0, help="assumed bearing 1-sigma, deg")
    a = ap.parse_args()

    if a.nodes:
        with open(a.nodes) as f:
            nodes = json.load(f)
    elif a.demo:
        nodes = {"1": {"lat": 50.4500, "lon": 30.5200}, "2": {"lat": 50.4509, "lon": 30.5312},
                 "3": {"lat": 50.4460, "lon": 30.5249}}
    else:
        ap.error("--nodes is required (or --demo)")

    q: queue.Queue = queue.Queue()
    if a.demo:
        threading.Thread(target=demo_stream, args=(q, nodes), daemon=True).start()
    for port in a.serial:
        threading.Thread(target=reader, args=(open_serial(port, a.baud), q), daemon=True).start()
    if not a.demo and not a.serial:
        threading.Thread(target=reader, args=(sys.stdin, q), daemon=True).start()

    host, port = a.dest.rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)  # stay on the local segment

    def send(xml: bytes):
        if a.dry_run:
            print(xml.decode())
        else:
            sock.sendto(xml, (host, int(port)))

    ids = list(nodes)
    lat0, lon0 = nodes[ids[0]]["lat"], nodes[ids[0]]["lon"]
    latest: dict[int, tuple[float, nmea.DoaReport]] = {}
    last_fuse = 0.0
    while True:
        t, r = q.get()
        pos = nodes.get(str(r.node))
        if pos is None:
            print(f"unknown node {r.node} - add it to --nodes", file=sys.stderr)
            continue
        active = r.detected or r.tracking
        bearing = (r.track_az_true_deg if r.tracking else r.az_true_deg) if active else None
        send(cot.sensor_event(r.node, pos["lat"], pos["lon"], bearing, fov_deg=FOV_DEG,
                              confidence=r.confidence))
        if active:
            latest[r.node] = (t, r)

        # Fuse at most every 0.25 s, from bearings that are fresh.
        now = time.monotonic()
        if now - last_fuse < 0.25:
            continue
        last_fuse = now
        fresh = [(n, rep) for n, (tt, rep) in latest.items() if now - tt < FRESH_S]
        if len(fresh) < 2:
            continue
        xy = np.array([enu_from_latlon(nodes[str(n)]["lat"], nodes[str(n)]["lon"], lat0, lon0)
                       for n, _ in fresh])
        b = np.array([rep.track_az_true_deg if rep.tracking else rep.az_true_deg for _, rep in fresh])
        fix = triangulate(xy, b, sigma_deg=a.sigma)
        if fix is None:
            continue
        lat, lon = latlon_from_enu(fix.xy, lat0, lon0)
        send(cot.track_event("1", lat, lon, fix.cep50_m, len(fresh)))
        if a.dry_run:
            print(f"# fix {lat:.5f},{lon:.5f} CEP50 {fix.cep50_m:.0f} m from {len(fresh)} nodes",
                  file=sys.stderr)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
