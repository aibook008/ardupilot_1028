#!/usr/bin/env python3
"""DF-200 mission runner (fully scripted, no QGC interaction needed).

Two execution profiles:

  --profile hover   (default) MC hover subject - no mission file needed:
      GUIDED -> arm -> MAV_CMD_NAV_TAKEOFF(z) [vehicle climbs by itself]
      -> QLOITER at altitude -> hold --hold seconds -> QLAND -> disarm.
      QLOITER is in_vtol_mode() so the FW transition state machine never
      runs (ArduPlane/quadplane.cpp:1764 gates transition->update() on
      "not in vtol mode") - no Q_ASSIST trigger, no Lua mix fade.
      MAV_CMD_NAV_TAKEOFF requires GUIDED (QuadPlane::do_user_takeoff).

  --profile auto    AUTO mission subject (brake/full/transition/cruise):
      upload mission (strict per-item handshake + download verify) ->
      AUTO -> arm -> monitor until disarm.

Usage (from repo root, SITL already running):
    python3 docs/scripts/df200_mission_run.py --profile hover --hold 20
    python3 docs/scripts/df200_mission_run.py --profile auto \
        Tools/autotest/Generic_Missions/DF200_brake.txt
    ... [--param SIM_MASS_SCL=1.15 ...]   bias injection before arming

Link: tries SERIAL1 (5762) first - SERIAL0 (5760) only serves the first
client, which is QGC when it is connected.
"""
import argparse
import socket
import struct
import sys
import threading
import time

from pymavlink import mavutil

SITL_RCIN_PORT = 5501  # SITL sim RC input, UDP (SITL_cmdline.cpp:223; +10*instance)

MISSION_TYPE_MISSION = 0

# ArduPlane custom modes (ArduPlane/mode.h:33)
MODE = dict(GUIDED=15, QLOITER=19, QLAND=20, AUTO=10)


def connect():
    m = None
    for conn in ('tcp:127.0.0.1:5762',  # SERIAL1: free when SERIAL0 taken by QGC
                 'tcp:127.0.0.1:5760'):  # SERIAL0: only client when QGC absent
        try:
            m = mavutil.mavlink_connection(conn, source_system=250)
            m.wait_heartbeat(timeout=8)
            print('hb ok via %s: sysid=%d' % (conn, m.target_system), flush=True)
            break
        except Exception:
            if m is not None:
                m.close()
            m = None
    if m is None:
        print('no MAVLink link on 5762/5760 - is SITL running?', flush=True)
        sys.exit(1)
    time.sleep(2)
    return m


def parse_wpl(path):
    """Parse a QGC WPL 110 waypoint file into mission item dicts."""
    lines = [l.strip() for l in open(path) if l.strip() and not l.startswith('#')]
    assert lines[0].startswith('QGC WPL 110'), 'not a QGC WPL file: %s' % path
    items = []
    for l in lines[1:]:
        f = l.split()
        items.append(dict(
            seq=int(f[0]), frame=int(f[2]), cmd=int(f[3]),
            p=[float(f[4]), float(f[5]), float(f[6]), float(f[7])],
            x=float(f[8]), y=float(f[9]), z=float(f[10]),
            auto=int(f[11])))
    items.sort(key=lambda it: it['seq'])
    return items


def upload(m, items, timeout=30):
    """Strict per-item handshake upload. Returns (sent, ack_type).

    v1 of this script sent items blindly and uploaded 3/6 (MISSION_ACK
    TIMEOUT) - the vehicle flew a truncated mission. The vehicle must
    REQUEST each item.
    """
    m.mav.mission_count_send(m.target_system, m.target_component, len(items),
                             MISSION_TYPE_MISSION)
    sent = 0
    ack = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=5)
        if msg is None:
            continue
        typ = msg.get_type()
        if typ == 'MISSION_ACK':
            ack = msg.type
            print('  MISSION_ACK type=%d %s' % (
                ack, 'ACCEPTED' if ack == 0 else 'REJECTED/TIMEOUT'), flush=True)
            break
        seq = msg.seq
        it = items[seq]
        # signature: (target_system, target_component, seq, frame, command,
        #             current, autocontinue, param1..4, x, y, z, mission_type)
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, seq,
            it['frame'], it['cmd'], 0, it['auto'],
            float(it['p'][0]), float(it['p'][1]),
            float(it['p'][2]), float(it['p'][3]),
            int(round(it['x'] * 1e7)), int(round(it['y'] * 1e7)),
            float(it['z']),
            MISSION_TYPE_MISSION)
        sent += 1
        print('  -> item %d (cmd %d)' % (seq, it['cmd']), flush=True)
    return sent, ack


def download_and_verify(m, items, timeout=20):
    """Download the mission back and compare against `items`. Returns ok."""
    m.mav.mission_request_list_send(m.target_system, m.target_component,
                                    MISSION_TYPE_MISSION)
    msg = m.recv_match(type='MISSION_COUNT', blocking=True, timeout=timeout)
    if msg is None or msg.count != len(items):
        print('  download count mismatch: %s' % (msg.count if msg else None),
              flush=True)
        return False
    got = {}
    t0 = time.time()
    while len(got) < msg.count and time.time() - t0 < timeout:
        m.mav.mission_request_int_send(m.target_system, m.target_component,
                                       len(got), MISSION_TYPE_MISSION)
        r = m.recv_match(type=['MISSION_ITEM_INT', 'MISSION_REQUEST'],
                         blocking=True, timeout=3)
        if r is None or r.get_type() != 'MISSION_ITEM_INT':
            continue
        got[r.seq] = r
    if len(got) != len(items):
        print('  download incomplete: %d/%d' % (len(got), len(items)), flush=True)
        return False
    for it in items:
        r = got[it['seq']]
        if it['seq'] == 0:
            # seq0 is the home row: the vehicle overwrites it with its own
            # SITL home (ground offset alters z by ~0.1m, coords by ~4e-7 deg),
            # so only check the command id here.
            if r.command != it['cmd']:
                print('  MISMATCH seq=0 cmd %d != %d' % (r.command, it['cmd']),
                      flush=True)
                return False
            continue
        if (r.command != it['cmd']
                or abs(r.x - it['x'] * 1e7) > 10
                or abs(r.y - it['y'] * 1e7) > 10
                or abs(r.z - it['z']) > 0.5):
            print('  MISMATCH seq=%d: cmd %d!=%d x %.0f!=%.0f y %.0f!=%.0f '
                  'z %.2f!=%.2f' % (
                      it['seq'], r.command, it['cmd'],
                      r.x, it['x'] * 1e7, r.y, it['y'] * 1e7, r.z, it['z']),
                  flush=True)
            return False
    print('  download verify: %d/%d items OK' % (len(items), len(items)),
          flush=True)
    return True


def set_params(m, pairs):
    for name, value in pairs:
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.3)
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        p = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        got = p.param_value if p else None
        status = 'ok' if (p is not None
                          and abs(p.param_value - float(value)) < 1e-4) else 'FAIL'
        print('  param %s = %s (set %.4f) %s' % (name, got, float(value), status),
              flush=True)
        if status == 'FAIL':
            return False
    return True


def set_mode(m, mode_number, name, timeout=8):
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_number)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg and msg.get_srcSystem() == 1 and msg.custom_mode == mode_number:
            print('mode=%s ok' % name, flush=True)
            return True
    print('MODE CHANGE TO %s FAILED' % name, flush=True)
    return False


def wait_altitude(m, alt_min, alt_max, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg is None:
            continue
        rel = msg.relative_alt / 1000.0
        if alt_min <= rel <= alt_max:
            print('altitude %.1f m reached' % rel, flush=True)
            return True
    print('ALTITUDE WAIT TIMEOUT (%.0f..%.0f m)' % (alt_min, alt_max), flush=True)
    return False


def monitor(m, max_time, on_event=None):
    """Common monitor loop. Returns True if disarmed before timeout.

    on_event(t, msg) -> 'stop' can inject profile-specific behavior.
    Only trusts vehicle heartbeats (srcSystem==1); GCS heartbeats (QGC,
    srcSystem 255) carry custom_mode too and would fake mode flapping.
    """
    t0 = time.time()
    last_mode = None
    while time.time() - t0 < max_time:
        msg = m.recv_match(blocking=True, timeout=2)
        if msg is None:
            continue
        typ = msg.get_type()
        t = time.time() - t0
        if typ == 'HEARTBEAT' and msg.get_srcSystem() != 1:
            continue
        if typ == 'STATUSTEXT':
            print('[%7.1f] %s' % (t, msg.text), flush=True)
            if on_event and on_event(t, msg) == 'stop':
                return True
        elif typ == 'HEARTBEAT':
            try:
                mode = m.mode_mapping()[msg.custom_mode]
            except Exception:
                mode = msg.custom_mode
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if (mode, armed) != last_mode:
                print('[%7.1f] mode=%s armed=%s' % (t, mode, armed), flush=True)
                last_mode = (mode, armed)
            if not armed and t > 10:
                print('[%7.1f] DISARMED -> mission end' % t, flush=True)
                return True
    print('TIMEOUT after %.0fs' % max_time, flush=True)
    return False


def send_takeoff(m, alt, max_tries=6):
    """Send MAV_CMD_NAV_TAKEOFF with retry until ACK ACCEPTED.

    do_user_takeoff() rejects the command while the arming sequence is still
    completing ("Must be armed for takeoff", quadplane.cpp:4170) - the first
    attempt can legitimately fail, so retry with confirmation each time.
    Frame LOCAL_OFFSET_NED, z DOWN-positive; the handler negates it
    (GCS_Mavlink.cpp:1193), so send z = -alt for an upward target.
    """
    for attempt in range(1, max_tries + 1):
        m.mav.command_int_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0,
            0, 0, 0, 0,
            0, 0, -float(alt))
        # drain to OUR ack (match command id - other links' ACKs share the wire)
        t0 = time.time()
        while time.time() - t0 < 5:
            ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack is None:
                break
            if ack.command != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                continue  # someone else's ack (QGC REQUEST_MESSAGE etc.)
            if ack.result == 0:
                print('  takeoff accepted (try %d)' % attempt, flush=True)
                return True
            print('  takeoff rejected result=%d (try %d)' % (ack.result, attempt),
                  flush=True)
            break
        time.sleep(1.5)
    print('TAKEOFF: no ACCEPTED ack after %d tries' % max_tries, flush=True)
    return False


def wait_tilt_vertical(m, timeout=30):
    """Wait for DISARMED_TILT_UP (Q_OPTIONS bit21) to slew tilt to vertical.

    NOTE: the TILT log message is DataFlash-only (ArduPlane/Log.cpp:446), it
    never streams over MAVLink - waiting for it always times out. Use the
    tilt servos instead: SERVO13/14 (k_motor_tilt), 1000us = vertical,
    2000us = forward (df200 parm convention).
    """
    # request SERVO_OUTPUT_RAW stream
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
    t0 = time.time()
    s13 = s14 = None
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
        if msg is None:
            continue
        # field names are servo13_raw/servo14_raw (chan*_raw belongs to
        # RC_CHANNELS, a different message)
        s13 = getattr(msg, 'servo13_raw', None)
        s14 = getattr(msg, 'servo14_raw', None)
        if s13 is None or s14 is None:
            continue
        if s13 <= 1100 and s14 <= 1100:
            print('tilt vertical (servo13/14 = %d/%d us)' % (s13, s14),
                  flush=True)
            return True
    print('TILT NOT VERTICAL after %.0fs (servo13/14 = %s/%s us)' %
          (timeout, s13, s14), flush=True)
    return False


def run_hover(m, args):
    """GUIDED -> arm -> NAV_TAKEOFF(z) -> QLOITER -> hold -> QLAND."""
    # defensive: clear any leftover mission. A persisted mission firing
    # "Mission complete -> RTL" stole the QLOITER hold in run 16.
    m.mav.command_long_send(m.target_system, m.target_component,
                            45,  # MAV_CMD_MISSION_CLEAR_ALL (MAVLink spec cmd 45;
                                 # constant missing in this pymavlink dialect build)
                            0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.5)
    print('leftover mission cleared', flush=True)

    if not set_mode(m, MODE['GUIDED'], 'GUIDED'):
        sys.exit(2)
    if not wait_tilt_vertical(m):
        sys.exit(2)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, 0, 0, 0, 0, 0, 0)
    print('arm sent', flush=True)
    # wait for armed state instead of a blind sleep - arming completes
    # asynchronously and an early takeoff command is rejected
    t0 = time.time()
    while time.time() - t0 < 15:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if (hb and hb.get_srcSystem() == 1
                and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print('armed confirmed', flush=True)
            break
    else:
        print('ARM FAILED (no armed heartbeat in 15s)', flush=True)
        sys.exit(2)

    print('takeoff to %.0f m' % args.alt, flush=True)
    if not send_takeoff(m, args.alt):
        sys.exit(3)
    if not wait_altitude(m, args.alt - 2, args.alt + 5, timeout=120):
        sys.exit(3)

    if not set_mode(m, MODE['QLOITER'], 'QLOITER'):
        sys.exit(4)
    # SITL with no RC driver keeps throttle at 1000 (SITL_State.cpp:307-310).
    # QLOITER reads sub-mid throttle as a descend command, so the "hover"
    # silently descends (confirmed: run 15, DAlt tracked Alt down 18->3 m).
    # MAVLink RC_CHANNELS_OVERRIDE loses to the sim's own RC stream, so drive
    # the sim RC input the same way autotest's rc_thread_main does: raw 16x
    # uint16 PWM UDP packets to port 5501 at 20 Hz (ch1..4 = 1500 mid-stick).
    print('[hover] sim RC feed: ch1-4=1500 mid stick (alt hold)', flush=True)
    stop_rc = threading.Event()

    def rc_feed():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        chans = [1000] * 16
        chans[0] = chans[1] = chans[2] = chans[3] = 1500  # aile/elev/thr/rud
        buf = struct.pack('<16H', *chans)
        while not stop_rc.is_set():
            sock.sendto(buf, ('127.0.0.1', SITL_RCIN_PORT))
            time.sleep(0.05)
        sock.close()

    feeder = threading.Thread(target=rc_feed, daemon=True)
    feeder.start()
    time.sleep(args.hold)
    stop_rc.set()
    feeder.join(timeout=2)
    print('[hover] hold complete -> QLAND', flush=True)
    if not set_mode(m, MODE['QLAND'], 'QLAND'):
        sys.exit(5)
    if not monitor(m, args.max_time):
        sys.exit(6)
    print('done', flush=True)


def run_auto(m, args):
    """Upload mission -> AUTO -> arm -> monitor to disarm."""
    items = parse_wpl(args.mission)
    print('mission %s: %d items' % (args.mission, len(items)), flush=True)

    print('uploading...', flush=True)
    sent, ack = upload(m, items)
    print('sent %d/%d' % (sent, len(items)), flush=True)
    if sent != len(items) or ack != 0:
        print('UPLOAD FAILED (ack=%s) - NOT arming' % ack, flush=True)
        sys.exit(2)

    print('verifying by download...', flush=True)
    if not download_and_verify(m, items):
        print('VERIFY FAILED - NOT arming', flush=True)
        sys.exit(3)

    if not set_mode(m, MODE['AUTO'], 'AUTO'):
        sys.exit(4)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, 0, 0, 0, 0, 0, 0)
    print('arm sent', flush=True)
    if not monitor(m, args.max_time):
        sys.exit(5)
    print('done', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mission', nargs='?',
                    help='QGC WPL mission file (profile auto only)')
    ap.add_argument('--profile', choices=['hover', 'auto'], default='hover',
                    help='hover: GUIDED takeoff + QLOITER hold + QLAND; '
                         'auto: mission upload + AUTO')
    ap.add_argument('--alt', type=float, default=20,
                    help='hover takeoff altitude (m, profile hover)')
    ap.add_argument('--hold', type=float, default=20,
                    help='hover hold seconds (profile hover)')
    ap.add_argument('--param', action='append', default=[],
                    help='NAME=VALUE, repeatable; set before arming')
    ap.add_argument('--max-time', type=float, default=900,
                    help='monitor timeout (s)')
    args = ap.parse_args()

    m = connect()

    if args.param:
        print('setting params...', flush=True)
        pairs = []
        for kv in args.param:
            name, value = kv.split('=', 1)
            pairs.append((name, value))
        if not set_params(m, pairs):
            print('PARAM SET FAILED - NOT arming', flush=True)
            sys.exit(4)

    if args.profile == 'hover':
        run_hover(m, args)
    else:
        if not args.mission:
            print('profile auto needs a mission file argument', flush=True)
            sys.exit(1)
        run_auto(m, args)


if __name__ == '__main__':
    main()
