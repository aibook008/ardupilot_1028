#!/usr/bin/env python3
"""DF-200 bias-simulation case runner - closed loop + matrix mode.

Single case closed loop:
    inject params -> fly full-chain AUTO mission -> disarm -> archive

Matrix mode:
    --matrix cla+20 cla-20 cd0+30 --params-file cases.json
    runs each case in sequence. Between cases the script DISARMS and
    re-injects from a RESET state: user restarts SITL with -w between
    runs OR the script relies on every case setting ALL params it cares
    about (explicitly set, read-back verified) so nothing leaks. The
    simplest robust flow: stop SITL, start SITL (user or wrapper), run
    next case - the matrix driver therefore shells out to a restart
    command if --restart-cmd is given.

Archive (all under logs/ for traceability):
    logs/df200_cases/<case>_<NNN>/run.log       script output
    logs/df200_cases/<case>_<NNN>/flight.BIN    vehicle DataFlash copy
    logs/df200_cases/<case>_<NNN>/result.json   params + milestones + status
    logs/df200_cases/summary_<timestamp>.md     matrix summary table

Mission: DF200_full.txt = A->B straight line (2.8km, 50m AGL, VTOL
takeoff at A, VTOL land at B, ~2.5min per flight).
"""
import argparse
import json
import glob
import os
import shutil
import subprocess
import sys
import time

from pymavlink import mavutil

MISSION_TYPE_MISSION = 0
MODE_AUTO = 10
CASE_ROOT = os.path.join('logs', 'df200_cases')
DEFAULT_MISSION = 'Tools/autotest/Generic_Missions/DF200_full.txt'
MILESTONES = [
    'VTOL takeoff', 'yaw aligned', 'Transition', 'airspeed reached',
    'Transition FW done', 'VTOL position1', 'VTOL Overshoot',
    'VTOL position2', 'Land descend', 'Land final', 'Land complete',
    'Throttle disarmed', 'Hit ground', 'Failsafe',
]

# predefined perturbation library: name -> param dict
CASE_LIB = {
    'baseline':  {},
    'cla+20':    {'SIM_AERO_CLA_SCL': 1.2},
    'cla-20':    {'SIM_AERO_CLA_SCL': 0.8},
    'cd0+30':    {'SIM_AERO_CD0_SCL': 1.3},
    'ind+30':    {'SIM_AERO_IND_SCL': 1.3},
    'cma-25':    {'SIM_AERO_CMA_SCL': 0.75},
    'ele-30':    {'SIM_AERO_ELE_SCL': 0.7},
    'ail-30':    {'SIM_AERO_AIL_SCL': 0.7},
    'rud-30':    {'SIM_AERO_RUD_SCL': 0.7},
    'cgx+8cm':   {'SIM_AERO_CGX_OFF': 0.08},
    'mass+15':   {'SIM_MASS_SCL': 1.15},
    'inertia+30': {'SIM_INERTIA_SCL': 1.3},
}


def connect(retries=3):
    """Connect with real-heartbeat verification.

    wait_heartbeat() returns None on timeout without raising, and SITL serial
    links serve clients unreliably right after a previous connection closes
    (matrix case transitions) - so require a REAL heartbeat (sysid != 0) and
    retry across ports.
    """
    for attempt in range(retries):
        for conn in ('tcp:127.0.0.1:5762', 'tcp:127.0.0.1:5760'):
            try:
                m = mavutil.mavlink_connection(conn, source_system=250)
                hb = m.wait_heartbeat(timeout=8)
                if hb is None or hb.get_srcSystem() == 0:
                    print('connect %s: no heartbeat (attempt %d)'
                          % (conn, attempt + 1), flush=True)
                    m.close()
                    time.sleep(2)
                    continue
                print('hb ok via %s sysid=%d' % (conn, hb.get_srcSystem()),
                      flush=True)
                time.sleep(2)
                return m
            except Exception:
                try:
                    m.close()
                except Exception:
                    pass
        time.sleep(3)
    print('ERROR: no MAVLink link with heartbeat on 5762/5760', flush=True)
    return None


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
        ok = p is not None and abs(p.param_value - float(value)) < 1e-4
        print('  param %-18s = %-8.4g %s' % (name, got or -1,
                                             'ok' if ok else 'FAIL'), flush=True)
        if not ok:
            return False
    return True


def parse_wpl(path):
    lines = [l.strip() for l in open(path) if l.strip() and not l.startswith('#')]
    assert lines[0].startswith('QGC WPL 110'), 'not QGC WPL: %s' % path
    items = []
    for l in lines[1:]:
        f = l.split()
        items.append(dict(seq=int(f[0]), frame=int(f[2]), cmd=int(f[3]),
                          p=[float(x) for x in f[4:8]],
                          x=float(f[8]), y=float(f[9]), z=float(f[10]),
                          auto=int(f[11])))
    items.sort(key=lambda it: it['seq'])
    return items


def upload(m, items, timeout=30):
    """Strict handshake upload: send only what the vehicle REQUESTS."""
    m.mav.mission_count_send(m.target_system, m.target_component, len(items),
                             MISSION_TYPE_MISSION)
    sent, ack = 0, None
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=5)
        if msg is None:
            continue
        if msg.get_type() == 'MISSION_ACK':
            ack = msg.type
            print('  MISSION_ACK %d (%s)' % (
                ack, 'ACCEPTED' if ack == 0 else 'REJECTED'), flush=True)
            break
        it = items[msg.seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, msg.seq,
            it['frame'], it['cmd'], 0, it['auto'],
            it['p'][0], it['p'][1], it['p'][2], it['p'][3],
            int(round(it['x'] * 1e7)), int(round(it['y'] * 1e7)), it['z'],
            MISSION_TYPE_MISSION)
        sent += 1
        print('  item %d cmd %d' % (msg.seq, it['cmd']), flush=True)
    return sent, ack


def newest_bin():
    bins = sorted(glob.glob('logs/*.BIN'), key=os.path.getmtime)
    return bins[-1] if bins else None


class Tee:
    def __init__(self, path):
        self.f = open(path, 'w')

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()


def run_case(case_name, params, mission, max_time, restart_cmd=None):
    """One closed loop. Returns result dict."""
    n = 1
    while os.path.exists(os.path.join(CASE_ROOT, '%s_%03d' % (case_name, n))):
        n += 1
    case_dir = os.path.join(CASE_ROOT, '%s_%03d' % (case_name, n))
    os.makedirs(case_dir)
    sys.stdout = Tee(os.path.join(case_dir, 'run.log'))

    print('case %s | params %s | mission %s' % (case_name, params, mission),
          flush=True)
    result = dict(case=case_name, mission=mission, params=params,
                  milestones=[], status='RUNNING')

    if restart_cmd and restart_cmd.lower() != 'none':
        print('restarting SITL: %s' % restart_cmd, flush=True)
        subprocess.run(restart_cmd, shell=True)
        print('waiting 20s for SITL boot...', flush=True)
        time.sleep(20)

    m = connect()
    if m is None:
        result['status'] = 'FAIL: no link'
        json.dump(result, open(os.path.join(case_dir, 'result.json'), 'w'),
                  indent=1)
        sys.stdout = sys.__stdout__
        return result

    # 1. inject bias params
    if params:
        print('injecting params...', flush=True)
        if not set_params(m, list(params.items())):
            result['status'] = 'FAIL: param inject'
            json.dump(result, open(os.path.join(case_dir, 'result.json'), 'w'),
                      indent=1)
            sys.stdout = sys.__stdout__
            return result

    # 2. upload mission
    items = parse_wpl(mission)
    print('uploading mission (%d items)...' % len(items), flush=True)
    sent, ack = upload(m, items)
    if sent != len(items) or ack != 0:
        print('UPLOAD FAILED (sent %d/%d ack=%s)' % (sent, len(items), ack),
              flush=True)
        result['status'] = 'FAIL: upload'
        json.dump(result, open(os.path.join(case_dir, 'result.json'), 'w'),
                  indent=1)
        sys.stdout = sys.__stdout__
        return result

    # 3. AUTO + arm
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        MODE_AUTO)
    time.sleep(1)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, 0, 0, 0, 0, 0, 0)
    print('AUTO + arm sent, monitoring...', flush=True)

    # 4. monitor to disarm
    t0 = time.time()
    disarmed = False
    while time.time() - t0 < max_time:
        msg = m.recv_match(blocking=True, timeout=2)
        if msg is None:
            continue
        typ = msg.get_type()
        t = time.time() - t0
        if typ == 'HEARTBEAT':
            if msg.get_srcSystem() != 1:
                continue
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) \
                    and t > 15:
                print('[%7.1f] DISARMED - flight end' % t, flush=True)
                disarmed = True
                break
        elif typ == 'STATUSTEXT':
            print('[%7.1f] %s' % (t, msg.text), flush=True)
            if any(k.lower() in msg.text.lower() for k in MILESTONES):
                result['milestones'].append(dict(t=round(t, 1), text=msg.text))
    if not disarmed:
        print('TIMEOUT/interrupt after %.0fs' % (time.time() - t0), flush=True)
        result['status'] = 'FAIL: timeout'
        result['milestones'].append(dict(t=round(time.time() - t0, 1),
                                         text='TIMEOUT'))

    # 5. archive
    src = newest_bin()
    if src:
        dst = os.path.join(case_dir, 'flight.BIN')
        shutil.copy2(src, dst)
        result['log_bin'] = dst
        result['log_bin_size'] = os.path.getsize(dst)
        print('archived %s -> %s' % (src, dst), flush=True)
    ok_end = (result['status'] == 'RUNNING'
              and any('Land complete' in x['text']
                      for x in result['milestones'])
              # normal VTOL landing also logs "Hit ground" - only treat it
              # as a failure when Land complete never came
              and not any('Hit ground' in x['text']
                          and not any('Land complete' in y['text']
                                      for y in result['milestones'])
                          for x in result['milestones']))
    result['status'] = 'PASS' if ok_end else result.get('status', 'FAIL')
    json.dump(result, open(os.path.join(case_dir, 'result.json'), 'w'),
              indent=1)
    print('CASE %s: %s' % (case_name, result['status']), flush=True)
    sys.stdout = sys.__stdout__
    # release the serial link for the next case (matrix mode reuses SITL)
    try:
        m.close()
    except Exception:
        pass
    return result


def write_summary(results):
    ts = time.strftime('%Y%m%d_%H%M%S')
    path = os.path.join(CASE_ROOT, 'summary_%s.md' % ts)
    with open(path, 'w') as f:
        f.write('# DF-200 bias matrix summary %s\n\n' % ts)
        f.write('| case | params | status | milestones | archive |\n')
        f.write('|------|--------|--------|------------|---------|\n')
        for r in results:
            p = ' '.join('%s=%g' % kv for kv in r['params'].items()) or '(baseline)'
            ms = len(r['milestones'])
            arch = r.get('log_bin', '-')
            f.write('| %s | %s | %s | %d | %s |\n'
                    % (r['case'], p, r['status'], ms, arch))
    print('summary: %s' % path, flush=True)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--case',
                    help='single case name (CASE_LIB key) or arbitrary name '
                         'with --param')
    ap.add_argument('--param', action='append', default=[],
                    help='NAME=VALUE (single-case mode, overrides CASE_LIB)')
    ap.add_argument('--matrix', nargs='+',
                    help='list of CASE_LIB names to run in sequence')
    ap.add_argument('--mission', default=DEFAULT_MISSION)
    ap.add_argument('--max-time', type=float, default=600,
                    help='per-case monitor timeout (s), default 600 = 10min')
    ap.add_argument('--restart-cmd', default='./docs/scripts/df200_sitl_restart.sh',
                    help='shell command run automatically before EACH case to '
                         'restart SITL (default: the helper script - kill old '
                         'SITL, wipe params with -w, boot fresh at KSFO). '
                         'Pass "none" to reuse the running SITL instead '
                         '(params from earlier cases then LEAK).')
    args = ap.parse_args()

    results = []
    if args.matrix:
        for name in args.matrix:
            if name not in CASE_LIB:
                print('unknown matrix case %s (lib: %s)'
                      % (name, sorted(CASE_LIB)), flush=True)
                sys.exit(1)
            results.append(run_case(name, CASE_LIB[name], args.mission,
                                    args.max_time, args.restart_cmd))
            time.sleep(5)
        write_summary(results)
    else:
        name = args.case or 'case'
        params = dict(CASE_LIB.get(name, {}))
        for kv in args.param:
            k, v = kv.split('=', 1)
            params[k] = float(v)
        r = run_case(name, params, args.mission, args.max_time,
                     args.restart_cmd)
        print('CASE %s: %s' % (name, r['status']), flush=True)


if __name__ == '__main__':
    main()
