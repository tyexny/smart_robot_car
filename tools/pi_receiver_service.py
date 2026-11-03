"""Start/stop one Pi receiver in its own session, independent of an SSH terminal."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from pi_runtime_config import load_config

ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT / '.runtime' / 'pi_receiver.json'
RECEIVER = ROOT / 'raspberry_pi' / 'udp_yolo_receiver.py'


def start_time(pid):
    return (Path('/proc')/str(pid)/'stat').read_text().partition(')')[2].split()[19]


def running():
    try:
        saved = json.loads(STATE.read_text())
        pid = saved['pid']
        if type(pid) is not int or pid < 2:
            return None
        cmdline = (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
        if os.fsencode(str(RECEIVER)) not in cmdline or saved['start_time'] != start_time(pid):
            return None
        return saved
    except (OSError, ValueError, KeyError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'status'])
    parser.add_argument('mode', nargs='?', choices=['vision', 'target', 'yolo'], default='vision')
    args, extra = parser.parse_known_args()
    if os.name != 'posix' or not Path('/proc').is_dir():
        parser.error('Run this helper on the Raspberry Pi')
    saved = running()
    if args.action == 'status':
        print(json.dumps(saved) if saved else 'Receiver is not running under this helper')
        return
    if args.action == 'stop':
        if not saved:
            print('No matching managed receiver; no signal sent')
            return
        os.kill(saved['pid'], signal.SIGINT)
        for _ in range(80):
            if not running():
                STATE.unlink(missing_ok=True)
                print('Stopped; inspect log for SAVED if recording')
                return
            time.sleep(.1)
        print('Receiver is still shutting down; inspect log before restarting')
        return
    if saved:
        raise SystemExit(f"Receiver already running: PID {saved['pid']}, mode {saved['mode']}")
    try:
        load_config()
    except ValueError as error:
        raise SystemExit(str(error)) from None
    python = ROOT / '.venv-task6' / 'bin' / 'python'
    if not python.is_file():
        raise SystemExit('Run install_task6_pi.sh first')
    # Catch a foreground receiver too, rather than launching a duplicate.
    for kind, port in [(socket.SOCK_DGRAM, 5000), (socket.SOCK_STREAM, 8088)]:
        with socket.socket(socket.AF_INET, kind) as probe:
            if kind == socket.SOCK_STREAM:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(('0.0.0.0', port))
            except OSError:
                raise SystemExit(f'Port {port} is busy; stop the existing receiver first')
    logdir = ROOT / 'raspberry_pi' / 'logs'
    logdir.mkdir(parents=True, exist_ok=True)
    log = logdir / f'{args.mode}_{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns()}.log'
    command = [str(python), '-u', str(RECEIVER), '--task6-pc-url', os.environ['TASK6_PC_URL'],
               '--task-console', '--web-port', '8088', *extra, '--task6-mode', args.mode]
    if args.mode == 'vision':
        command += ['--no-task6-hsv-display']
    with log.open('ab') as output:
        os.chmod(log, 0o600)
        child = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=output,
                                 stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(.6)
    if child.poll() is not None:
        raise SystemExit(f'Receiver exited with {child.returncode}; see {log}')
    STATE.parent.mkdir(parents=True, exist_ok=True)
    STATE.write_text(json.dumps(dict(pid=child.pid, mode=args.mode,
                                    start_time=start_time(child.pid), log=str(log))))
    print(f'Started PID {child.pid}, mode {args.mode}; SSH disconnect will not stop it')
    print(f'Log: {log}')
    print(f'View: tail -f "{log}"')
    print('Use the Phone UI address printed in that log')
    for _ in range(25):
        lines = [line for line in log.read_text(errors='replace').splitlines() if 'Phone UI:' in line]
        if lines:
            print(lines[-1])
            break
        time.sleep(.1)


if __name__ == '__main__':
    main()
