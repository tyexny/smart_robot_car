"""Manage the read-only preview in the background, independent of SSH windows."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

STATE = Path.home() / '.cache/smart_robot_car/thermal_preview_service.json'
VIEWER = Path(__file__).resolve().with_name('thermal_viewer.py')


def start_time(pid):
    return (Path('/proc') / str(pid) / 'stat').read_text().partition(')')[2].split()[19]


def running():
    try:
        record = json.loads(STATE.read_text())
        pid = record['pid']
        if type(pid) is not int or pid < 2:
            return None
        args = (Path('/proc') / str(pid) / 'cmdline').read_bytes().split(b'\0')
        if (os.fsencode(record['script']) not in args or
                Path(record['script']).name != 'thermal_viewer.py' or
                record['start_time'] != start_time(pid)):
            return None
        return record
    except (OSError, ValueError, KeyError, IndexError, TypeError):
        return None


def stop_preview_service(quiet=False):
    record = running()
    if record is None:
        return False
    # Only this helper's verified session is signalled, never all Python jobs.
    pid = record['pid']
    try:
        if os.getpgid(pid) == pid:
            os.killpg(pid, signal.SIGINT)
        else:
            os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    for _ in range(40):
        active = running()
        if active is None or active['pid'] != pid:
            if active is None:
                STATE.unlink(missing_ok=True)
            if not quiet:
                print('Thermal preview stopped; sensor released.')
            return True
        time.sleep(.1)
    raise RuntimeError('Preview is still shutting down; wait before starting a task')


def public_ip():
    peers = os.getenv('SSH_CLIENT', '').split()
    peer = peers[0] if peers else '192.168.67.23'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.connect((peer, 9))  # Route lookup only; sends no datagram.
            return probe.getsockname()[0]
        except OSError:
            return socket.gethostbyname(socket.gethostname())


def start_preview():
    existing = running()
    if existing:
        print('PREVIEW_URL: ' + existing['url'])
        return
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(('0.0.0.0', 8090))
        except OSError as error:
            raise RuntimeError('Port 8090 is busy; stop the existing foreground preview first') from error
    root = VIEWER.parent.parent
    candidates = [root/'.venv-task6/bin/python', root/'venv/bin/python',
                  root/'.venv/bin/python', VIEWER.parent/'.venv/bin/python']
    interpreter = next((str(path) for path in candidates if path.is_file()), sys.executable)
    logs = VIEWER.parent / 'logs'
    logs.mkdir(exist_ok=True)
    log_path = logs / ('thermal_preview_' + str(time.time_ns()) + '.log')
    with log_path.open('wb') as log:
        os.chmod(log_path, 0o600)
        child = subprocess.Popen([interpreter, '-u', str(VIEWER), '--host', '0.0.0.0', '--port', '8090'],
            cwd=root, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        for _ in range(50):
            text = log_path.read_text(errors='replace')
            if child.poll() is not None:
                raise RuntimeError('Preview could not start: ' + text[-2000:])
            match = re.search(r'THERMAL_PREVIEW_READY (http://[^\s]+)', text)
            if match:
                loopback = match[1].replace('0.0.0.0', '127.0.0.1', 1)
                try:
                    with urllib.request.urlopen(loopback, timeout=.5) as response:
                        if response.status != 200:
                            continue
                except (urllib.error.URLError, TimeoutError):
                    time.sleep(.1)
                    continue
                url = match[1].replace('0.0.0.0', public_ip(), 1)
                record = dict(pid=child.pid, script=str(VIEWER), start_time=start_time(child.pid),
                              url=url, log=str(log_path))
                STATE.parent.mkdir(parents=True, exist_ok=True)
                temporary = STATE.with_suffix('.tmp')
                temporary.write_text(json.dumps(record))
                os.chmod(temporary, 0o600)
                temporary.replace(STATE)
                print('PREVIEW_URL: ' + url)
                print('Preview runs in background; starting the normal task receiver will close it automatically.')
                return
            time.sleep(.1)
        raise RuntimeError('Preview did not become ready; see ' + str(log_path))
    except BaseException:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=5)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    args = parser.parse_args()
    if args.action == 'start':
        start_preview()
    elif args.action == 'stop':
        if not stop_preview_service():
            print('No managed thermal preview is running.')
    else:
        record = running()
        print(json.dumps(record) if record else 'No managed thermal preview is running.')


if __name__ == '__main__':
    main()
