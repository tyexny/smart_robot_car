"""Load the saved Pi-to-PC credentials without executing shell configuration."""
import argparse
import json
import os
from pathlib import Path
import shlex
import sys
import urllib.error
import urllib.parse
import urllib.request


CONFIG = Path.home() / '.config' / 'smart_robot_car' / 'task6.env'
KEYS = {'TASK6_PC_URL', 'TASK6_TOKEN'}


def load_config(path=None, environ=None, report=None):
    """The saved pair is authoritative; an old shell export cannot split it."""
    path = CONFIG if path is None else Path(path)
    environ = os.environ if environ is None else environ
    report = (lambda message: print(message, file=sys.stderr)) if report is None else report
    if path.is_file():
        saved = {}
        for line in path.read_text(encoding='utf-8-sig').splitlines():
            try:
                fields = shlex.split(line, comments=True)
            except ValueError:
                raise ValueError('Invalid saved task6 configuration; credential values omitted') from None
            if fields[:1] == ['export']:
                fields = fields[1:]
            if not fields:
                continue
            name, separator, value = fields[0].partition('=')
            if name not in KEYS:
                continue
            if len(fields) != 1 or not separator or not value:
                raise ValueError('Invalid saved task6 configuration; credential values omitted')
            saved[name] = value
        if saved.keys() != KEYS:
            raise ValueError('Saved task6 configuration must contain both TASK6_PC_URL and TASK6_TOKEN')
        changed = any(environ.get(key) and environ[key] != value for key, value in saved.items())
        environ.update(saved)
        report('Task6 configuration: saved Pi file' + ('; replaced stale shell settings' if changed else ''))
    else:
        report('Task6 configuration: shell environment (no saved Pi file)')
    if any(not environ.get(key, '').strip() for key in KEYS):
        raise ValueError('Configure TASK6_PC_URL and TASK6_TOKEN; no default token is used')
    url = urllib.parse.urlsplit(environ['TASK6_PC_URL'])
    if url.scheme not in ('http', 'https') or not url.hostname or url.username or url.password or url.query or url.fragment:
        raise ValueError('TASK6_PC_URL must be an HTTP(S) server address without credentials or query')
    return environ


def check_pc(environ, mode, opener=urllib.request.urlopen):
    """Health GET only: never submit a task, invoke inference or move the car."""
    request = urllib.request.Request(environ['TASK6_PC_URL'].rstrip('/') + '/health',
                                    headers={'X-Task6-Token': environ['TASK6_TOKEN']})
    try:
        with opener(request, timeout=3) as response:
            health = json.loads(response.read(4096))
    except urllib.error.HTTPError as error:
        if error.code == 403:
            raise ValueError('PC authentication failed (403): synchronize TASK6_TOKEN, then restart both programs') from None
        raise ValueError('PC health request failed: HTTP ' + str(error.code)) from None
    except (OSError, ValueError, urllib.error.URLError):
        raise ValueError('PC health unavailable: check the PC service, URL, port and firewall') from None
    if health.get('mock') is not False:
        raise ValueError('PC is not a real inference service; hardware tasks remain disabled')
    if health.get('mode') != mode:
        raise ValueError('PC/Pi mode mismatch: PC=' + str(health.get('mode')) + ', Pi=' + mode)
    print('PC health OK; authentication OK; mode=' + mode)
    return health


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='authenticated health GET, without starting a receiver')
    parser.add_argument('--mode', choices=['target', 'vision', 'yolo'], default='target')
    parser.add_argument('--receiver', action='store_true', help='pass the loaded PC URL to the receiver')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    try:
        load_config()
        if args.check:
            check_pc(os.environ, args.mode)
            return
        command = args.command[1:] if args.command[:1] == ['--'] else args.command
        if not command:
            parser.error('Pass --check or a command after --')
        if args.receiver:
            command += ['--task6-pc-url', os.environ['TASK6_PC_URL']]
        os.execvpe(command[0], command, os.environ)
    except ValueError as error:
        parser.exit(1, str(error) + '\n')


if __name__ == '__main__':
    main()
