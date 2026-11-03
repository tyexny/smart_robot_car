"""Standalone thermal viewer: python thermal_viewer.py, then open localhost:8090."""
import argparse
import os
from pathlib import Path
import secrets

from thermal_monitor import ThermalMonitor
from thermal_preview import register_thermal_preview


class PreviewOnly:
    def sendto(self, *_):
        pass  # A standalone preview sends no packets or commands to the FPGA.


def receiver_running():
    root = Path('/proc')
    if not root.is_dir():
        return False
    for item in root.iterdir():
        if not item.name.isdecimal():
            continue
        try:
            args = (item / 'cmdline').read_bytes().split(b'\0')
            if any(arg.endswith(b'/udp_yolo_receiver.py') or arg == b'udp_yolo_receiver.py' for arg in args):
                return True
        except OSError:
            continue
    return False


def preview_app(monitor, token):
    from flask import Flask, abort, redirect, request
    app = Flask(__name__)
    register_thermal_preview(app, token, monitor.frame_snapshot, standalone=True)
    @app.get('/')
    def root():
        if request.remote_addr not in ('127.0.0.1', '::1'):
            abort(404)  # LAN viewers use the complete authorized link.
        return redirect('/' + token + '/thermal')
    @app.get('/' + token)
    def home():
        return redirect('/' + token + '/thermal')
    return app


def preview_token():
    path = Path(__file__).resolve().with_name('.thermal_preview_token')
    try:
        token = path.read_text().strip()
        if len(token) >= 20 and all(character.isalnum() or character in '-_' for character in token):
            return token
    except FileNotFoundError:
        pass
    token = secrets.token_urlsafe(18)
    path.write_text(token + '\n')
    os.chmod(path, 0o600)
    return token


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args(argv)
    if receiver_running():
        parser.exit(1, 'Task receiver is running. Use its task page: click 查看热成像, instead of opening a second sensor reader.\n')
    monitor = ThermalMonitor(PreviewOnly(), None, lambda: False)
    token = preview_token()
    app = preview_app(monitor, token)
    print('THERMAL_PREVIEW_READY http://%s:%d/%s/thermal' % (args.host, args.port, token), flush=True)
    print('On this Raspberry Pi, open http://localhost:%d . Ctrl+C stops preview.' % args.port, flush=True)
    print('Stop standalone preview before starting the normal task receiver.', flush=True)
    monitor.start()
    try:
        app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)
    finally:
        monitor.stop()


if __name__ == '__main__':
    main()
