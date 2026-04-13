"""Lightweight HTTP server for the Cesium 3D drone viewer.

Serves two endpoints:
  GET /       — The CesiumJS HTML viewer page (cached in memory)
  GET /state  — Current drone state JSON (read from disk each request)

Launched by cesium_bridge.m as a background process.
Usage:
  python cesium_server.py --port 8765 --html viewer.html --state state.json
"""

import argparse
import os
import sys
from http.server import HTTPServer, BaseHTTPRequestHandler

# Globals set from CLI args
HTML_BYTES = b''
STATE_PATH = ''


class Handler(BaseHTTPRequestHandler):
    """Minimal request handler — no directory listing, no logging spam."""

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self._respond(200, 'text/html; charset=utf-8', HTML_BYTES)
        elif self.path == '/state':
            self._serve_state()
        else:
            self._respond(404, 'text/plain', b'Not Found')

    def _serve_state(self):
        try:
            with open(STATE_PATH, 'rb') as f:
                body = f.read()
        except Exception:
            body = b'{"error":"loading"}'
        self._respond(200, 'application/json; charset=utf-8', body)

    def _respond(self, code, content_type, body):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache, no-store')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        """Suppress default stderr logging to avoid console spam."""
        pass


def main():
    global HTML_BYTES, STATE_PATH

    parser = argparse.ArgumentParser(description='Cesium drone viewer server')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--html', required=True, help='Path to cesium_viewer.html')
    parser.add_argument('--state', required=True, help='Path to state JSON file')
    args = parser.parse_args()

    STATE_PATH = args.state

    if not os.path.isfile(args.html):
        print(f'ERROR: HTML file not found: {args.html}', file=sys.stderr)
        sys.exit(1)

    with open(args.html, 'rb') as f:
        HTML_BYTES = f.read()

    server = HTTPServer(('127.0.0.1', args.port), Handler)
    print(f'Cesium server listening on http://localhost:{args.port}')
    sys.stdout.flush()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
