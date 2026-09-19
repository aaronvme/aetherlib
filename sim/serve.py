#!/usr/bin/env python3
"""Serve sim/web on http://localhost:8080.

    python sim/serve.py [port]

Plain static hosting is enough -- the sim uses Asyncify rather than pthreads,
so no COOP/COEP headers are required. Caching is disabled so a rebuild shows up
on refresh instead of serving a stale .wasm.
"""
import http.server
import os
import socketserver
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        super().end_headers()

    def log_message(self, fmt, *args):
        if "GET" in (fmt % args):
            super().log_message(fmt, *args)


Handler.extensions_map.update({
    ".js": "text/javascript",
    ".mjs": "text/javascript",
    ".wasm": "application/wasm",
})

if __name__ == "__main__":
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"chad-sim  ->  http://localhost:{PORT}")
        print("Ctrl+C to stop")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print()
