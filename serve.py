#!/usr/bin/env python3
"""
Simple HTTP server with Range request support for audio/video seeking.
Usage: python serve.py [port]
"""

import os
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler
from functools import partial

class RangeRequestHandler(SimpleHTTPRequestHandler):
    """HTTP handler with Range request support for media seeking."""

    def send_head(self):
        """Handle Range requests for audio/video seeking."""
        path = self.translate_path(self.path)

        if not os.path.isfile(path):
            return super().send_head()

        # Check for Range header
        range_header = self.headers.get('Range')
        if range_header is None:
            return super().send_head()

        # Parse Range header (e.g., "bytes=0-1023")
        try:
            range_spec = range_header.replace('bytes=', '')
            start_str, end_str = range_spec.split('-')
            file_size = os.path.getsize(path)

            start = int(start_str) if start_str else 0
            end = int(end_str) if end_str else file_size - 1

            # Validate range
            if start >= file_size:
                self.send_error(416, 'Requested Range Not Satisfiable')
                return None

            end = min(end, file_size - 1)
            content_length = end - start + 1

            # Send partial content response
            f = open(path, 'rb')
            f.seek(start)

            self.send_response(206)  # Partial Content
            self.send_header('Content-Type', self.guess_type(path))
            self.send_header('Content-Length', str(content_length))
            self.send_header('Content-Range', f'bytes {start}-{end}/{file_size}')
            self.send_header('Accept-Ranges', 'bytes')
            self.end_headers()
            return f

        except (ValueError, IOError):
            return super().send_head()

    def do_GET(self):
        """Override to add Accept-Ranges header to all responses."""
        f = self.send_head()
        if f:
            try:
                # For range requests, only send the requested portion
                if self.headers.get('Range'):
                    range_spec = self.headers.get('Range').replace('bytes=', '')
                    start_str, end_str = range_spec.split('-')
                    file_size = os.path.getsize(self.translate_path(self.path))
                    start = int(start_str) if start_str else 0
                    end = int(end_str) if end_str else file_size - 1
                    content_length = end - start + 1
                    self.wfile.write(f.read(content_length))
                else:
                    self.copyfile(f, self.wfile)
            finally:
                f.close()

    def end_headers(self):
        """Add Accept-Ranges to all responses."""
        self.send_header('Accept-Ranges', 'bytes')
        super().end_headers()


def run(port=8889, directory=None):
    if directory:
        os.chdir(directory)

    handler = RangeRequestHandler
    server = HTTPServer(('', port), handler)
    print(f"Serving at http://localhost:{port}")
    print(f"Directory: {os.getcwd()}")
    print("Range requests enabled for audio/video seeking")
    print("Press Ctrl+C to stop")
    server.serve_forever()


if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8889
    run(port)
