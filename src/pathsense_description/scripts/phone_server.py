#!/usr/bin/env python3
"""Serve the phone remote page (web/index.html) on port 8000 and print the address to open on the phone."""
import functools
import http.server
import os
import socket
import socketserver
import subprocess

from ament_index_python.packages import get_package_share_directory

PORT = 8000


def lan_ips():
    ips = set()
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True).stdout.split()
        ips.update(ip for ip in out if '.' in ip and not ip.startswith('127.'))
    except Exception:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))
        ips.add(s.getsockname()[0])
        s.close()
    except Exception:
        pass
    return sorted(ips) or ['<laptop-ip>']


def main():
    web = os.path.join(get_package_share_directory('pathsense_description'), 'web')
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=web)
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(('0.0.0.0', PORT), handler) as httpd:
        print('\n========================================================')
        print(' PathSense phone remote is running. On your phone open:')
        for ip in lan_ips():
            print(f'     http://{ip}:{PORT}')
        print(' (phone and laptop must be on the same Wi-Fi / hotspot)')
        print('========================================================\n', flush=True)
        httpd.serve_forever()


if __name__ == '__main__':
    main()
