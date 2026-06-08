#!/usr/bin/env python3

import argparse
import socket
import struct
import sys
from urllib.parse import urlparse


ADDR_SOCKET_PROTOCOL_BASE = 34
DEFAULT_SERVICE_ID = 1


def update_crc(crc_accum, data):
    crc_table = [
        0x0000, 0x8005, 0x800f, 0x000a, 0x801b, 0x001e, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003c, 0x8039, 0x0028, 0x802d, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006c, 0x8069, 0x0078, 0x807d, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805f, 0x005a, 0x804b, 0x004e, 0x0044, 0x8041,
        0x80c3, 0x00c6, 0x00cc, 0x80c9, 0x00d8, 0x80dd, 0x80d7, 0x00d2,
        0x00f0, 0x80f5, 0x80ff, 0x00fa, 0x80eb, 0x00ee, 0x00e4, 0x80e1,
        0x00a0, 0x80a5, 0x80af, 0x00aa, 0x80bb, 0x00be, 0x00b4, 0x80b1,
        0x8093, 0x0096, 0x009c, 0x8099, 0x0088, 0x808d, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018c, 0x8189, 0x0198, 0x819d, 0x8197, 0x0192,
        0x01b0, 0x81b5, 0x81bf, 0x01ba, 0x81ab, 0x01ae, 0x01a4, 0x81a1,
        0x01e0, 0x81e5, 0x81ef, 0x01ea, 0x81fb, 0x01fe, 0x01f4, 0x81f1,
        0x81d3, 0x01d6, 0x01dc, 0x81d9, 0x01c8, 0x81cd, 0x81c7, 0x01c2,
        0x0140, 0x8145, 0x814f, 0x014a, 0x815b, 0x015e, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017c, 0x8179, 0x0168, 0x816d, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012c, 0x8129, 0x0138, 0x813d, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811f, 0x011a, 0x810b, 0x010e, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030c, 0x8309, 0x0318, 0x831d, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833f, 0x033a, 0x832b, 0x032e, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836f, 0x036a, 0x837b, 0x037e, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035c, 0x8359, 0x0348, 0x834d, 0x8347, 0x0342,
        0x03c0, 0x83c5, 0x83cf, 0x03ca, 0x83db, 0x03de, 0x03d4, 0x83d1,
        0x83f3, 0x03f6, 0x03fc, 0x83f9, 0x03e8, 0x83ed, 0x83e7, 0x03e2,
        0x83a3, 0x03a6, 0x03ac, 0x83a9, 0x03b8, 0x83bd, 0x83b7, 0x03b2,
        0x0390, 0x8395, 0x839f, 0x039a, 0x838b, 0x038e, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828f, 0x028a, 0x829b, 0x029e, 0x0294, 0x8291,
        0x82b3, 0x02b6, 0x02bc, 0x82b9, 0x02a8, 0x82ad, 0x82a7, 0x02a2,
        0x82e3, 0x02e6, 0x02ec, 0x82e9, 0x02f8, 0x82fd, 0x82f7, 0x02f2,
        0x02d0, 0x82d5, 0x82df, 0x02da, 0x82cb, 0x02ce, 0x02c4, 0x82c1,
        0x8243, 0x0246, 0x024c, 0x8249, 0x0258, 0x825d, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827f, 0x027a, 0x826b, 0x026e, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822f, 0x022a, 0x823b, 0x023e, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021c, 0x8219, 0x0208, 0x820d, 0x8207, 0x0202,
    ]
    for byte in data:
        i = ((crc_accum >> 8) ^ byte) & 0xFF
        crc_accum = ((crc_accum << 8) ^ crc_table[i]) & 0xFFFF
    return crc_accum


def make_packet(dxl_id, instruction, params=b''):
    length = len(params) + 3
    packet = bytearray([0xFF, 0xFF, 0xFD, 0x00, dxl_id])
    packet.extend(struct.pack('<H', length))
    packet.append(instruction)
    packet.extend(params)
    packet.extend(struct.pack('<H', update_crc(0, packet)))
    return bytes(packet)


def parse_service_port_name(value):
    if not value.startswith('e2d2svc:'):
        raise ValueError('service_port_name must use e2d2svc:<ip>:<port>')
    parsed = urlparse('udp://' + value[len('e2d2svc:'):])
    if not parsed.hostname or not parsed.port:
        raise ValueError('service_port_name must use e2d2svc:<ip>:<port>')
    return parsed.hostname, parsed.port


def read_status(sock, timeout):
    sock.settimeout(timeout)
    data = sock.recv(4096)
    if len(data) < 11 or data[:4] != b'\xff\xff\xfd\x00':
        raise RuntimeError(f'invalid status packet: {data.hex()}')
    crc_expected = struct.unpack('<H', data[-2:])[0]
    crc_actual = update_crc(0, data[:-2])
    if crc_expected != crc_actual:
        raise RuntimeError(
            f'bad status CRC: expected 0x{crc_expected:04x}, got 0x{crc_actual:04x}'
        )
    if data[7] != 0x55:
        raise RuntimeError(f'unexpected status instruction: 0x{data[7]:02x}')
    if data[8] != 0:
        raise RuntimeError(f'E2D2 returned packet error 0x{data[8]:02x}')
    return data


def tx_rx(sock, packet, timeout, retries):
    last_error = None
    for _ in range(retries):
        try:
            sock.send(packet)
            return read_status(sock, timeout)
        except (TimeoutError, socket.timeout, RuntimeError) as exc:
            last_error = exc
    raise RuntimeError(f'no valid status packet after {retries} attempts: {last_error}')


def ping(sock, dxl_id, timeout, retries):
    status = tx_rx(sock, make_packet(dxl_id, 0x01), timeout, retries)
    params = status[9:-2]
    if len(params) >= 3:
        model_number = params[0] | (params[1] << 8)
        firmware = params[2]
        print(f'E2D2 service ping OK: id={dxl_id}, model={model_number}, firmware={firmware}')
    else:
        print(f'E2D2 service ping OK: id={dxl_id}')


def write_socket_protocol(sock, dxl_id, channel, protocol, timeout, retries):
    address = ADDR_SOCKET_PROTOCOL_BASE + channel - 1
    params = struct.pack('<HB', address, protocol)
    tx_rx(sock, make_packet(dxl_id, 0x03, params), timeout, retries)
    label = 'UDP' if protocol == 1 else 'TCP'
    print(f'Socket {channel} Protocol <- {protocol} ({label})')


def parse_channels(value):
    channels = []
    for item in value.split(','):
        channel = int(item.strip())
        if channel < 1 or channel > 6:
            raise ValueError('channels must be in the range 1..6')
        channels.append(channel)
    return channels


def main():
    parser = argparse.ArgumentParser(
        description='Configure E2D2 bridge TCP/UDP protocol without an E2D2 model file.'
    )
    parser.add_argument('--service-port-name', default='e2d2svc:192.168.0.1:5008')
    parser.add_argument('--bridge-protocol', type=int, choices=[0, 1], required=True)
    parser.add_argument('--channels', default='1,2,3,4,5,6')
    parser.add_argument('--id', type=int, default=DEFAULT_SERVICE_ID)
    parser.add_argument('--timeout', type=float, default=0.25)
    parser.add_argument('--retries', type=int, default=5)
    args = parser.parse_args()

    ip, port = parse_service_port_name(args.service_port_name)
    channels = parse_channels(args.channels)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((ip, port))
        ping(sock, args.id, args.timeout, args.retries)
        for channel in channels:
            write_socket_protocol(
                sock, args.id, channel, args.bridge_protocol, args.timeout, args.retries
            )

    print('E2D2 bridge protocol configuration complete.')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(f'configure_e2d2_bridge_protocol failed: {exc}', file=sys.stderr)
        sys.exit(1)
