"""
RoboBoat 2026 RoboCommand framing: b'$R' + 1 length byte + protobuf payload + b'!!'.

From RoboNation's TCP_library.py (github.com/robonation/robocommand, Apache 2.0): no
checksum; a payload is at most 255 bytes; a receiver that finds no footer where the length
says skips to the next header.
"""

HEADER = b'$R'
FOOTER = b'!!'
MAX_PAYLOAD = 255


def frame(payload):
    """Return one framed message; ValueError if the payload does not fit the length byte."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload {len(payload)} B: RoboCommand frames carry at most '
                         f'{MAX_PAYLOAD} B')
    return HEADER + bytes([len(payload)]) + payload + FOOTER


class Deframer:
    """Pull whole payloads out of a byte stream; junk between frames is skipped."""

    def __init__(self):
        self.buf = bytearray()
        self.skipped = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            start = self.buf.find(HEADER)
            if start < 0:
                keep = 1 if self.buf.endswith(HEADER[:1]) else 0
                self.skipped += len(self.buf) - keep
                del self.buf[:len(self.buf) - keep]
                return out
            if start:
                self.skipped += start
                del self.buf[:start]
            if len(self.buf) < 3:
                return out
            n = self.buf[2]
            end = 3 + n
            if len(self.buf) < end + 2:
                return out
            if self.buf[end:end + 2] != FOOTER:
                del self.buf[:1]                  # not a frame: look for the next header
                self.skipped += 1
                continue
            out.append(bytes(self.buf[3:end]))
            del self.buf[:end + 2]
