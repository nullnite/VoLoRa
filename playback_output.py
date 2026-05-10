import pylink
import time
import numpy as np
import sounddevice
import queue
import threading

NSAM = 160
RATE = 8000
FRAME_LEN = NSAM * 2  # 320 bytes of s16-le PCM
HEADER_LEN = 6  # 0xFF 0xFE seq len_lo len_hi xor_chk
MARKER = b"\xff\xfe"

audio_q = queue.Queue(maxsize=30)  # 30 frames = 600 ms max latency
stats = {
    "frames": 0,
    "underruns": 0,
    "overruns": 0,
    "bad_len": 0,
    "bad_chk": 0,
    "bad_seq": 0,
}
expected_seq = None


def audio_callback(outdata, frames, time_info, status):
    try:
        data = audio_q.get_nowait()
        outdata[:, 0] = data[:frames]
    except queue.Empty:
        outdata[:] = 0
        stats["underruns"] += 1


def parse_frames(buf):
    """Extract PCM frames from a mixed RTT stream; print everything else as text."""
    global expected_seq
    frames = []
    while True:
        idx = buf.find(MARKER)
        if idx == -1:
            # No marker — print printable tail as log text, keep last bytes as
            # potential split-marker guard
            tail = HEADER_LEN + FRAME_LEN
            if len(buf) > tail:
                _print_text(buf[:-tail])
                buf = buf[-tail:]
            break

        # Print any log text that precedes this frame
        if idx > 0:
            _print_text(buf[:idx])

        if len(buf) < idx + HEADER_LEN + FRAME_LEN:
            buf = buf[idx:]  # incomplete frame — wait for more data
            break

        seq = buf[idx + 2]
        ln = buf[idx + 3] | (buf[idx + 4] << 8)
        chk = buf[idx + 5]

        if ln != FRAME_LEN:
            stats["bad_len"] += 1
            buf = buf[idx + 1 :]
            continue

        payload = buf[idx + HEADER_LEN : idx + HEADER_LEN + FRAME_LEN]

        computed = 0
        for b in payload:
            computed ^= b
        if computed != chk:
            stats["bad_chk"] += 1
            buf = buf[idx + 1 :]
            continue

        if expected_seq is not None and seq != expected_seq:
            stats["bad_seq"] += 1
        expected_seq = (seq + 1) & 0xFF

        samples = (
            np.frombuffer(bytes(payload), dtype=np.int16).astype(np.float32) / 32768.0
        )
        frames.append(samples)
        stats["frames"] += 1
        buf = buf[idx + HEADER_LEN + FRAME_LEN :]

    return buf, frames


def _print_text(data):
    """Print bytes that look like UTF-8 log output."""
    try:
        print(bytes(data).decode("utf-8", errors="replace"), end="", flush=True)
    except Exception:
        pass


def connect():
    jlink = pylink.JLink()
    jlink.open()
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect("nRF5340_xxAA")
    jlink.rtt_start()
    time.sleep(0.5)
    print("[connected]")
    return jlink


def capture():
    jlink = None
    buf = bytearray()
    while True:
        try:
            if jlink is None:
                jlink = connect()

            data = jlink.rtt_read(0, 1024)
            if data:
                buf += bytes(data)
                buf, frames = parse_frames(buf)
                for f in frames:
                    try:
                        audio_q.put_nowait(f)
                    except queue.Full:
                        audio_q.get_nowait()
                        audio_q.put_nowait(f)
                        stats["overruns"] += 1
            time.sleep(0.05)

        except Exception as e:
            print(f"\n[disconnected: {e}, retrying...]")
            try:
                jlink.close()
            except Exception:
                pass
            jlink = None
            buf = bytearray()
            time.sleep(2)


def play():
    print(sounddevice.query_devices())
    try:
        with sounddevice.OutputStream(
            samplerate=RATE,
            channels=1,
            dtype="float32",
            blocksize=NSAM,
            callback=audio_callback,
            latency=0.05,
        ) as stream:
            print(f"Stream opened: {stream.samplerate}Hz latency={stream.latency:.3f}s")
            last = stats.copy()
            while True:
                time.sleep(2)
                d = {k: stats[k] - last[k] for k in stats}
                last = stats.copy()
                print(
                    f"Queue:{audio_q.qsize():3d}  Frames:{d['frames']}  "
                    f"Underruns:{d['underruns']}  Overruns:{d['overruns']}  "
                    f"bad_len:{d['bad_len']}  bad_chk:{d['bad_chk']}  bad_seq:{d['bad_seq']}"
                )
    except Exception as e:
        print(f"Stream error: {e}")
        import traceback

        traceback.print_exc()


threading.Thread(target=capture, daemon=True).start()

try:
    play()
except KeyboardInterrupt:
    print("\nDone")
