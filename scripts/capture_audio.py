#!/usr/bin/env python3
"""
Capture PDM audio data from VoLoRa serial console and save as WAV file.

The firmware outputs base64-encoded PCM blocks (one line per 100ms block).

Usage:
    python3 capture_audio.py /dev/ttyACM0 output.wav
"""

import base64
import sys
import wave

import serial


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <serial_port> <output.wav>")
        sys.exit(1)

    port = sys.argv[1]
    output_file = sys.argv[2]

    print(f"Opening {port}...")
    ser = serial.Serial(port, timeout=30)

    # Wait for PDM_START header
    print("Waiting for recording to start...")
    sample_rate = 16000
    bit_width = 16
    channels = 1

    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("<<PDM_START"):
            # Parse parameters from header
            for part in line.replace("<<PDM_START", "").replace(">>", "").split():
                key, val = part.split("=")
                if key == "rate":
                    sample_rate = int(val)
                elif key == "bits":
                    bit_width = int(val)
                elif key == "channels":
                    channels = int(val)
            print(f"Recording: {sample_rate} Hz, {bit_width}-bit, {channels} ch")
            break
        elif line:
            print(f"  [{line}]")

    # Collect base64-encoded audio blocks
    raw_data = bytearray()
    block_count = 0

    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line == "<<PDM_END>>":
            break
        if not line:
            continue
        # Skip log lines
        if line.startswith("[") or line.startswith("<"):
            continue
        try:
            raw_data.extend(base64.b64decode(line))
            block_count += 1
            print(f"\r  Blocks received: {block_count}", end="", flush=True)
        except Exception:
            continue

    ser.close()
    print()

    num_samples = len(raw_data) // (bit_width // 8)
    duration = num_samples / sample_rate
    print(f"Captured {num_samples} samples ({duration:.2f}s)")

    if not raw_data:
        print("No samples captured!")
        sys.exit(1)

    # Normalize audio to use full 16-bit range
    import struct

    import numpy as np

    samples = np.frombuffer(raw_data, dtype=np.int16).copy()
    peak = max(abs(samples.min()), abs(samples.max()))
    if peak > 0:
        # Normalize to -3 dBFS (leave some headroom)
        target = int(32767 * 0.707)
        gain = target / peak
        samples = np.clip(samples * gain, -32768, 32767).astype(np.int16)
        print(f"Normalized: peak {peak} -> {target} (gain {gain:.1f}x / {20*np.log10(gain):.1f} dB)")

    # Write WAV file
    with wave.open(output_file, "w") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(bit_width // 8)
        wf.setframerate(sample_rate)
        wf.writeframes(samples.tobytes())

    print(f"Saved to {output_file}")


if __name__ == "__main__":
    main()
