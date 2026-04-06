#!/usr/bin/env python3
"""
receive_samples.py

MeganeMouse Sound Click — Phase 1: PC-side Sample Receiver

Listens on a serial port for WAV data sent by SoundSampleCollector.ino
and saves each sample as a properly organized .wav file.

Usage:
    python receive_samples.py --port /dev/ttyACM0
    python receive_samples.py --port COM3 --output ./my_samples
    python receive_samples.py --port /dev/ttyACM0 --list-ports

The script creates a folder structure like:
    samples/
        click_left/
            click_left.0001.wav
            click_left.0002.wav
        click_right/
            click_right.0001.wav
        noise/
            noise.0001.wav

Requirements:
    pip install pyserial

License: MIT
"""

import argparse
import base64
import os
import sys
import time
import re
from datetime import datetime
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial is required. Install it with:")
    print("  pip install pyserial")
    sys.exit(1)


# ============================================================
# Configuration
# ============================================================

DEFAULT_BAUD = 115200
DEFAULT_OUTPUT_DIR = "samples"

START_MARKER = ">>>WAV_START:"
END_MARKER = "<<<WAV_END"


# ============================================================
# Serial port utilities
# ============================================================

def list_serial_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return

    print("\nAvailable serial ports:")
    print("-" * 60)
    for port in sorted(ports):
        print(f"  {port.device:20s}  {port.description}")
    print()


def open_serial(port, baud):
    """Open a serial connection with appropriate settings."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=1.0,
            write_timeout=1.0,
        )
        # Give the device time to reset after connection
        time.sleep(2.0)
        # Flush any startup messages
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        sys.exit(1)


# ============================================================
# WAV file handling
# ============================================================

def validate_wav(data):
    """Basic validation that the data looks like a WAV file."""
    if len(data) < 44:
        return False, "Data too short for WAV header"
    if data[0:4] != b'RIFF':
        return False, "Missing RIFF header"
    if data[8:12] != b'WAVE':
        return False, "Missing WAVE marker"
    if data[12:16] != b'fmt ':
        return False, "Missing fmt chunk"
    return True, "OK"


def get_wav_info(data):
    """Extract WAV file information from header."""
    if len(data) < 44:
        return {}

    sample_rate = int.from_bytes(data[24:28], 'little')
    bits_per_sample = int.from_bytes(data[34:36], 'little')
    num_channels = int.from_bytes(data[22:24], 'little')
    data_size = int.from_bytes(data[40:44], 'little')
    duration_ms = (data_size * 1000) // (sample_rate * num_channels * bits_per_sample // 8)

    return {
        "sample_rate": sample_rate,
        "bits_per_sample": bits_per_sample,
        "channels": num_channels,
        "data_size": data_size,
        "duration_ms": duration_ms,
    }


# ============================================================
# Sample receiver
# ============================================================

def get_next_index(class_dir, class_name):
    """
    Finds the maximum index among existing files in the target directory
    and returns the next available index. 
    This safely handles cases where intermediate files have been deleted 
    or unrelated files exist in the same directory.
    """
    max_idx = -1
    # Regex to match files like: class_name.0012.wav
    pattern = re.compile(rf"^{re.escape(class_name)}\.(\d+)\.wav$")
    
    if not class_dir.exists():
        return 0

    for filepath in class_dir.iterdir():
        if filepath.is_file():
            match = pattern.match(filepath.name)
            if match:
                idx = int(match.group(1))
                if idx > max_idx:
                    max_idx = idx
                    
    return max_idx + 1


def save_sample(wav_data, class_name, output_dir):
    """
    Save WAV data to a file in the appropriate class folder 
    with a safely generated sequential index.
    """
    class_dir = Path(output_dir) / class_name
    class_dir.mkdir(parents=True, exist_ok=True)

    # Generate a safe sequential index based on existing files on the PC,
    # ignoring any index sequence provided by the Arduino.
    next_index = get_next_index(class_dir, class_name)

    filename = f"{class_name}.{next_index:04d}.wav"
    filepath = class_dir / filename
    filepath.write_bytes(wav_data)

    return filepath


def receive_one_sample(ser):
    """
    Wait for and receive one WAV sample from serial.

    Returns:
        tuple: (class_name, index, wav_data) or (None, None, None) on error
    """
    # Wait for start marker
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace').strip()
        except serial.SerialException:
            return None, None, None

        if not line:
            continue

        # Print non-protocol messages for debugging
        if not line.startswith(">>>") and not line.startswith("<<<"):
            if line.startswith("CLASS_CHANGED:"):
                class_name = line.split(":")[1]
                print(f"  [Device] Class changed to: {class_name}")
            elif line.startswith("GAIN_CHANGED:"):
                gain = line.split(":")[1]
                print(f"  [Device] Mic gain changed to: {gain}")
            elif line.startswith("STATUS:"):
                parts = line.split(":")
                if len(parts) >= 4:
                    print(f"  [Device] Status — Class: {parts[1]}, "
                          f"Count: {parts[2]}, Gain: {parts[3]}")
            else:
                print(f"  [Device] {line}")
            continue

        if line.startswith(START_MARKER):
            # Parse: >>>WAV_START:class_name:index
            parts = line[len(START_MARKER):].split(":")
            if len(parts) >= 2:
                class_name = parts[0]
                try:
                    index = int(parts[1])
                except ValueError:
                    index = 0
                break

    # Receive base64 data lines
    print(f"  Receiving: {class_name} #{index:04d} ...", end="", flush=True)

    wav_data = bytearray()
    line_count = 0

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace').strip()
        except serial.SerialException:
            print(" ERROR: serial connection lost")
            return None, None, None

        if line == END_MARKER:
            break

        if line:
            try:
                wav_data.extend(base64.b64decode(line))
                line_count += 1
            except Exception as e:
                print(f" ERROR: base64 decode failed on line: {e}")
                return None, None, None

    print(f" {len(wav_data)} bytes", end="")

    # Validate WAV
    valid, msg = validate_wav(wav_data)
    if not valid:
        print(f" ERROR: invalid WAV — {msg}")
        return None, None, None

    info = get_wav_info(wav_data)
    print(f" ({info.get('duration_ms', '?')}ms, "
          f"{info.get('sample_rate', '?')}Hz)")

    return class_name, index, wav_data


def run_receiver(port, baud, output_dir, label):
    """Main receiver loop."""
    print(f"Opening serial port: {port} @ {baud} baud")
    ser = open_serial(port, baud)

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    print(f"Output directory: {output_path.resolve()}")
    print()
    print("=" * 55)
    print(" MeganeMouse Sound Sample Collector — PC Receiver")
    print("=" * 55)
    print()
    print("Waiting for samples from device...")
    print("(Press the button on AtomS3R to record)")
    print("(Press Ctrl+C to stop)")
    print()

    # Track totals per class
    class_totals = {}
    total_count = 0

    try:
        while True:
            class_name, index, wav_data = receive_one_sample(ser)

            if wav_data is None:
                continue

            # Save the file
            filepath = save_sample(wav_data, label, output_dir)

            # Update counts
            class_totals[class_name] = class_totals.get(class_name, 0) + 1
            total_count += 1

            print(f"  Saved: {filepath}")

            # Print summary every 10 samples
            if total_count % 10 == 0:
                print()
                print(f"  --- Progress ({total_count} total) ---")
                for cn, ct in sorted(class_totals.items()):
                    print(f"    {cn}: {ct} samples")
                print()

    except KeyboardInterrupt:
        print()
        print()
        print("=" * 55)
        print(" Collection complete!")
        print("=" * 55)
        print()
        print(f"Total samples: {total_count}")
        for cn, ct in sorted(class_totals.items()):
            print(f"  {cn}: {ct} samples")
        print()
        print(f"Files saved in: {output_path.resolve()}")
        print()
        print("Next steps:")
        print("  1. Upload the samples/ folder to Edge Impulse")
        print("  2. Each subfolder name becomes the class label")
        print()

    finally:
        ser.close()


# ============================================================
# CLI: remote commands
# ============================================================

def send_command(port, baud, command):
    """Send a single command to the device."""
    ser = open_serial(port, baud)

    ser.write(f"{command}\n".encode())
    time.sleep(0.5)

    # Read response
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            print(f"  {line}")

    ser.close()


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="MeganeMouse Sound Sample Collector — PC Receiver",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --port /dev/ttyACM0 --label click          # Linux
  %(prog)s --port /dev/cu.usbmodem* --label noise     # macOS
  %(prog)s --port COM3 --label click                  # Windows
  %(prog)s --port COM3 --label click --output custom  # Custom output folder
  %(prog)s --list-ports                               # Show available ports
        """
    )

    parser.add_argument(
        "--port", "-p",
        help="Serial port (e.g., /dev/ttyACM0, COM3)"
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Baud rate (default: {DEFAULT_BAUD})"
    )
    parser.add_argument(
        "--label", "-L",
        help="Target class name for the audio (e.g., click, noise) [Required for receiving]"
    )
    parser.add_argument(
        "--output", "-o",
        default=DEFAULT_OUTPUT_DIR,
        help=f"Output directory (default: {DEFAULT_OUTPUT_DIR})"
    )
    parser.add_argument(
        "--list-ports", "-l",
        action="store_true",
        help="List available serial ports and exit"
    )
    parser.add_argument(
        "--command", "-c",
        choices=["REC", "STATUS"],
        help="Send a command to the device (REC=record, STATUS=query)"
    )

    args = parser.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    if not args.port:
        print("Error: --port is required. Use --list-ports to see available ports.")
        parser.print_help()
        sys.exit(1)

    if args.command:
        send_command(args.port, args.baud, args.command)
    else:
        # Require --label when entering the normal receiving loop
        if not args.label:
            print("Error: --label is required for receiving samples.")
            parser.print_help()
            sys.exit(1)
            
        print("=========================================")
        print(" MeganeMouse Sound Sample Collector      ")
        print("=========================================")
        print(f" Port        : {args.port}")
        print(f" Target Label: {args.label}")
        print(f" Output Dir  : {args.output}")
        print("=========================================\n")
        
        # Pass args.label to run_receiver so it can be used in save_sample()
        run_receiver(args.port, args.baud, args.output, args.label)


if __name__ == "__main__":
    main()
