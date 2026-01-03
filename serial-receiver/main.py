#!/usr/bin/env python3
"""
Serial receiver for rocket telemetry data (ESP-NOW version).
Listens on a USB serial port connected to the ESP-NOW ground station.
Appends telemetry CSV data to a file and displays log messages.
Provides an interactive command prompt to send control commands
to the rocket via the ground station.
"""

import argparse
import sys
import readline
import threading
from pathlib import Path
from datetime import datetime
import serial


# Global flag for graceful shutdown
running = True
telemetry_lock = threading.Lock()
download_mode = False
download_buffer = bytearray()


class CommandProcessor:
    """Processes and maps user commands to ESP-NOW protocol format."""

    def __init__(self):
        self.commands = {
            'start': 'START',
            'stop': 'STOP',
            'recalibrate': 'RECALIBRATE',
            'stats': 'STATS',
            'download': 'DOWNLOAD',
        }

    def get_command_list(self):
        """Returns list of available commands."""
        return sorted(self.commands.keys())

    def parse_command(self, user_input):
        """Parse user input and return command string."""
        cmd = user_input.strip().lower()

        if not cmd:
            return None

        # Handle help
        if cmd in ['help', '?']:
            return None, self.get_help()

        # Handle predefined commands (case-insensitive)
        if cmd in self.commands:
            return self.commands[cmd]

        return None, f"Unknown command: {cmd}"

    def get_help(self):
        """Return help message."""
        help_text = [
            "\nAvailable commands:",
            "  start        - Start telemetry transmission",
            "  stop         - Stop telemetry transmission",
            "  recalibrate  - Recalibrate altimeter baseline",
            "  download     - Download flight log from rocket",
            "  stats        - Show ground station statistics",
            "",
            "Special commands:",
            "  help, ?      - Show this help",
            "  quit, exit   - Exit the program",
            "",
            "Examples:",
            "  start",
            "  stop",
            "  recalibrate",
            "  download",
            "  stats",
        ]
        return "\n".join(help_text)


def telemetry_reader(ser, csv_path):
    """Background thread to read telemetry data from serial."""
    global running, download_mode, download_buffer

    packet_count = 0

    # Check if file is new and needs header
    file_is_new = not csv_path.exists() or csv_path.stat().st_size == 0

    with open(csv_path, 'a', encoding='utf-8') as csv_file:
        # Write CSV header if file is new
        if file_is_new:
            csv_file.write("timestamp,temperature,pressure,altitude,humidity,rssi\n")
            csv_file.flush()
        while running:
            try:
                # Read line from serial port
                if ser.in_waiting > 0:
                    line = ser.readline()

                    if line:
                        # Try to decode as text first
                        try:
                            data = line.decode('utf-8', errors='strict').strip()

                            if data.startswith('DATA: '):
                                # Extract CSV data after "DATA: " prefix
                                csv_data = data[6:]  # Skip "DATA: " prefix

                                # Write telemetry to CSV file
                                csv_file.write(csv_data + '\n')
                                csv_file.flush()

                                packet_count += 1
                                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                                # Log telemetry to stdout
                                with telemetry_lock:
                                    print(f"\r\033[K[{timestamp}] Packet #{packet_count}: {csv_data}")
                                    print("rocket> ", end='', flush=True)

                            elif data.startswith('LOG: '):
                                # Print log messages from ground station
                                log_msg = data[5:]  # Skip "LOG: " prefix

                                # Check if download is complete
                                if "File download complete" in log_msg and download_mode:
                                    download_mode = False

                                    # Save downloaded file
                                    download_path = csv_path.parent / f"downloaded_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                                    with open(download_path, 'wb') as f:
                                        f.write(download_buffer)

                                    with telemetry_lock:
                                        print(f"\r\033[K[GS] {log_msg}")
                                        print(f"\r\033[K[DOWNLOAD] Saved to: {download_path}")
                                        print(f"\r\033[K[DOWNLOAD] Size: {len(download_buffer)} bytes")
                                        print("rocket> ", end='', flush=True)

                                    download_buffer.clear()
                                else:
                                    with telemetry_lock:
                                        print(f"\r\033[K[GS] {log_msg}")
                                        print("rocket> ", end='', flush=True)

                            elif data:
                                # Print any other output (shouldn't happen normally)
                                with telemetry_lock:
                                    print(f"\r\033[K{data}")
                                    print("rocket> ", end='', flush=True)

                        except UnicodeDecodeError:
                            # Binary data - likely file download chunk
                            if download_mode:
                                download_buffer.extend(line)

            except Exception as e:
                if running:
                    with telemetry_lock:
                        print(f"\r\033[KWarning: Telemetry reader error: {e}")
                        print("rocket> ", end='', flush=True)


def main():
    global running

    parser = argparse.ArgumentParser(
        description="Interactive rocket telemetry receiver and controller (ESP-NOW)"
    )
    parser.add_argument(
        "serial_device",
        help="USB serial device path (e.g., /dev/ttyUSB0 or COM3)"
    )
    parser.add_argument(
        "csv_file",
        help="Path to CSV file for logging telemetry data"
    )
    parser.add_argument(
        "-b", "--baudrate",
        type=int,
        default=115200,
        help="Serial port baud rate (default: 115200)"
    )
    parser.add_argument(
        "-t", "--timeout",
        type=float,
        default=0.1,
        help="Serial port read timeout in seconds (default: 0.1)"
    )

    args = parser.parse_args()

    # Ensure CSV file path is valid
    csv_path = Path(args.csv_file)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Rocket Telemetry Controller (ESP-NOW)")
    print("=" * 60)
    print(f"Serial device: {args.serial_device}")
    print(f"CSV output file: {csv_path.absolute()}")
    print(f"Baud rate: {args.baudrate}")
    print(f"Protocol: ESP-NOW via ground station")
    print("-" * 60)

    ser = None
    reader_thread = None

    try:
        # Open serial port
        ser = serial.Serial(
            port=args.serial_device,
            baudrate=args.baudrate,
            timeout=args.timeout,
            write_timeout=1.0
        )
        print(f"Successfully opened serial port: {ser.name}")
        print("")

        # Start telemetry reader thread
        reader_thread = threading.Thread(
            target=telemetry_reader,
            args=(ser, csv_path),
            daemon=True
        )
        reader_thread.start()

        # Initialize command processor
        cmd_processor = CommandProcessor()

        # Configure readline for command history
        readline.parse_and_bind('tab: complete')
        readline.set_completer_delims(' \t\n')

        print("Type 'help' for available commands, 'quit' to exit")
        print("-" * 60)

        # Interactive command loop
        while running:
            try:
                user_input = input("rocket> ").strip()

                if not user_input:
                    continue

                # Handle quit/exit
                if user_input.lower() in ['quit', 'exit']:
                    print("Exiting...")
                    running = False
                    break

                # Parse and execute command
                result = cmd_processor.parse_command(user_input)

                if isinstance(result, tuple):
                    # Error case: (None, error_message)
                    _, message = result
                    print(message)
                elif result:
                    # Valid command - send to ground station
                    command_str = result + '\n'
                    ser.write(command_str.encode('utf-8'))
                    ser.flush()

                    # Enable download mode if this is a download command
                    if result == 'DOWNLOAD':
                        download_mode = True
                        download_buffer.clear()
                        print("[DOWNLOAD] Download initiated, waiting for data...")
                else:
                    # Empty result
                    print("Invalid command. Type 'help' for usage.")

            except EOFError:
                # Ctrl+D pressed
                print("\nExiting...")
                running = False
                break

    except serial.SerialException as e:
        print(f"Error: Failed to open serial port {args.serial_device}", file=sys.stderr)
        print(f"Details: {e}", file=sys.stderr)
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n" + "-" * 60)
        print("Stopped by user (Ctrl+C)")
        running = False

    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        running = False
        sys.exit(1)

    finally:
        running = False

        # Wait for reader thread to finish
        if reader_thread and reader_thread.is_alive():
            reader_thread.join(timeout=1.0)

        # Close serial port
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")


if __name__ == "__main__":
    main()
