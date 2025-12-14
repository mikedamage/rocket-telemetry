#!/usr/bin/env python3
"""
Serial receiver for rocket telemetry data.
Listens on a USB serial port and appends CSV data to a file.
Also provides an interactive command prompt to send control commands
to the rocket via the ground station.
"""

import argparse
import sys
import json
import readline
import threading
from pathlib import Path
from datetime import datetime
import serial


# Global flag for graceful shutdown
running = True
telemetry_lock = threading.Lock()


class CommandProcessor:
    """Processes and maps user commands to serial protocol format."""

    def __init__(self):
        self.commands = {
            'status': ('GET', '/status', ''),
            'get_config': ('GET', '/config', ''),
            'get_telemetry': ('GET', '/telemetry', ''),
            'delete_telemetry': ('DELETE', '/telemetry', ''),
            'start': ('POST', '/command', '{"command": "start"}'),
            'stop': ('POST', '/command', '{"command": "stop"}'),
            'reboot': ('POST', '/command', '{"command": "reboot"}'),
            'reset': ('POST', '/command', '{"command": "reset"}'),
            'recalibrate_altimeter': ('POST', '/command', '{"command": "recalibrate_altimeter"}'),
            'start_beacon': ('POST', '/command', '{"command": "start_beacon"}'),
        }

    def get_command_list(self):
        """Returns list of available commands."""
        return sorted(self.commands.keys())

    def parse_command(self, user_input):
        """Parse user input and return serial protocol string."""
        parts = user_input.strip().split(maxsplit=1)
        if not parts:
            return None

        cmd = parts[0].lower()

        # Handle update_config specially (takes arguments)
        if cmd == 'update_config':
            if len(parts) < 2:
                return None, "Usage: update_config {json_config}"

            try:
                # Validate JSON
                json.loads(parts[1])
                return f"PUT /config '{parts[1]}'"
            except json.JSONDecodeError as e:
                return None, f"Invalid JSON: {e}"

        # Handle raw API call format: METHOD endpoint payload
        if cmd.upper() in ['GET', 'POST', 'PUT', 'DELETE']:
            if len(parts) < 2:
                return None, f"Usage: {cmd.upper()} /endpoint '{{\"json\":\"payload\"}}'"

            # Parse endpoint and optional payload
            rest = parts[1].strip()
            endpoint_parts = rest.split(maxsplit=1)
            endpoint = endpoint_parts[0]
            payload = endpoint_parts[1] if len(endpoint_parts) > 1 else "''"

            # Remove quotes if present
            if payload.startswith("'") and payload.endswith("'"):
                payload = payload[1:-1]

            # Validate JSON for POST/PUT
            if cmd.upper() in ['POST', 'PUT']:
                if not payload:
                    return None, f"{cmd.upper()} requires JSON payload"
                try:
                    json.loads(payload)
                except json.JSONDecodeError as e:
                    return None, f"Invalid JSON: {e}"

            return f"{cmd.upper()} {endpoint} '{payload}'"

        # Handle predefined commands
        if cmd in self.commands:
            method, endpoint, payload = self.commands[cmd]
            return f"{method} {endpoint} '{payload}'"

        # Handle help
        if cmd in ['help', '?']:
            return None, self.get_help()

        return None, f"Unknown command: {cmd}"

    def get_help(self):
        """Return help message."""
        help_text = [
            "\nAvailable commands:",
            "  " + ", ".join(self.get_command_list()),
            "",
            "Special commands:",
            "  update_config {JSON}  - Update rocket configuration",
            "  help, ?               - Show this help",
            "  quit, exit            - Exit the program",
            "",
            "Raw API calls:",
            "  GET /endpoint         - Make GET request",
            "  POST /endpoint '{}'   - Make POST request with JSON",
            "  PUT /endpoint '{}'    - Make PUT request with JSON",
            "  DELETE /endpoint      - Make DELETE request",
            "",
            "Examples:",
            "  status",
            "  start",
            "  update_config {\"reading_interval\": 100}",
            "  GET /status",
            "  POST /command '{\"command\": \"start\"}'",
        ]
        return "\n".join(help_text)


def telemetry_reader(ser, csv_path):
    """Background thread to read telemetry data from serial."""
    global running

    packet_count = 0

    with open(csv_path, 'a', encoding='utf-8') as csv_file:
        while running:
            try:
                # Read line from serial port
                if ser.in_waiting > 0:
                    line = ser.readline()

                    if line:
                        # Decode bytes to string and strip whitespace
                        data = line.decode('utf-8', errors='ignore').strip()

                        if data and not data.startswith('HTTP') and not data.startswith('ERROR'):
                            # Write telemetry to CSV file
                            csv_file.write(data + '\n')
                            csv_file.flush()

                            packet_count += 1
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                            # Log telemetry to stdout
                            with telemetry_lock:
                                print(f"\r\033[K[{timestamp}] Packet #{packet_count}: {data}")
                                print("rocket> ", end='', flush=True)

                        elif data.startswith('HTTP') or data.startswith('ERROR'):
                            # Print HTTP responses and errors
                            with telemetry_lock:
                                print(f"\r\033[K{data}")
                                print("rocket> ", end='', flush=True)

            except UnicodeDecodeError:
                continue
            except Exception as e:
                if running:
                    with telemetry_lock:
                        print(f"\r\033[KWarning: Telemetry reader error: {e}")
                        print("rocket> ", end='', flush=True)


def main():
    global running

    parser = argparse.ArgumentParser(
        description="Interactive rocket telemetry receiver and controller"
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

    print(f"Rocket Telemetry Controller")
    print("=" * 60)
    print(f"Serial device: {args.serial_device}")
    print(f"CSV output file: {csv_path.absolute()}")
    print(f"Baud rate: {args.baudrate}")
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
