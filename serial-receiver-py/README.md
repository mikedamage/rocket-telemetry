# Serial Receiver

Interactive Python script for receiving telemetry data from the ground station via USB serial, logging it to a CSV file, and sending control commands to the rocket.

## Setup

This project uses `uv` for Python package management. The virtual environment and dependencies are managed automatically.

## Usage

Run the script with:

```bash
uv run python main.py <serial_device> <csv_file> [options]
```

### Arguments

- `serial_device`: USB serial device path (e.g., `/dev/ttyUSB0` on Linux/Mac or `COM3` on Windows)
- `csv_file`: Path to the CSV file where data will be logged

### Options

- `-b, --baudrate`: Serial port baud rate (default: 115200)
- `-t, --timeout`: Serial port read timeout in seconds (default: 0.1)

### Examples

```bash
# Basic usage
uv run python main.py /dev/ttyUSB0 telemetry.csv

# With custom baud rate
uv run python main.py /dev/ttyUSB0 telemetry.csv -b 9600

# With custom timeout
uv run python main.py /dev/ttyUSB0 telemetry.csv -t 2.0
```

## Features

### Telemetry Reception
- Automatically creates the CSV file and parent directories if they don't exist
- Appends data to existing CSV files
- Flushes data immediately to ensure no data loss
- Logs each packet to STDOUT with timestamp and packet number
- Runs telemetry reader in background thread for non-blocking operation
- Handles Unicode decode errors gracefully

### Interactive Command Control
- Bidirectional serial communication with ground station
- Interactive command prompt with readline support (command history)
- Predefined rocket control commands
- Support for raw REST API calls
- Real-time HTTP response display

## Interactive Commands

Once running, you'll see an interactive `rocket>` prompt. Available commands:

### Predefined Commands

- `status` - Get rocket status
- `get_config` - Get current rocket configuration
- `get_telemetry` - Get stored telemetry data
- `delete_telemetry` - Delete stored telemetry
- `start` - Start telemetry collection
- `stop` - Stop telemetry collection
- `reboot` - Reboot the rocket
- `reset` - Reset the rocket
- `recalibrate_altimeter` - Recalibrate altimeter to ground level
- `start_beacon` - Start beacon mode
- `update_config {JSON}` - Update rocket configuration

### Raw API Calls

You can also make raw REST API calls:

```
GET /endpoint
POST /endpoint '{"json":"payload"}'
PUT /endpoint '{"json":"payload"}'
DELETE /endpoint
```

### Examples

```
rocket> status
rocket> start
rocket> update_config {"reading_interval": 100}
rocket> GET /status
rocket> POST /command '{"command": "start"}'
rocket> recalibrate_altimeter
```

### Help and Exit

- `help` or `?` - Display available commands
- `quit` or `exit` - Exit the program

## Output

The script displays telemetry data and command responses in real-time:

```
Rocket Telemetry Controller
============================================================
Serial device: /dev/ttyUSB0
CSV output file: /path/to/telemetry.csv
Baud rate: 115200
------------------------------------------------------------
Successfully opened serial port: /dev/ttyUSB0

Type 'help' for available commands, 'quit' to exit
------------------------------------------------------------
rocket> status
HTTP 200: {"state":"running","uptime":12345}
[12:34:56.789] Packet #1: {"temp":25.5,"pressure":1013.25,"altitude":120.5}
rocket> start
HTTP 200: OK
[12:34:57.123] Packet #2: {"temp":25.6,"pressure":1013.20,"altitude":121.0}
rocket>
```

Press Ctrl+C or type `quit` to exit. Telemetry continues to stream in the background while you type commands.
