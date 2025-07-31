import socket

HOST = '192.168.1.101'  # Ground station IP (check Serial Monitor)
PORT = 5001

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:
        cmd = input("Enter 8-bit integer (0-255, or 'q' to quit): ")
        if cmd == 'q':
            break
        try:
            value = int(cmd)
            if 0 <= value <= 255:
                s.sendall(bytes([value]))
                print(f"Sent: {value}")
            else:
                print("Value must be 0-255")
        except ValueError:
            print("Invalid input")