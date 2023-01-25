import socket
import serial
import sys

def main():
    if len(sys.argv) != 2:
        exit(f"Usage: python serial_bridge.py <PORT>\nExample: python3 serial_bridge.py COM3")

    port = sys.argv[1];

    # Create a UDP socket at client side
    serverAddressPort = ("127.0.0.1", 5433)
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) as UDPClientSocket, serial.Serial(port, 115200) as serialConn:
        # Serial connection
        print("Running...");
        while True:
            data = serialConn.readline()
            data_dec = data.decode('ascii')
            if data_dec[0] != '{':
                print(data_dec.rstrip())

            UDPClientSocket.sendto(data, serverAddressPort)


if __name__ == '__main__':
    main()