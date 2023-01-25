import socket
import json


# Create a UDP socket at client side
serverAddressPort = ("127.0.0.1", 5433)
with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) as UDPClientSocket:

    # Send to server using created UDP socket
    data = {
        "wrist": {"x":0.0,"y":0.0,"z":1.0,"w":1.0},
        "fingers":[{"joints":[{"x":1.0,"y":0.0,"z":0.0,"w":1.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0}]},{"joints":[{"x":1.0,"y":0.0,"z":0.0,"w":1.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0}]},{"joints":[{"x":1.0,"y":0.0,"z":0.0,"w":1.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0}]},{"joints":[{"x":1.0,"y":0.0,"z":0.0,"w":1.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0}]},{"joints":[{"x":1.0,"y":0.0,"z":0.0,"w":1.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0},{"x":0.0,"y":0.0,"z":0.0,"w":0.0}]}]
    }

    while True:
        w = float(input(""))
        data["wrist"]["w"] = w
        dataJSON = str.encode(json.dumps(data));
        UDPClientSocket.sendto(dataJSON, serverAddressPort)

    UDPClientSocket.close()
