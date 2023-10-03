import socket
import time


def main() -> None:
    """Connect."""
    host = "192.168.12.20"  # client ip
    port = 4005

    server = ("192.168.12.1", 4000)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))

    while True:
        message = str(time.time())
        s.sendto(message.encode("utf-8"), server)
        data, addr = s.recvfrom(1024)
        data = data.decode("utf-8")
        print("Received from server: " + data)
    s.close()


if __name__ == "__main__":
    main()
