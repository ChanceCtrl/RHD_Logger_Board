import socket

import RHDResponses.RawResult


def main() -> None:
    UDP_IP = "192.168.4.2"
    UDP_PORT = 5005

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening on {UDP_IP}:{UDP_PORT}")

    while True:
        data, addr = sock.recvfrom(256)
        raw_result = RHDResponses.RawResult.RawResult.GetRootAs(data, 0)
        print(f"""
        Timestamp: {raw_result.TimestampUs()}
        Command: {raw_result.CommandType()}
        Register: {raw_result.Register()}
        rhd_a: {raw_result.RhdA()}
        rhd_b: {raw_result.RhdB()}
        """)


if __name__ == "__main__":
    main()
