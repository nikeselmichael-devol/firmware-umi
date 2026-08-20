"""
Minimal receiver matching the ESP32 Packet struct (main.cpp).
Run: python receiver.py
Listens on 0.0.0.0:5005 for a TCP connection from the ESP32.
"""
import socket
import struct

# seq(I) t_us(I) enc1(H) enc2(H) enc1_err(B) enc2_err(B) ax ay az gx gy gz (6f)
FMT = "<IIHHBB6f"
PKT_SIZE = struct.calcsize(FMT)

HOST = "0.0.0.0"
PORT = 5005

def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"Listening on {HOST}:{PORT}, packet size = {PKT_SIZE} bytes")

    while True:
        conn, addr = srv.accept()
        print(f"Connected: {addr}")
        buf = b""
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                buf += data
                while len(buf) >= PKT_SIZE:
                    chunk, buf = buf[:PKT_SIZE], buf[PKT_SIZE:]
                    (seq, t_us, enc1, enc2, enc1_err, enc2_err,
                     ax, ay, az, gx, gy, gz) = struct.unpack(FMT, chunk)

                    enc1_deg = enc1 * 360.0 / 16384.0
                    enc2_deg = enc2 * 360.0 / 16384.0

                    print(f"seq={seq:6d} t={t_us:10d}us "
                          f"enc1={enc1_deg:7.2f}deg(err={enc1_err}) "
                          f"enc2={enc2_deg:7.2f}deg(err={enc2_err}) "
                          f"a=({ax:+.2f},{ay:+.2f},{az:+.2f})g "
                          f"g=({gx:+.1f},{gy:+.1f},{gz:+.1f})dps")
        except ConnectionResetError:
            pass
        finally:
            conn.close()
            print("Disconnected, waiting for next connection...")

if __name__ == "__main__":
    main()
