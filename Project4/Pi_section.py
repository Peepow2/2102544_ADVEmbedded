# -------------- Raspberry Pi Client (Refactored) -------------- #
import socket
import struct
import hashlib
import random
from Crypto.Cipher import AES

# --- Helper: Receive exact bytes from TCP stream ---
def read_n_bytes(sock, n):
    buffer = b''
    while len(buffer) < n:
        chunk = sock.recv(n - len(buffer))
        if not chunk:
            raise IOError("Connection closed unexpectedly")
        buffer += chunk
    return buffer

# --- Classroom DH Parameters ---
DH_PRIME = 23
DH_BASE = 5

# --- Server Configuration ---
HOST = "ESP32_IP"  # ESP32 IP
PORT = 10000
PAYLOAD_SIZE = 144  # 136 bytes struct + 8 bytes padding

# --- Sensor Data Struct (144 bytes) ---
SENSOR_STRUCT_FMT = '<ff32sffffff32sff32s8s'
SENSOR_STRUCT_SIZE = struct.calcsize(SENSOR_STRUCT_FMT)

# --- Diffie-Hellman Handshake & AES Key Derivation ---
def classroom_dh_key_exchange(sock):
    print("[DH] Initiating Classroom-style DH handshake...")

    # Generate client private/public
    priv = random.randint(2, 20)
    pub = pow(DH_BASE, priv, DH_PRIME)
    print(f"[DH] Client Private={priv}, Public={pub}")

    # Send client public key (1 byte)
    sock.sendall(bytes([pub]))
    print("[DH] Client public key sent, waiting for server...")

    # Receive server public key (1 byte)
    server_pub_bytes = read_n_bytes(sock, 1)
    server_pub = int.from_bytes(server_pub_bytes, 'big')
    print(f"[DH] Received server public key: {server_pub}")

    # Compute shared secret
    secret = pow(server_pub, priv, DH_PRIME)
    print(f"[DH] Shared secret: {secret}")

    # Derive 32-byte AES key using SHA-256
    secret_bytes = secret.to_bytes(8, 'little')
    aes_key = hashlib.sha256(secret_bytes).digest()
    print(f"[AES] Derived AES key: {aes_key.hex()}")

    return aes_key

# --- Main Client Loop ---
def main():
    print(f"Connecting to {HOST}:{PORT}...")
    print(f"Expected struct size: {SENSOR_STRUCT_SIZE} bytes")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((HOST, PORT))
        print("[INFO] Connected to server.")

        # Perform DH handshake
        AES_KEY = classroom_dh_key_exchange(sock)

        while True:
            # Receive IV + Encrypted Payload
            iv = read_n_bytes(sock, 16)
            encrypted = read_n_bytes(sock, PAYLOAD_SIZE)

            # Decrypt using AES-CBC
            cipher = AES.new(AES_KEY, AES.MODE_CBC, iv)
            decrypted = cipher.decrypt(encrypted)

            try:
                # Unpack sensor struct
                data = struct.unpack(SENSOR_STRUCT_FMT, decrypted)

                bmp_temp, bmp_pressure, bmp_time = data[0], data[1], data[2].decode('utf-8').rstrip('\x00')
                accel_x, accel_y, accel_z = data[3], data[4], data[5]
                gyro_x, gyro_y, gyro_z = data[6], data[7], data[8]
                mpu_time = data[9].decode('utf-8').rstrip('\x00')
                sht_temp, sht_humidity = data[10], data[11]
                sht_time = data[12].decode('utf-8').rstrip('\x00')

                print(f"\n[AES] Current key: {AES_KEY.hex()}")
                print("---------------------------------")
                print(f"SUCCESSFULLY DECRYPTED 3-SENSOR DATA")
                print(f"SHT4x ({sht_time}): Temp={sht_temp:.2f}C, Humidity={sht_humidity:.2f}%")
                print(f"BMP280 ({bmp_time}): Temp={bmp_temp:.2f}C, Pressure={bmp_pressure:.2f} hPa")
                print(f"MPU6050 ({mpu_time}): Accel[X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f}], "
                      f"Gyro[X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}]")
                print("---------------------------------\n")

            except Exception as e:
                print(f"!!! ERROR unpacking/decrypting data: {e} !!!")

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        sock.close()
        print("[INFO] Connection closed.")

if __name__ == "__main__":
    main()
