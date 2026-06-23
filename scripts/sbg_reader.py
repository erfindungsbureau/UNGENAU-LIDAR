#!/usr/bin/env python3
import serial, struct, math, json, time

PORT, BAUD, OUTPUT = "/dev/ttyUSB0", 921600, "/tmp/imu.json"

def run():
    s = serial.Serial(PORT, BAUD, timeout=0.1)
    print("SBG reader running", flush=True)
    buf = bytearray()
    while True:
        chunk = s.read(512)
        if chunk:
            buf.extend(chunk)

        consumed = 0
        while consumed < len(buf) - 7:
            i = consumed
            if buf[i] != 0xFF or buf[i+1] != 0x5A:
                consumed += 1
                continue
            c, d   = buf[i+2], buf[i+3]
            length = buf[i+4] | (buf[i+5] << 8)
            total  = 6 + length + 2
            if i + total > len(buf):
                break  # Frame noch nicht vollständig

            if c == 0x06 and d == 0x00 and length >= 16:
                payload = buf[i+6:i+6+length]
                _, roll, pitch, yaw = struct.unpack_from("<Ifff", payload, 0)
                imu = {
                    "roll":  round(math.degrees(roll), 2),
                    "pitch": round(math.degrees(pitch), 2),
                    "yaw":   round(math.degrees(yaw), 2),
                }
                with open(OUTPUT, "w") as f:
                    f.write(json.dumps(imu))

            consumed += total

        del buf[:consumed]  # Verarbeitete Bytes entfernen

while True:
    try:
        run()
    except Exception as e:
        print(f"Error: {e}, retry in 2s", flush=True)
        time.sleep(2)
