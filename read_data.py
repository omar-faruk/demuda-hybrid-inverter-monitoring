import serial
import time
import csv

# ---------------- CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
SLAVE_ID = 1

START_ADDR = 0
END_ADDR = 1000
BLOCK_SIZE = 10

INTER_DELAY = 0.05

# ---------------- CRC ----------------
def crc16(data: bytes) -> bytes:
    crc = 0xFFFF

    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1

    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


# ---------------- READ BLOCK ----------------
def read_block(ser, addr, count):

    frame = bytes([
        SLAVE_ID,
        0x03
    ]) + addr.to_bytes(2, "big") + count.to_bytes(2, "big")

    frame += crc16(frame)

    ser.reset_input_buffer()
    ser.write(frame)

    time.sleep(0.05)

    return ser.read(256)


# ---------------- PARSE ----------------
def parse_response(resp):

    if len(resp) < 5:
        return None, "TIMEOUT"

    if resp[1] & 0x80:
        return None, f"EXCEPTION {resp[2]}"

    byte_count = resp[2]

    regs = []
    for i in range(0, byte_count, 2):
        regs.append(int.from_bytes(resp[3+i:3+i+2], "big"))

    return regs, None


# ---------------- MAIN ----------------
def main():

    print("\n=== FULL FC03 DUMP (0 → 1000) ===\n")

    with serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    ) as ser:

        with open("fc03_full_dump.csv", "w", newline="") as f:

            writer = csv.writer(f)
            writer.writerow(["Address", "Value", "Hex", "Status"])

            addr = START_ADDR

            while addr <= END_ADDR:

                resp = read_block(ser, addr, BLOCK_SIZE)
                regs, err = parse_response(resp)

                print(f"\n[ADDR {addr:4d} - {addr+BLOCK_SIZE-1:4d}]")

                if err:
                    # print(f"  -> {err}")

                    addr += BLOCK_SIZE
                    continue

                for i, val in enumerate(regs):

                    a = addr + i

                    print(f"  {a:4d} = {val:6d} (0x{val:04X})")

                    writer.writerow([
                        a,
                        val,
                        f"0x{val:04X}",
                        "OK"
                    ])

                addr += BLOCK_SIZE

                time.sleep(INTER_DELAY)

    print("\nDONE → saved to fc03_full_dump.csv\n")


if __name__ == "__main__":
    main()