import smbus
import time

bus = smbus.SMBus(1)
addr = 0x23

while True:
    bus.write_byte(addr, 0x10)

    time.sleep(0.2)

    data = bus.read_i2c_block_data(addr, 0x00, 2)

    lux = ((data[0] << 8) + data[1]) / 1.2

    print(f"조도: {lux:.2f} lux")

    time.sleep(1)