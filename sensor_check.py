import argparse
import os
import time

import smbus


DEFAULT_SENSOR_BUS = int(os.getenv("ONPLANT_SENSOR_BUS", "3"))

BH1750_ADDR = 0x23
BH1750_CONT_HIGH_RES = 0x10

ADS1115_ADDR_CANDIDATES = [0x48, 0x49, 0x4A, 0x4B]
ADS1115_POINTER_CONVERSION = 0x00
ADS1115_POINTER_CONFIG = 0x01

ADS_CHANNEL_CONFIG = {
    0: 0x4000,
    1: 0x5000,
    2: 0x6000,
    3: 0x7000,
}

ADS_PGA_4_096V = 0x0200
ADS_MODE_SINGLE_SHOT = 0x0100
ADS_DATA_RATE_128SPS = 0x0080
ADS_COMP_DISABLE = 0x0003
ADS_OS_SINGLE = 0x8000
ADS_LSB_VOLTS = 4.096 / 32768.0


def swap_word(value):
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF)


def find_ads1115(bus):
    for address in ADS1115_ADDR_CANDIDATES:
        try:
            bus.read_i2c_block_data(address, ADS1115_POINTER_CONFIG, 2)
            return address
        except OSError:
            pass
    return None


def read_ads_channel(bus, address, channel):
    config = (
        ADS_OS_SINGLE
        | ADS_CHANNEL_CONFIG[channel]
        | ADS_PGA_4_096V
        | ADS_MODE_SINGLE_SHOT
        | ADS_DATA_RATE_128SPS
        | ADS_COMP_DISABLE
    )

    bus.write_word_data(address, ADS1115_POINTER_CONFIG, swap_word(config))
    time.sleep(0.012)

    raw = swap_word(bus.read_word_data(address, ADS1115_POINTER_CONVERSION))
    if raw & 0x8000:
        raw -= 0x10000

    volts = raw * ADS_LSB_VOLTS
    return raw, volts


def read_bh1750_lux(bus):
    bus.write_byte(BH1750_ADDR, BH1750_CONT_HIGH_RES)
    time.sleep(0.18)
    data = bus.read_i2c_block_data(BH1750_ADDR, 0x00, 2)
    return ((data[0] << 8) + data[1]) / 1.2


def moisture_percent(volts, dry_volts, wet_volts):
    if dry_volts == wet_volts:
        return 0.0

    percent = (dry_volts - volts) * 100.0 / (dry_volts - wet_volts)
    return max(0.0, min(100.0, percent))


def main():
    parser = argparse.ArgumentParser(description="Check BH1750 light and ADS1115 soil sensor")
    parser.add_argument("--bus", type=int, default=DEFAULT_SENSOR_BUS)
    parser.add_argument("--dry-volts", type=float, default=3.0)
    parser.add_argument("--wet-volts", type=float, default=1.2)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    bus = smbus.SMBus(args.bus)
    ads_addr = find_ads1115(bus)

    print(f"sensor I2C bus={args.bus}")
    if ads_addr is None:
        print("ADS1115 not found. Expected 0x48, 0x49, 0x4a, or 0x4b.")
    else:
        print(f"ADS1115 found at 0x{ads_addr:02x}")

    print("BH1750 expected at 0x23")
    print("Watch A0 first. If A0 does not change, move the signal wire to A0 and retry.")
    print("Press Ctrl+C to stop.")

    while True:
        try:
            lux = read_bh1750_lux(bus)
            lux_text = f"{lux:8.2f} lux"
        except Exception as e:
            lux_text = f"ERROR {e}"

        if ads_addr is None:
            print(f"light={lux_text} | ADS1115=not_found")
        else:
            a0_raw, a0_v = read_ads_channel(bus, ads_addr, 0)
            a1_raw, a1_v = read_ads_channel(bus, ads_addr, 1)
            a0_pct = moisture_percent(a0_v, args.dry_volts, args.wet_volts)
            a1_pct = moisture_percent(a1_v, args.dry_volts, args.wet_volts)

            print(
                f"light={lux_text} | "
                f"A0 raw={a0_raw:6d} {a0_v:.3f}V soil~{a0_pct:5.1f}% | "
                f"A1 raw={a1_raw:6d} {a1_v:.3f}V soil~{a1_pct:5.1f}%"
            )

        time.sleep(args.interval)



if __name__ == "__main__":
    main()
