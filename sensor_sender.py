from __future__ import annotations

import argparse
import os
import queue
import random
import subprocess
import threading
import time
from typing import Any

import requests

from local_runtime import write_sensor_cache

try:
    import smbus
except ImportError:
    import smbus2 as smbus

try:
    import adafruit_dht
    import board
except ImportError:
    adafruit_dht = None
    board = None


DEFAULT_SERVER_URL = "http://192.168.100.6:5050"
DEFAULT_ROBOT_ID = "raspbot-a"

DEFAULT_SENSOR_BUS = int(os.getenv("ONPLANT_SENSOR_BUS", "3"))
DHT_READ_INTERVAL = float(os.getenv("ONPLANT_DHT_READ_INTERVAL", "3"))
_last_dht_read = 0.0
_last_dht_values: tuple[float | None, float | None] = (None, None)

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
ADS_OS_SINGLE = 0x8000
ADS_PGA_4_096V = 0x0200
ADS_MODE_SINGLE_SHOT = 0x0100
ADS_DATA_RATE_128SPS = 0x0080
ADS_COMP_DISABLE = 0x0003
ADS_LSB_VOLTS = 4.096 / 32768.0


def swap_word(value: int) -> int:
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF)


def find_ads1115(bus: smbus.SMBus) -> int | None:
    for address in ADS1115_ADDR_CANDIDATES:
        try:
            bus.read_i2c_block_data(address, ADS1115_POINTER_CONFIG, 2)
            return address
        except OSError:
            pass
    return None


def read_bh1750_lux(bus: smbus.SMBus) -> float:
    bus.write_byte(BH1750_ADDR, BH1750_CONT_HIGH_RES)
    time.sleep(0.18)
    data = bus.read_i2c_block_data(BH1750_ADDR, 0x00, 2)
    return round(((data[0] << 8) + data[1]) / 1.2, 1)


def read_ads_channel(bus: smbus.SMBus, address: int, channel: int) -> tuple[int, float]:
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


def moisture_percent(volts: float, dry_volts: float, wet_volts: float) -> float:
    if dry_volts == wet_volts:
        return 0.0
    percent = (dry_volts - volts) * 100.0 / (dry_volts - wet_volts)
    return round(max(0.0, min(100.0, percent)), 1)


def enable_dht_power(gpio: int, settle_seconds: float) -> None:
    if gpio < 0:
        return
    try:
        subprocess.run(["pinctrl", "set", str(gpio), "op", "dh"], check=True)
        print(f"DHT power GPIO{gpio}=HIGH")
        time.sleep(max(0.0, settle_seconds))
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"DHT power setup failed on GPIO{gpio}:", exc)


def read_dht11(pin_name: str) -> tuple[float | None, float | None]:
    global _last_dht_read, _last_dht_values

    now = time.monotonic()
    if now - _last_dht_read < DHT_READ_INTERVAL and any(value is not None for value in _last_dht_values):
        return _last_dht_values

    if adafruit_dht is None or board is None:
        raise RuntimeError("adafruit_dht or board module is not installed")

    pin = getattr(board, pin_name)
    dht = adafruit_dht.DHT11(pin, use_pulseio=False)
    try:
        temperature = dht.temperature
        humidity = dht.humidity
        if temperature is None or humidity is None:
            _last_dht_read = now
            return _last_dht_values
        _last_dht_values = (round(float(temperature), 1), round(float(humidity), 1))
        _last_dht_read = now
        return _last_dht_values
    finally:
        dht.exit()


def dummy_payload() -> dict[str, float]:
    return {
        "lux": round(random.uniform(820, 940), 1),
        "temperature": round(random.uniform(23.0, 26.0), 1),
        "humidity": round(random.uniform(45.0, 58.0), 1),
        "soil_moisture": round(random.uniform(30.0, 42.0), 1),
    }


def read_sensors(args: argparse.Namespace, bus: smbus.SMBus, ads_addr: int | None) -> dict[str, Any]:
    values: dict[str, Any] = {
        "lux": None,
        "temperature": None,
        "humidity": None,
        "soil_moisture": None,
    }

    if args.lux_mode == "dummy":
        values["lux"] = dummy_payload()["lux"]
    elif args.lux_mode == "real":
        try:
            values["lux"] = read_bh1750_lux(bus)
        except Exception as exc:
            print("BH1750 read failed:", exc)

    if args.dht_mode == "dummy":
        dummy = dummy_payload()
        values["temperature"] = dummy["temperature"]
        values["humidity"] = dummy["humidity"]
    elif args.dht_mode == "real":
        try:
            temperature, humidity = read_dht11(args.dht_pin)
            values["temperature"] = temperature
            values["humidity"] = humidity
        except RuntimeError as exc:
            print("DHT read skipped:", exc)
        except Exception as exc:
            print("DHT read failed:", exc)

    if args.soil_mode == "dummy":
        values["soil_moisture"] = dummy_payload()["soil_moisture"]
    elif args.soil_mode == "real":
        if ads_addr is None:
            print("ADS1115 not found. soil_moisture=None")
        else:
            try:
                _raw, volts = read_ads_channel(bus, ads_addr, args.soil_channel)
                values["soil_moisture"] = moisture_percent(volts, args.dry_volts, args.wet_volts)
            except Exception as exc:
                print("ADS1115 read failed:", exc)

    return values


def post_sensor(server_url: str, robot_id: str, values: dict[str, Any]) -> dict[str, Any]:
    payload = {
        "robot_id": robot_id,
        "lux": values.get("lux"),
        "temperature": values.get("temperature"),
        "humidity": values.get("humidity"),
        "soil_moisture": values.get("soil_moisture"),
        "source": "raspberry-real",
    }
    response = requests.post(f"{server_url.rstrip('/')}/api/sensors", json=payload, timeout=5)
    response.raise_for_status()
    return response.json()


def sensor_post_worker(
    server_url: str,
    robot_id: str,
    pending_values: queue.Queue,
    retry_seconds: float = 30.0,
) -> None:
    values = None
    while True:
        if values is None:
            values = pending_values.get()
        try:
            stored = post_sensor(server_url, robot_id, values)
            print(
                "sent",
                f"id={stored.get('id')}",
                f"lux={stored.get('lux')}",
                f"temp={stored.get('temperature')}",
                f"hum={stored.get('humidity')}",
                f"soil={stored.get('soil_moisture')}",
            )
            values = None
        except Exception as exc:
            print("send failed; local sensor cache continues:", exc)
            try:
                values = pending_values.get(timeout=retry_seconds)
            except queue.Empty:
                pass


def enqueue_latest(pending_values: queue.Queue, values: dict[str, Any]) -> None:
    latest = dict(values)
    try:
        pending_values.put_nowait(latest)
        return
    except queue.Full:
        pass
    try:
        pending_values.get_nowait()
    except queue.Empty:
        pass
    pending_values.put_nowait(latest)


def main() -> int:
    parser = argparse.ArgumentParser(description="Send real OnPlant sensor values to FastAPI server.")
    parser.add_argument("--server", default=DEFAULT_SERVER_URL)
    parser.add_argument("--robot-id", default=DEFAULT_ROBOT_ID)
    parser.add_argument("--bus", type=int, default=DEFAULT_SENSOR_BUS)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--server-interval", type=float, default=5.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--lux-mode", choices=["real", "dummy", "none"], default="real")
    parser.add_argument("--dht-mode", choices=["real", "dummy", "none"], default="dummy")
    parser.add_argument("--soil-mode", choices=["real", "dummy", "none"], default="dummy")
    parser.add_argument("--dht-pin", default=os.getenv("ONPLANT_DHT_PIN", "D27"))
    parser.add_argument("--dht-power-gpio", type=int, default=int(os.getenv("ONPLANT_DHT_POWER_GPIO", "17")))
    parser.add_argument("--dht-power-settle", type=float, default=float(os.getenv("ONPLANT_DHT_POWER_SETTLE", "5")))
    parser.add_argument("--soil-channel", type=int, choices=[0, 1, 2, 3], default=0)
    parser.add_argument("--dry-volts", type=float, default=3.0)
    parser.add_argument("--wet-volts", type=float, default=1.2)
    args = parser.parse_args()

    if args.dht_mode == "real":
        enable_dht_power(args.dht_power_gpio, args.dht_power_settle)

    bus = smbus.SMBus(args.bus)
    print(f"sensor I2C bus={args.bus}")
    ads_addr = find_ads1115(bus)
    if args.soil_mode == "real":
        if ads_addr is None:
            print("ADS1115 not found. Expected 0x48, 0x49, 0x4a, or 0x4b.")
        else:
            print(f"ADS1115 found at 0x{ads_addr:02x}")

    pending_values: queue.Queue = queue.Queue(maxsize=1)
    if not args.once:
        threading.Thread(
            target=sensor_post_worker,
            args=(args.server, args.robot_id, pending_values),
            daemon=True,
        ).start()

    next_server_post = 0.0
    while True:
        values = read_sensors(args, bus, ads_addr)
        write_sensor_cache(args.robot_id, values)
        now = time.monotonic()
        if args.once:
            try:
                stored = post_sensor(args.server, args.robot_id, values)
                print(
                    "sent",
                    f"id={stored.get('id')}",
                    f"lux={stored.get('lux')}",
                    f"temp={stored.get('temperature')}",
                    f"hum={stored.get('humidity')}",
                    f"soil={stored.get('soil_moisture')}",
                )
            except Exception as exc:
                print("send failed; local sensor cache is still updated:", exc, values)
        elif now >= next_server_post:
            enqueue_latest(pending_values, values)
            next_server_post = now + max(args.interval, args.server_interval)

        if args.once:
            break
        time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
