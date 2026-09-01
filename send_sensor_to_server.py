import time
import requests
import smbus
import board
import adafruit_dht

SERVER_URL = "http://192.168.100.198:8000"

bus = smbus.SMBus(1)

# BH1750
BH1750_ADDR = 0x23

# ADS1115
ADS1115_ADDR = 0x48
ADS1115_POINTER_CONVERSION = 0x00
ADS1115_POINTER_CONFIG = 0x01

# DHT11 on physical pin 11 = GPIO17 = D17
dht = adafruit_dht.DHT11(board.D17)

def swap_word(value):
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF)

def read_light():
    bus.write_byte(BH1750_ADDR, 0x10)
    time.sleep(0.18)
    data = bus.read_i2c_block_data(BH1750_ADDR, 0x00, 2)
    return round(((data[0] << 8) + data[1]) / 1.2, 2)

def read_ads_a0():
    config = 0x8000 | 0x4000 | 0x0200 | 0x0100 | 0x0080 | 0x0003
    bus.write_word_data(ADS1115_ADDR, ADS1115_POINTER_CONFIG, swap_word(config))
    time.sleep(0.012)

    raw = swap_word(bus.read_word_data(ADS1115_ADDR, ADS1115_POINTER_CONVERSION))
    if raw & 0x8000:
        raw -= 0x10000

    volts = raw * (4.096 / 32768.0)
    return raw, volts

def moisture_percent(volts, dry_volts=2.30, wet_volts=0.80):
    percent = (dry_volts - volts) * 100.0 / (dry_volts - wet_volts)
    return max(0.0, min(100.0, percent))

def read_dht():
    try:
        temperature = dht.temperature
        humidity = dht.humidity
        if temperature is None or humidity is None:
            return 0, 0
        return round(temperature, 1), round(humidity, 1)
    except RuntimeError:
        return 0, 0

try:
    while True:
        light = read_light()

        raw, soil_v = read_ads_a0()
        soil = round(moisture_percent(soil_v), 1)

        temperature, humidity = read_dht()

        data = {
            "robot_id": "razbot1",
            "light": light,
            "temperature": temperature,
            "humidity": humidity,
            "soil_moisture": soil
        }

        try:
            res = requests.post(f"{SERVER_URL}/sensor", json=data, timeout=2)
            print("sent", res.status_code, data)
        except Exception as e:
            print("send failed:", e, data)

        time.sleep(5)

except KeyboardInterrupt:
    pass

finally:
    dht.exit()