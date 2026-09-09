import time
import board
import adafruit_dht

dht = adafruit_dht.DHT11(board.D17)  # DHT22면 DHT22로 변경

while True:
    try:
        print("TEMP=", dht.temperature, "HUM=", dht.humidity)
    except RuntimeError as e:
        print("read error:", e)
    time.sleep(2)