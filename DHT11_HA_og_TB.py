import network
import time
from umqtt.simple import MQTTClient
import dht
from machine import Pin

# ---- WiFi ----
ssid = "Gruppe4L"
password = "gruppe4b"

# ---- DHT11 ----
sensor = dht.DHT11(Pin(19))

# ---- ThingsBoard MQTT ----
tb_server = "demo.thingsboard.io"
tb_port = 1883
tb_token = "BxGt1zd4kwJgCWqQNsBr"

# ---- Home Assistant MQTT ----
ha_server = "172.16.2.13"
ha_port = 1883
ha_user = "G4"            # <-- 
ha_password = "gruppe4b"          # <--
ha_topic = "homeassistant/sensor/esp32/state"

# ---- WiFi connnect ----
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(ssid, password)

print("Connecting to WiFi...")
while not wifi.isconnected():
    time.sleep(0.5)
print("Connected:", wifi.ifconfig())

# ---- MQTT Clients ----
tb_client = MQTTClient(
    client_id="ESP32_TB",
    server=tb_server,
    port=tb_port,
    user=tb_token,
    password=""
)

ha_client = MQTTClient(
    client_id="ESP32_HA",
    server=ha_server,
    port=ha_port,
    user=ha_user,
    password=ha_password
)

tb_client.connect()
print("Connected to ThingsBoard")

ha_client.connect()
print("Connected to Home Assistant MQTT")


# ---- MAIN LOOP ----
while True:
    try:
        sensor.measure()
        temp = sensor.temperature()
        hum = sensor.humidity()

        payload = '{{"temperature": {}, "humidity": {}}}'.format(temp, hum)

        # --- send til ThingsBoard ---
        tb_client.publish("v1/devices/me/telemetry", payload)
        print("Sent to ThingsBoard:", payload)

        # --- send til Home Assistant ---
        ha_client.publish(ha_topic, payload)
        print("Sent to Home Assistant:", payload)

    except Exception as e:
        print("Error:", e)

    time.sleep(5)
