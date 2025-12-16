import requests
from machine import Pin
from time import sleep
import esp32


try:    import ujson as json
except ImportError:
    import json 
RL = Pin(22, Pin.OUT)
GL = Pin(23, Pin.OUT)
relay = Pin(16, Pin.OUT)


while True:
    CO2_response = requests.get(url='https://api.energidataservice.dk/dataset/CO2Emis?limit=2')
    CO2_result = CO2_response.json()
    CO2_records = CO2_result.get('records', [])
    
    for record in CO2_records:
        print(CO2_records[1])
        co2_emission = CO2_result["records"][1]["CO2Emission"]
        sleep(1)
        if co2_emission <= 100:
            
            print(co2_emission)
            GL.on()
            relay.on()
            RL.off()
            print('grøn')
        else:
            print(co2_emission)
            RL.on()
            relay.off()
            GL.off()
            print('ikke grøn')