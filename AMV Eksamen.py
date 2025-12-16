from uthingsboard.client import TBDeviceMqttClient
from time import sleep
from sys import exit
import gc
import secrets
from machine import Pin
AMV = Pin(16, Pin.OUT)

def Blink():
    AMV.on()

    sleep(5)

    AMV.off()

def handler(req_id, method, params):
    """handler callback to recieve RPC from server """
    print(f'Response {req_id}: {method}, params {params}')
    print(params, "params type:", type(params))
    try:
        if method == "toggle_Blink":
            if params == True:
                print("buzzer on")
                Blink()
            else:
                print("buzzer off")  
        if method == "sendCommand":
            print(params.get("command"))

    except TypeError as e:
        print(e)

client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token = secrets.ACCESS_TOKEN)

client.connect()
print("connected to thingsboard, starting to send and receive data")
while True:
    try:
        print(f"free memory: {gc.mem_free()}")
        if gc.mem_free() < 2000:
            print("Garbage collected!")
            gc.collect()
        
        client.set_server_side_rpc_request_handler(handler) 
        
        client.check_msg()
        sleep(3)
    except KeyboardInterrupt:
        print("Disconnected!")
        client.disconnect()
        exit()

    