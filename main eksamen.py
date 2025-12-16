import urequests as requests
from time import sleep, ticks_ms, ticks_diff, time
from machine import Pin, UART, PWM, I2C
import gc
import secrets
import math
from gps_simple import GPS_SIMPLE
from gpio_lcd import GpioLcd
from uthingsboard.client import TBDeviceMqttClient
from adc_sub import ADC_substitute
from mpu6050 import MPU6050

lcd = GpioLcd(rs_pin=Pin(27), enable_pin=Pin(25),
              d4_pin=Pin(33), d5_pin=Pin(32),
              d6_pin=Pin(21), d7_pin=Pin(22),
              num_lines=4, num_columns=20)

uart = UART(2, 9600)
gps = GPS_SIMPLE(uart)

adc = ADC_substitute(34)

buzzer = PWM(Pin(14, Pin.OUT), duty=0)
alarm_LED = Pin(26, Pin.OUT)

venstre_LED = Pin(15, Pin.OUT)  
venstre_knap = Pin(4, Pin.IN)
højre_LED = Pin(2, Pin.OUT)
højre_knap = Pin(0, Pin.IN)


bremse_LED = Pin(13, Pin.OUT)
i2c = I2C(0)
imu = MPU6050(i2c)

client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token=secrets.ACCESS_TOKEN)
client.connect()

alarm_armed = False
alarm_triggered = False
last_lat = None
last_lon = None
fence_radius = 0.0003
last_movement_time = time()
AUTO_ARM_DELAY = 180

# Blinklys
turn_active = False
turn_side = None
turn_start = 0
last_blink = 0
blink_state = False
BLINK_INTERVAL = 300
BLINK_DURATION = 10000

# Bremselys
braking = False
last_brake_check = 0

# Timing
last_display = 0
DISPLAY_INTERVAL = 4000
last_telemetry = 0
TELEMETRY_INTERVAL = 5000
last_weather = 0
WEATHER_INTERVAL = 60000

# Vejr
weather_desc = "Loading..."
temperature = 0
wind_speed = 0
frost_status = "Unknown"

toggle = 0 

def get_gps():
    if gps.receive_nmea_data():
        lat = gps.get_latitude()
        lon = gps.get_longitude()
        speed = gps.get_speed()
        if lat != -999.0 and lon != -999.0:
            return lat, lon, speed
    return None, None, 0

def get_battery():
    val = adc.read_adc()
    batt = int((val - 1659)/(2380-1659)*100)
    return max(0, min(100, batt))

def update_weather():
    global weather_desc, temperature, wind_speed, frost_status
    try:
        url = "http://api.openweathermap.org/data/2.5/weather?q=Copenhagen,dk&APPID=a472846cd47c408475fded82cadf6336"
        print("Henter vejr...")
        r = requests.get(url, timeout=10)
        print("HTTP Status:", r.status_code)
        
        if r.status_code != 200:
            print("Fejl respons:", r.text)
            raise Exception("HTTP " + str(r.status_code))
            
        data = r.json()
        print("Data modtaget:", data.keys())
        
        weather_desc = data["weather"][0]["description"]
        temperature = data["main"]["feels_like"] - 273.15
        wind_speed = data["wind"]["speed"]
        frost_status = "Frost!" if wind_speed < 2 and temperature < 2 else "No frost"
        r.close()
        print("Vejr opdateret OK")
        
    except Exception as e:
        print("Vejr fejl:", type(e).__name__, str(e))
        weather_desc = "No connection"
        temperature = 0
        wind_speed = 0
        frost_status = "Unknown"

def check_movement(lat, lon):
    global last_lat, last_lon, last_movement_time, alarm_triggered
    if lat is None or lon is None:
        return
    if last_lat is None:
        last_lat, last_lon = lat, lon
        last_movement_time = time()
        return
    distance = math.sqrt((lat-last_lat)**2 + (lon-last_lon)**2)
    if distance >= fence_radius:
        last_lat, last_lon = lat, lon
        last_movement_time = time()
        if alarm_armed and not alarm_triggered:
            alarm_triggered = True
            sound_alarm()

def sound_alarm():
    for _ in range(3):
        buzzer.freq(440)
        buzzer.duty(512)
        alarm_LED.on()
        sleep(0.3)
        buzzer.duty(0)
        alarm_LED.off()
        sleep(0.3)

def auto_arm():
    global alarm_armed, alarm_triggered
    if not alarm_armed and time() - last_movement_time >= AUTO_ARM_DELAY:
        alarm_armed = True
        alarm_triggered = False

def rpc_handler(req_id, method, params):
    global alarm_armed, alarm_triggered, last_movement_time
    if method == "Toggle_knap":
        alarm_armed = bool(params)
        alarm_triggered = False
        last_movement_time = time()
        print("Alarm:", "ON" if alarm_armed else "OFF")

def check_turn_buttons():
    """Tjek om blinklys-knapper er trykket"""
    global turn_active, turn_side, turn_start, last_blink, blink_state
    
    if not turn_active:
        if venstre_knap.value() == 0:
            turn_active = True
            turn_side = "left"
            turn_start = ticks_ms()
            last_blink = ticks_ms()
            blink_state = False
            print("Venstre blinklys ON")
        elif højre_knap.value() == 0:
            turn_active = True
            turn_side = "right"
            turn_start = ticks_ms()
            last_blink = ticks_ms()
            blink_state = False
            print("Højre blinklys ON")

def update_turn_signals():
    """Opdater blinklys (non-blocking)"""
    global turn_active, last_blink, blink_state
    
    if not turn_active:
        return
    
    now = ticks_ms()
    
    if ticks_diff(now, turn_start) >= BLINK_DURATION:
        turn_active = False
        venstre_LED.off()
        højre_LED.off()
        print("Blinklys OFF (timeout)")
        return
    
    if turn_side == "left" and højre_knap.value() == 0:
        turn_active = False
        venstre_LED.off()
        print("Blinklys OFF (højre knap)")
        return
    if turn_side == "right" and venstre_knap.value() == 0:
        turn_active = False
        højre_LED.off()
        print("Blinklys OFF (venstre knap)")
        return
    
    if ticks_diff(now, last_blink) >= BLINK_INTERVAL:
        blink_state = not blink_state
        if turn_side == "left":
            venstre_LED.value(blink_state)
        else:
            højre_LED.value(blink_state)
        last_blink = now


def check_braking():
    """Tjek om der bremses (non-blocking)"""
    global braking, last_brake_check
    
    now = ticks_ms()
    
    if ticks_diff(now, last_brake_check) < 100:
        return
    
    last_brake_check = now
    
    try:
        values = imu.get_values()
        x_acc = values.get("acc x")
        
        if x_acc is not None and x_acc < -2000:
            if not braking:
                print("Bremsning registreret!")
                braking = True
            bremse_LED.on()
        else:
            if braking:
                print("Ingen bremsning")
                braking = False
            bremse_LED.off()
    except Exception as e:
        pass

def update_display(lat, lon, speed, batt):
    global toggle
    lcd.clear()
    screen = toggle % 2
    toggle += 1
    
    if screen == 0:
        lcd.move_to(0,0)
        lcd.putstr("Lat:"+str(lat)[:12] if lat else "Lat:N/A")
        lcd.move_to(0,1)
        lcd.putstr("Lon:"+str(lon)[:12] if lon else "Lon:N/A")
        lcd.move_to(0,2)
        lcd.putstr("Spd:"+str(speed)[:5]+" Bat:"+str(batt)+"%")
        lcd.move_to(0,3)
        status = "A:" + ("ON" if alarm_armed else "OFF")
        lcd.putstr(status)
    else:
        lcd.move_to(0,0)
        lcd.putstr("Weather Copenhagen")
        lcd.move_to(0,1)
        lcd.putstr(weather_desc[:20])
        lcd.move_to(0,2)
        lcd.putstr("Temp:"+str(int(temperature))+"C Wind:"+str(wind_speed)[:3])
        lcd.move_to(0,3)
        lcd.putstr(frost_status)

client.set_server_side_rpc_request_handler(rpc_handler)

lcd.clear()
lcd.putstr("Smart Bike System")
lcd.move_to(0,1)
lcd.putstr("Stater op")
sleep(2)

print("Henter API")
update_weather()

try:
    while True:
        now = ticks_ms()
        
        lat, lon, speed = get_gps()
        batt = get_battery()
        
        auto_arm()
        check_movement(lat, lon)
        
        check_turn_buttons()
        update_turn_signals()
        
        check_braking()
        
        if ticks_diff(now, last_display) > 4000:
            update_display(lat, lon, speed, batt)
            last_display = now
        
        if ticks_diff(now, last_weather) > 60000:
            update_weather()
            last_weather = now
        
        if ticks_diff(now, last_telemetry) > 5000:
            client.send_telemetry({
                "latitude": lat or 0,
                "longitude": lon or 0,
                "speed": speed,
                "battery": batt,
                "alarm_armed": alarm_armed,
                "alarm_triggered": alarm_triggered,
                "weather": weather_desc,
                "temperature": temperature,
                "wind_speed": wind_speed,
                "frost_status": frost_status,
            })
            last_telemetry = now
        
        client.check_msg()
        
        if gc.mem_free() < 2000:
            gc.collect()
        
        sleep(0.05)

except KeyboardInterrupt:
    print("program slut")
