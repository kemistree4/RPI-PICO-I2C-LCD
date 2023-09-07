# This micropython script reads from two A02YYUW Ultrasonic sensors connected to the 
# UART0 and UART1 pins on a Raspberry Pi Pico. It displays the distance readings and the 
# number of fish counted on a connected I2C LCD screen. 
# Logic: a fish must pas the downstream sensor first, then the upstream one.
# A trigger is counted as a deviation of more than 30% of the default distance to 
# the opposite wall. Once the downstream sensor is triggered, the Pico then checks 
# the upstream sensor four times in the next second to see if it's triggered as well. 
# If it is, a fish is counted and the counter increments. If the upstream sensor is 
# not triggered within a second, the initial trigger is discounted and the loop 
# starts again.

from machine import Pin, UART, I2C
from lcd_api import LcdApi
from pico_i2c_lcd import I2cLcd
import time
import utime

# I2C screen setup
I2C_ADDR     = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 16
i2c = I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)    
utime.sleep(2)
lcd.clear()

# Distance sensors setup (dn=downstream, up=upstream)
uart_dn = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
uart_dn.init(bits=8, parity=None, stop=1)
led = Pin("LED", Pin.OUT)
uart_dn.write(b'\xff')

uart_up = UART(0, baudrate=9600, tx=Pin(12), rx=Pin(13))
uart_up.init(bits=8, parity=None, stop=1)
uart_up.write(b'\xff')

fish_count = 0

def checkUpstreamSensor():
    if uart_up.any():
        data_up = uart_up.read()
        distance = (data_up[1] * 256) + data_up[2]
    else:
        distance = 1000
    return distance

def checkDownstreamSensor():
    if uart_dn.any():
        data_dn = uart_dn.read()
        distance = (data_dn[1] * 256) + data_dn[2]
    else:
        distance = 1000
    return distance
    
while True:

    # Check downstream sensor
    # For testing, opposite wall is ~480mm away
    # If value < 329mm (30% change), check the upstream sensor 
    # four times over the next second to see if it's changed
    lcd.move_to(4, 1)
    lcd.putstr(str(fish_count))
    distance_dn = checkDownstreamSensor()
    print(distance_dn, 'mm')
    dist_string_dn = str(distance_dn) + 'mm'
    lcd.move_to(0, 0)
    lcd.putstr(dist_string_dn)

    if distance_dn < 330:
        # Got a reading; start checking the upstream sensor
        fish_occurred = False
        for _ in range(4):
            distance_up = checkUpstreamSensor()

            if distance_up < 330:  # you gotta fish
                print(distance_up, 'mm')
                dist_string_up = str(distance_up) + 'mm'
                lcd.move_to(8, 0)
                lcd.putstr(dist_string_up)
                fish_count += 1
                print(fish_count)
                fish_occurred = True
                break  # Exit the for loop if you gotta fish

            time.sleep(0.25)  # Wait for 0.25 seconds between checks
    
        if fish_occurred:
            continue        

    # Check the sensor(s) every 0.25 seconds 
    time.sleep(0.25)    

