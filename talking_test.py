
import time
from pyb import UART

# Nicla Vision external UART:
# PA9 = TX, PA10 = RX
uart = UART("LP1", 115200)

print("NICLA UART TEST STARTED")

while True:
    uart.write("UNHARMED\n")
    print("Sent: UNHARMED")
    time.sleep_ms(1000)
