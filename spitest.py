import spidev
import time

spi = spidev.SpiDev()

spi.open(0,0)
spi.mode = 0
spi.max_speed_hz = 5000000

while 1:
        result = spi.xfer([0xfd, 0x00, 0xFF, 0x00, 0x00]) #spi.readbytes(16)
        print(result)
        time.sleep(2)
