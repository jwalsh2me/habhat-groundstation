from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306 #* extra options, ssd1325, ssd1331, sh1106
from time import sleep

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, rotate=0)

# Box and text rendered in portrait mode
with canvas(device) as draw:
    draw.rectangle(device.bounding_box, outline="white", fill="black")
    draw.text((10, 0), "Hello World - 10,0", fill="white")
    draw.text((10, 10), "Hello World - 10,10", fill="white")
    draw.text((10, 20), "Hello World - 10,20", fill="white")
    draw.text((10, 30), "Hello World - 10,30", fill="white")
    draw.text((10, 40), "Hello World - 10,40", fill="white")
    draw.text((10, 50), "Hello World - 10,50", fill="white")
sleep(10)