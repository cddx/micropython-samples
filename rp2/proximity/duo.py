# duo.py Test two instances of proximity class
# Released under the MIT License (MIT). See LICENSE.
# Copyright (c) 2025 Peter Hinch

# The timer constantly populates a ring buffer, at a rate of 1000 samples/sec.
# Display code periodically averages buffer contents.
# Buffer size is defined to correspond to an integer number of mains cycles.
# Prior to display an offset is subtracted from the buffer mean: this compensates
# for stray capacitance.

from time import sleep_ms
from proximity import Proximity
import neopixel
from machine import Pin

# NeoPixel
p = Pin(16)
n = neopixel.NeoPixel(p, 8)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
cmap = [black, green, red, blue]


def run():
    p0 = Proximity(0, 2)  # SM 0, pin 2
    p1 = Proximity(1, 22)  # SM 1 pin 22
    print("wait 3s")  # Discard samples while tester vacates the area
    sleep_ms(3000)
    print("Offset 0=", offs0 := p0.fetch())
    print("Offset 1=", offs1 := p1.fetch())
    while True:
        sleep_ms(500)  # Time is not critical
        v0 = p0.fetch(offs0)
        v1 = p1.fetch(offs1)
        # print(v1)
        print("=" * v0)
        print("#" * v1)
        v0 //= 6  # Scaling for neopixel
        v1 //= 6
        for i in range(8):
            n[i] = cmap[int(v0 > i) + 2 * int(v1 > i)]
        n.write()


run()
