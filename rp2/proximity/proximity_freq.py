# proximity_freq.py

# Released under the MIT License (MIT). See LICENSE.
# Copyright (c) 2026 Peter Hinch

# Illustration of proximity detection using a continuously running oscillator
# and measuring frequency. See docs for reasons why this is not preferred.

# The Pin is configured as an output and set high. It is then configured as an
# input and the SM waits until it reads low. A counter (x) is decremented and the
# cycle repeats. At a precise interval a hard timer ISR interrupts this, reading
# the value of X from the SM, resetting X and re-starting the process. The amount
# by which X decreases corresponds to the oscillator frequency, which is
# proportional to 1/C.

# Application code applies an offset to compensate for stray capacitance.

# Pico 2: see errata RP2350-E9: this design requires chip stepping level >= A3
# On earlier stepping SM times out because C never discharges.

import rp2
from machine import Pin, Timer

# Value to seed SM which decrements it inversely proportional to capacitance.
K = const(0x3FFF_FFFF)  # 30 bits (2**30 - 1)


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, out_init=rp2.PIO.OUT_LOW, autopush=True, autopull=True)
def get_cap():
    out(y, 32)  # Wait for count from Python
    wrap_target()
    mov(x, y)  # Reset count
    label("loop")
    set(pindirs, 1)  # Set sense pin to output.
    set(pins, 1)[31]  # Drive it high and charge.
    set(pindirs, 0)  # Change to input
    wait(0, pin, 0)
    jmp(not_osre, "done")  # Python ISR wants data
    jmp(x_dec, "loop")
    set(x, 1)
    jmp("loop")  # Should never get to zero (8.5s without an IRQ)
    label("done")
    in_(x, 30)  # Push count << 2 to Python. FIFO fills and SM stalls until a word is read.
    out(x, 32)  # Wait for trigger from ISR
    wrap()


class Proximity:
    def __init__(self, sm_no=0, pin_no=2, freq=10):
        pin_sense = Pin(pin_no, Pin.IN)  # External pull-down 470KΩ to 4.7MΩ
        self._value = 0
        self._sm = rp2.StateMachine(
            sm_no,
            get_cap,
            freq=125_000_000,
            set_base=pin_sense,  # set pin mapping
            out_base=pin_sense,  # Pindirs mapping
            in_base=pin_sense,
            in_shiftdir=rp2.PIO.SHIFT_RIGHT,
            push_thresh=30,
        )

        self._sm.active(1)
        self._sm.put(K)  # Initialise SM with counter value
        # Timer runs continuously, updating ._value
        self._tim = Timer(freq=freq, mode=Timer.PERIODIC, callback=self._tcb, hard=True)

    # Store an integer proportional to 1/capacitance. Note right shift by 2: this
    # ensures a small int is returned, enabling use in a hard ISR.
    def _tcb(self, _):  # Timer callback: store a frequency sample.
        self._sm.put(0)  # Tell SM to return a value
        self._value = K - self._sm.get(None, 2)

    # ***** API *****
    # If offs == 0 we are assumed to be acquiring an offset value. Otherwise we
    # are applying an offset.
    def fetch(self, offs=0):
        return max(offs - self._value, 0) if offs else self._value

    def deinit(self):
        self._tim.deinit()
        self._sm.active(0)
