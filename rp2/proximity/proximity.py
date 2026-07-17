# proximity.py

# Released under the MIT License (MIT). See LICENSE.
# Copyright (c) 2025 Peter Hinch

# The timer constantly populates a ring buffer, at a rate of ~500 samples/sec.
# Application code takes a reading by taking mean of buffer contents. An offset is
# applied to compensate for stray capacitance.
# Sampling frequency is chosen to ensure buffer holds an integer number of mains cycles.

# Pico 2: see errata RP2350-E9: this design requires chip stepping level >= A3
# On earlier stepping SM times out because C never discharges.

import rp2
from machine import Pin, Timer
from array import array

# Value to seed SM which decrements it proportional to capacitance.
K = const(25_000)  # Actual value is pretty arbitrary.
# Array size corresponds to 25 mains cycles. See freq constructor arg.
NSAMPLES = const(250)


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, out_init=rp2.PIO.OUT_LOW, autopush=True, autopull=True)
def get_cap():
    out(y, 32)  # Wait for count from Python
    wrap_target()
    label("full_scale")
    set(pindirs, 1)  # Set sense pin to output.
    set(pins, 1)[31]  # Drive it high and charge.
    in_(x, 30)  # Push count << 2 to Python. FIFO fills and SM stalls until a word is read.
    mov(x, y)  # Reset count
    set(pindirs, 0)  # Change to input
    label("loop")  # Wait for it to drift low
    jmp(x_dec, "next")  # Jump unless it's timed out
    set(x, 0)  # Return 0 on timeout (maximum time)
    jmp("full_scale")
    label("next")  # x was nonzero
    jmp(pin, "loop")  # Loop until sense pin reads low
    wrap()


def indx(i=0):  # Yield index values modulo NSAMPLES
    while True:
        yield (i := i - 1 if i else NSAMPLES - 1)


# freq should be 10*line frequency: fits 25 cycles in 250 sample buffer
class Proximity:
    def __init__(self, sm_no=0, pin_no=2, freq=500):
        pin_sense = Pin(pin_no, Pin.IN)  # External pull-down 470KΩ to 4.7MΩ
        self._a = array("h", (0 for _ in range(NSAMPLES)))
        self._idx = indx()

        self._sm = rp2.StateMachine(
            sm_no,
            get_cap,
            freq=125_000_000,
            set_base=pin_sense,  # set pin mapping
            out_base=pin_sense,  # Pindirs mapping
            jmp_pin=pin_sense,
            in_shiftdir=rp2.PIO.SHIFT_RIGHT,
            push_thresh=30,
        )

        self._sm.active(1)
        self._sm.put(K)  # Initialise SM with counter value
        # Timer runs continuously, populating the buffer with samples
        self._tim = Timer(freq=freq, mode=Timer.PERIODIC, callback=self._tcb, hard=True)

    # Store an integer proportional to capacitance. Note right shift by 2: this
    # ensures a small int is returned, enabling use in a hard ISR.
    def _tcb(self, _):  # Timer callback: get a sample and put in buffer
        # Duration 95μs: 2% overhead @500Hz
        self._a[next(self._idx)] = K - self._sm.get(None, 2)

    # ***** API *****
    # Very crude signal processing averages over N samples. Use of a moving average
    # rather than an FIR filter avoids any need to synchronise fetching and populating.
    def fetch(self, offs=0):
        return max(sum(self._a) // NSAMPLES - offs, 0)

    def deinit(self):
        self._tim.deinit()
        self._sm.active(0)
