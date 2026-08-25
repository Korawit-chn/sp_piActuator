"""mist_trigger.py — fires exactly ONE trigger event, then exits.

Running this once = one trigger = one toggle of the board's P3.1 loop.

    python3 mist_trigger.py          # default pin, D17
    python3 mist_trigger.py D27      # any board.* pin name

The pin is the ONLY thing that is configurable here, and it defaults to D17 so
running the file bare behaves exactly as it did when it was verified by hand.
relay_control.py passes the pin from `GPIO:` in config.txt.

Everything else about this file is load-bearing hardware behaviour. Keep it as
close as possible to what was confirmed working on the real board.
"""

import sys
import time

import board
import digitalio

DEFAULT_PIN = "D17"

name = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PIN

try:
    PIN = getattr(board, name)
except AttributeError:
    # Naming the bad value beats a bare AttributeError traceback, because the
    # value came from a config file the operator wrote.
    raise SystemExit(
        f"unknown GPIO pin {name!r} - expected a board attribute like D17"
    )

pin = digitalio.DigitalInOut(PIN)
pin.direction = digitalio.Direction.OUTPUT

pin.value = True
time.sleep(0.1)
pin.value = False

# No deinit() on purpose — the process exiting is part of what the board reacts
# to, and that's the behaviour you verified. Don't "clean this up".
