import time

from device import DEVICE

print(f"Device : {DEVICE}")

for i in range(5):
    print(f"Starting in {5 - i} seconds...")
    time.sleep(1)

if DEVICE in ["C3-MINI", "C3-SUPER-MINI"]:
    # from transmit import init, transmit
    # init()
    # transmit()

    from abeeper import do_beeper_button

    do_beeper_button(timeout_seconds=15)

else:
    from receive import init, listen

    init()
    listen()
