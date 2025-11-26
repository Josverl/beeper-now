import time

import machine

from config import set_color
from device import DEVICE

print(f"Device : {DEVICE}")

# for i in range(5):
#     print(f"Starting in {5 - i} seconds...")
#     if machine.reset_cause() == machine.DEEPSLEEP_RESET:
#         print("Wakker geworden uit deep sleep")
#         set_color("GREEN")
#     elif machine.reset_cause() == machine.WDT_RESET:
#         print("Wakker geworden uit WDT reset")
#         set_color("YELLOW")
#     else:
#         print("Normale start")
#         set_color((5, 5, 5))

#     print(">>>")
#     time.sleep(1)

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
