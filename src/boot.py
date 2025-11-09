# boot.py
# only show wake source 


import machine

from config import COLORS, NP_PIN, np, set_color

if machine.reset_cause() == machine.DEEPSLEEP_RESET:
    print("Wakker geworden uit deep sleep")
    set_color("GREEN")
elif machine.reset_cause() == machine.WDT_RESET:
    print("Wakker geworden uit WDT reset")
    set_color("YELLOW")
else:
    print("Normale start")
    set_color((5,5,5))