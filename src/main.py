import time

from device import DEVICE

print(f"Device : {DEVICE}")

for i in range(5):
    print(f"Starting in {5 - i} seconds...")
    time.sleep(1)

if DEVICE in ["M5STACK-GREY", "C3-SUPER-MINI"]:
    from transmit import init, transmit
    init()
    transmit()
else:
    from receive import init, listen
    init()
    listen()
