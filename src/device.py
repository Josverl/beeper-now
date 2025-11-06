from net_id import MAC_ADDRESS

ALL_DEVICES = {
    "C3-SUPER-MINI": b'\x8c\xd0\xb2\xa8\x7d\x37',
    "C3-MINI": b'\x18\x8b\x0e\x03\x1b\x54',
    "C3-AI": b'\x34\x85\x18\x02\x8c\xf8',
    "M5STACK-FIRE": b'\x84\x0d\x8e\x25\x98\xb4',
    "M5STACK-GREY": b'\x30\xae\xa4\x58\x37\x90',
}
"""Device names mapping to MAC Addresses """


def get_device_name(mac_address):
    """
    Lookup device name by MAC address.
    Returns the device name (key) if found, otherwise "UNKNOWN".
    """
    for device_name, device_mac in ALL_DEVICES.items():
        if device_mac == mac_address:
            return device_name
    return "UNKNOWN"


DEVICE = get_device_name(MAC_ADDRESS)
