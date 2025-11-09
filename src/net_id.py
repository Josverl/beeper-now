import network

# retrieve the MAC address
MAC_ADDRESS: bytes = network.WLAN(network.STA_IF).config("mac")
