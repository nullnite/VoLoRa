import pylink
import time

jlink = pylink.JLink()
jlink.open()
jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
jlink.connect("nRF5340_xxAA")
jlink.rtt_start()
time.sleep(0.5)

while True:
    data = jlink.rtt_read(0, 1024)
    if data:
        print(bytes(data).decode("utf-8", errors="replace"), end="")
    time.sleep(0.05)
