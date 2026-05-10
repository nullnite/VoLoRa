import pylink
import time

def connect():
    jlink = pylink.JLink()
    jlink.open()
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect("nRF5340_xxAA")
    jlink.rtt_start()
    time.sleep(0.5)
    print("[connected]")
    return jlink

jlink = None
while True:
    try:
        if jlink is None:
            jlink = connect()

        data = jlink.rtt_read(0, 1024)
        if data:
            print(bytes(data).decode("utf-8", errors="replace"), end="", flush=True)
        time.sleep(0.05)

    except Exception:
        print("[disconnected, retrying...]")
        try:
            jlink.close()
        except Exception:
            pass
        jlink = None
        time.sleep(2)
