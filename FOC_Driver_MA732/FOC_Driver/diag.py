import serial, time, sys
import Jetson.GPIO as G
PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyTHS1'
P2 = bytes.fromhex('FFFFFD00FE0300013142')   # Protocol 2.0 broadcast ping
P1 = bytes.fromhex('FFFF010201FB')           # Protocol 1.0 ping ID1
G.setwarnings(False); G.setmode(G.BOARD); G.setup(7, G.OUT, initial=0)
for inv in (0, 1):
    tx, rx = (0, 1) if inv else (1, 0)
    for b in [57600, 1000000, 115200, 9600, 2000000, 3000000]:
        s = serial.Serial(PORT, b, timeout=0)
        for name, pk in (('P2', P2), ('P1', P1)):
            G.output(7, rx); time.sleep(0.01); s.reset_input_buffer()
            G.output(7, tx); s.write(pk); s.flush(); G.output(7, rx)
            time.sleep(0.15); r = s.read(256)
            print(f"inv={inv} baud={b:>7} {name}: {r.hex(' ') if r else '-'}")
        s.close()
G.cleanup()
