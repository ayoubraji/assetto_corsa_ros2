from vjoy import vj, setJoy
from math import sin, pi
import time

SCALE = 16000
vj.open()

a = input("Listen on steering")
for i in range(0, 6280):
    setJoy(sin(i/1000) + 1,0,0,SCALE)
    time.sleep(0.001)

a = input("Listen on throttle")
for i in range(0,3140):
    setJoy(0,sin(i/1000),0,SCALE)
    time.sleep(0.001)

a = input("Listen on brake")
for i in range(0,3140):
    setJoy(0,0,sin(i/1000),SCALE)
    time.sleep(0.001)

vj.close()