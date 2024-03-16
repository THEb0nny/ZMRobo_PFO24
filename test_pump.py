import rcu
import pyb

while True:
    rcu.SetPump(1, 1)
    rcu.SetWaitForTime(5)
    rcu.SetPump(1, 0)
    rcu.SetWaitForTime(2)