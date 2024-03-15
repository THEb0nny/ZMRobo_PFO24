import rcu
import pyb

rcu.SetPump(1, 1)
rcu.SetWaitForTime(5)
rcu.SetPump(1, 0)