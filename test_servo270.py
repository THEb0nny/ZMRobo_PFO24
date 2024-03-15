import rcu
import pyb

rcu.SetServo(1, 0)
rcu.SetWaitForTime(2)
rcu.SetServo(1, 210)
rcu.SetWaitForTime(5)