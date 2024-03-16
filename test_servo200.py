import rcu
import pyb

while True:
    rcu.SetServo(1, 0)
    rcu.SetWaitForTime(2)
    rcu.SetServo(1, 100)
    rcu.SetWaitForTime(2)
    rcu.SetServo(1, 200)
    rcu.SetWaitForTime(2)