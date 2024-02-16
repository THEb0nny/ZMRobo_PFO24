import _thread
import rcu
import math
import mymath

BLACK_REF_RAW_LS1 = 5125
BLACK_REF_RAW_LS4 = 5185

WHITE_REF_RAW_LS1 = 2930
WHITE_REF_RAW_LS4 = 3041

def pauseUntilMotorStalled(motorPort):
    rcu.SetWaitForTime(0.050)
    while True:
        a = rcu.GetMotorCode(motorPort)
        rcu.SetWaitForTime(0.010)
        b = rcu.GetMotorCode(motorPort)
        c = math.abs(math.abs(b) - math.abs(a))
        rcu.SetLCDFilledRectangle2(35, 100, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 100, "c: " + str(c), 0xFFE0, 0x0000, 0)
        if math.abs(math.abs(b) - math.abs(a)) <= 1:
            break

def telemetry():
    while True:
        rrls1 = rcu.GetLightSensor(1) # Считать данные с датчика отражения 1 порта
        rrls4 = rcu.GetLightSensor(4) # Считать данные с датчика отражения 4 порта
        rls1 = mymath.map(rrls1, BLACK_REF_RAW_LS1, WHITE_REF_RAW_LS1, 0, 100)
        rls4 = mymath.map(rrls4, BLACK_REF_RAW_LS4, WHITE_REF_RAW_LS4, 0, 100)
        ls2 = rcu.GetLaserDist(2, 0)
        ls3 = rcu.GetLaserDist(3, 0)
        #em2 = rcu.GetMotorCode(2)
        #rcu.SetLCDClear(0) # Заливка экрана чёрным
        rcu.SetDisplayStringXY(1, 1, "rrls1: " + str(rrls1), 0xFFE0, 0x0000, 0)
        rcu.SetDisplayStringXY(1, 20, "rrls4: " + str(rrls4), 0xFFE0, 0x0000, 0)
        rcu.SetDisplayStringXY(120, 1, "rls1: " + str(rls1), 0xFFE0, 0x0000, 0)
        rcu.SetDisplayStringXY(120, 20, "rls4: " + str(rls4), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 40, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 40, "ls2: " + str(ls2), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 60, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 60, "ls3: " + str(ls3), 0xFFE0, 0x0000, 0)
        #rcu.SetLCDFilledRectangle2(35, 80, 60, 15, 0x0000)
        #rcu.SetDisplayStringXY(1, 80, "em2: " + str(em2), 0xFFE0, 0x0000, 0)
        rcu.SetWaitForTime(0.050)

def solve():
    rcu.SetMotor(2, -30)
    pauseUntilMotorStalled(2)
    rcu.SetWaitForTime(0.5)
    rcu.SetMotor(2, 0)
    rcu.SetWaitForTime(5)

# Главная функция
def main():
    #solve()
    _thread.start_new_thread(telemetry,())
    _thread.start_new_thread(solve,())

main() # Запуск главной функции

while 1:
  pass