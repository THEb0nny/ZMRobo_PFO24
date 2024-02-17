import _thread as thread
import rcu
import mymath
import time

BLACK_REF_RAW_LS1 = 5125
BLACK_REF_RAW_LS4 = 5185

WHITE_REF_RAW_LS1 = 2930
WHITE_REF_RAW_LS4 = 3041

REF_VAL_LS_TRESHOLD = 40

barcode_bin_array = [[1, 0], [0, 1], [1, 1], [0, 1]]

def pauseUntilMotorStalled(motorPort, timeout=10):
    start = time.time()
    rcu.SetWaitForTime(0.050)
    previous = rcu.GetMotorCode(motorPort)
    stall = 0
    while time.time() - start < start + timeout:
        current = rcu.GetMotorCode(motorPort)
        #rcu.SetLCDFilledRectangle2(35, 180, 60, 15, 0x0000)
        #rcu.SetDisplayStringXY(1, 180, "c: " + str(abs(current - previous)), 0xFFE0, 0x0000, 0)
        if abs(current - previous) < 1:
            stall += 1
            if stall > 2:
                break # not moving
        else:
            stall = 0
            previous = current
        rcu.SetWaitForTime(0.005)

# Функция, которая возвращает число с двоичной комбинации
def convBinary2DecimalCode(binArr):
    decimalNumArr = [] # Переменная для хранения результата
    for k in range(4):
        decimalNum = 0
        for i, j in range(0, 2), range(1, -1, -1):
            decimal_num += binArr[k][i] * 2 ** j
        decimalNumArr[k] = decimalNum

    return decimalNumArr # Вернуть массив чисел

# Функция для печати значений на экран
def telemetry():
    while True:
        rrls1 = rcu.GetLightSensor(1) # Считать данные с датчика отражения 1 порта
        rrls4 = rcu.GetLightSensor(4) # Считать данные с датчика отражения 4 порта
        rls1 = mymath.map(rrls1, BLACK_REF_RAW_LS1, WHITE_REF_RAW_LS1, 0, 100)
        rls4 = mymath.map(rrls4, BLACK_REF_RAW_LS4, WHITE_REF_RAW_LS4, 0, 100)
        ls2 = rcu.GetLaserDist(2, 0)
        ls3 = rcu.GetLaserDist(3, 0)
        em1 = rcu.GetMotorCode(1)
        em2 = rcu.GetMotorCode(2)
        em3 = rcu.GetMotorCode(3)
        em4 = rcu.GetMotorCode(4)
        #rcu.SetLCDClear(0) # Заливка экрана чёрным
        rcu.SetDisplayStringXY(1, 1, "rrls1: " + str(rrls1), 0xFFE0, 0x0000, 0)
        rcu.SetDisplayStringXY(1, 20, "rrls4: " + str(rrls4), 0xFFE0, 0x0000, 0)
        rcu.SetDisplayStringXY(120, 1, "rls1: " + str(rls1), 0xFFE0, 0x0000, 0)
        rcu.SetDisplayStringXY(120, 20, "rls4: " + str(rls4), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 40, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 40, "ls2: " + str(ls2), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 60, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 60, "ls3: " + str(ls3), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 80, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 80, "em1: " + str(em1), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 100, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 100, "em4: " + str(em4), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 120, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 120, "em2: " + str(em2), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 140, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 140, "em3: " + str(em3), 0xFFE0, 0x0000, 0)
        rcu.SetWaitForTime(0.050)

def readBarCode():
    # Считать штрихкод
    for i in range(4):
        rrls1 = rcu.GetLightSensor(1) # Считать данные с датчика отражения 1 порта
        rrls4 = rcu.GetLightSensor(4) # Считать данные с датчика отражения 4 порта
        rls1 = mymath.map(rrls1, BLACK_REF_RAW_LS1, WHITE_REF_RAW_LS1, 0, 100)
        rls4 = mymath.map(rrls4, BLACK_REF_RAW_LS4, WHITE_REF_RAW_LS4, 0, 100)
        # Левый датчик
        if rls1 > REF_VAL_LS_TRESHOLD:
            barcode_bin_array[i][0] = 0 # Белый
        else:
            barcode_bin_array[i][0] = 1 # Чёрный
        # Правый датчик
        if rls4 > REF_VAL_LS_TRESHOLD:
            barcode_bin_array[i][1] = 0 # Белый
        else:
            barcode_bin_array[i][1] = 1 # Чёрный
        rcu.SetWaitForTime(0.1); # Задержка
        if i < 3:
            pass
            #DistMove(25, 20) # Движение на следующий участок для считывания

# Функция решения задачи
def solve():
    rcu.SetMotor(2, -30)
    pauseUntilMotorStalled(2, 1)
    rcu.SetWaitForTime(0.5)
    rcu.SetMotor(2, 0)
    rcu.SetMotorStraight(1, 4, 20)
    while rcu.GetMotorCode(1) < 360 and rcu.GetMotorCode(4) < 360:
        pass
    rcu.SetMotor(1, 0)
    rcu.SetMotor(4, 0)
    #rcu.SetMotorStraightAngle(1, 4, 20, 360)
    #rcu.SetWaitForAngle(1, 20, 360)
    #rcu.SetWaitForAngle(4, 20, 360)
    #rcu.SetMotor(1, 0)
    #rcu.SetMotor(4, 0)

    #readBarCode()

    #result = convBinary2DecimalCode(barcode_bin_array) # Узнаём и записываем в переменную число от штрихкода

    rcu.SetWaitForTime(5)

# Главная функция
def main():
    #solve()
    thread.start_new_thread(telemetry,())
    thread.start_new_thread(solve,())

    while True:
        pass

main() # Запуск главной функции