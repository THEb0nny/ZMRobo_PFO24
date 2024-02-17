import _thread as thread
import rcu
import mymath
import time

BLACK_REF_RAW_LS1 = 5125
BLACK_REF_RAW_LS4 = 5185

WHITE_REF_RAW_LS1 = 2930
WHITE_REF_RAW_LS4 = 3041

REF_VAL_LS_TRESHOLD = 40

MOT2_ENC_RANGE = 300 # Тиков энкодера от края до края для мотора для горизонтальной перемещении каретки

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
        
def current_milli_time():
    return round(time.time() * 1000)

def robotMove(lenght, speed):
    Kp = 0.6

    while True:
        em1 = rcu.GetMotorCode(1)
        em4 = rcu.GetMotorCode(4)
        if (em1 + em4) / 2 >= lenght:
            break
        error_left = em4 - em1
        error_right = em1 - em4
        m1_speed = speed + error_left * Kp
        m4_speed = speed + error_right * Kp
        rcu.SetMotor(1, m1_speed)
        rcu.SetMotor(4, m4_speed)
        #rcu.SetWaitForTime(0.01)

    Kp = 0.8
    ENC_ERR_TRESHOLD = 10

    dereg_time = 200
    deregulation = False
    while True:
        current_time = current_milli_time()
        rcu.SetLCDFilledRectangle2(35, 180, 100, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 180, "ct: " + str(current_time), 0xFFE0, 0x0000, 0)
        em1 = rcu.GetMotorCode(1)
        em4 = rcu.GetMotorCode(4)
        error_left = lenght - em1
        error_right = lenght - em4
        if not(deregulation) and abs(error_left) <= ENC_ERR_TRESHOLD and abs(error_right) <= ENC_ERR_TRESHOLD:
            deregulation = True
            dereg_time += current_milli_time()
            rcu.SetLCDFilledRectangle2(35, 200, 100, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 200, "dst: " + str(dereg_time), 0xFFE0, 0x0000, 0)
        if deregulation and current_milli_time() >= dereg_time:
            break
        m1_speed = error_left * Kp
        m4_speed = error_right * Kp
        m1_speed = mymath.constrain(m1_speed, -30, 30)
        m4_speed = mymath.constrain(m4_speed, -30, 30)
        rcu.SetMotor(1, m1_speed)
        rcu.SetMotor(4, m4_speed)
        #rcu.SetWaitForTime(0.01)
    
    rcu.SetMotor(1, 0)
    rcu.SetMotor(4, 0)

# Функция решения задачи
def solve():
    rcu.SetMotor(2, -40)
    pauseUntilMotorStalled(2, 1)
    rcu.SetWaitForTime(0.01)
    rcu.SetMotor(2, 0)
    rcu.SetMotorCode(2)
    #rcu.SetMotorStraight(1, 4, 20)
    #rcu.SetMotorStraightAngle(1, 4, 20, 360)
    #robotMove(360, 30)

    #readBarCode()

    #result = convBinary2DecimalCode(barcode_bin_array) # Узнаём и записываем в переменную число от штрихкода

    rcu.SetWaitForTime(5)

# Главная функция
def main():
    thread.start_new_thread(telemetry,())
    thread.start_new_thread(solve,())

    while True:
        pass

main() # Запуск главной функции