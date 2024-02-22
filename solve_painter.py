import _thread as thread
import rcu
import pyb
import mymath
import pid


### Коэффициенты регуляторов
SYNC_MOTORS_MOVE_KP = 0.8

WALL_ALIGNMENT_KP = 1.6
WALL_ALIGNMENT_KD = 4

LINE_ALIGNMENT_KP = 0.55
LINE_ALIGNMENT_KD = 0.6

### Номера портов моторов
CHASSIS_LEFT_MOT_PORT = 2 # Разъём левого мотора в шасси
CHASSIS_RIGHT_MOT_PORT = 3 # Разъём правого мотора в шасси
PEN_MANIP_LINEAR_MOTOR_PORT = 1 # Разъём мотора линейного перемещения в манипуляторе маркера
PEN_MANIP_MOTOR_PORT = 4 # Разъём мотора с маркером

### Номера портов сенсоров
LEFT_LIGHT_SEN_PORT = 1 # Левый датчик отражения
RIGHT_LIGHT_SEN_PORT = 4 # Правый датчик отпражения
LEFT_LASER_SEN_PORT = 2 # Левый лазерный датчик
RIGHT_LASER_SEN_PORT = 3 # Правый лазерный датчик
LED_PORT = 5 # Порт модуля лампы

### Сырые значения отражения на белом и чёрном левого и правого датчика отражения
BLACK_REF_RAW_L_LS = 484
BLACK_REF_RAW_R_LS = 585
WHITE_REF_RAW_L_LS = 2903
WHITE_REF_RAW_R_LS = 3048

### Другое
MOTOR_STRAIGHT_TIME_OUT = 1000

REF_LS_TRESHOLD = 50 # Пороговое значение для определения чёрного для датчиков отражения

DIST_TO_CUBE_SIDE = 165 # Дистанция стенки куба для выравнивания

MOT2_ENC_RANGE = 300 # Тиков энкодера от края до края для мотора для горизонтальной перемещении каретки

barcode_bin_array = [[1, 0], [0, 1], [1, 1], [0, 1]]

# Функция, которая возвращает число с двоичной комбинации
def ConvBinary2DecimalCode(binArr):
    decimalNumArr = [] # Переменная для хранения результата
    for k in range(4):
        decimalNum = 0
        for i, j in range(0, 2), range(1, -1, -1):
            decimal_num += binArr[k][i] * 2 ** j
        decimalNumArr[k] = decimalNum

    return decimalNumArr # Вернуть массив чисел

# Функция для печати значений на экран
def Telemetry():
    global BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS
    while True:
        lrrls = rcu.GetLightSensor(LEFT_LIGHT_SEN_PORT) # Считать данные с датчика отражения 1 порта
        rrrls = rcu.GetLightSensor(RIGHT_LIGHT_SEN_PORT) # Считать данные с датчика отражения 4 порта
        lrls = mymath.map(lrrls, BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS, 0, 100)
        rrls = mymath.map(rrrls, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS, 0, 100)
        lrls = mymath.constrain(lrls, 0, 100)
        rrls = mymath.constrain(rrls, 0, 100)
        lls = rcu.GetLaserDist(LEFT_LASER_SEN_PORT, 0)
        rls = rcu.GetLaserDist(RIGHT_LASER_SEN_PORT, 0)
        elm = rcu.GetMotorCode(CHASSIS_LEFT_MOT_PORT)
        erm = rcu.GetMotorCode(CHASSIS_RIGHT_MOT_PORT)
        em1 = rcu.GetMotorCode(PEN_MANIP_LINEAR_MOTOR_PORT)
        em4 = rcu.GetMotorCode(3)
        #rcu.SetLCDClear(0) # Заливка экрана чёрным
        rcu.SetLCDFilledRectangle2(1, 1, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 1, "lrrls: " + str(lrrls), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(1, 20, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 20, "rrrls: " + str(rrrls), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(120, 1, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(120, 1, "lrls: " + str(lrls), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(120, 20, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(120, 20, "rrls: " + str(rrls), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 40, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 40, "lls: " + str(lls), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 60, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 60, "rls: " + str(rls), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 80, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 80, "elm: " + str(elm), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 100, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 100, "erm: " + str(erm), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 120, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 120, "em2: " + str(em1), 0xFFE0, 0x0000, 0)
        rcu.SetLCDFilledRectangle2(35, 140, 60, 15, 0x0000)
        rcu.SetDisplayStringXY(1, 140, "em3: " + str(em4), 0xFFE0, 0x0000, 0)
        rcu.SetWaitForTime(0.050)

def ReadBarCode():
    # Считать штрихкод
    for i in range(4):
        rrls1 = rcu.GetLightSensor(LEFT_LIGHT_SEN_PORT) # Считать данные с датчика отражения 1 порта
        rrls4 = rcu.GetLightSensor(RIGHT_LIGHT_SEN_PORT) # Считать данные с датчика отражения 4 порта
        rls1 = mymath.map(rrls1, BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS, 0, 100)
        rls4 = mymath.map(rrls4, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS, 0, 100)
        # Левый датчик
        if rls1 > REF_LS_TRESHOLD:
            barcode_bin_array[i][0] = 0 # Белый
        else:
            barcode_bin_array[i][0] = 1 # Чёрный
        # Правый датчик
        if rls4 > REF_LS_TRESHOLD:
            barcode_bin_array[i][1] = 0 # Белый
        else:
            barcode_bin_array[i][1] = 1 # Чёрный
        rcu.SetWaitForTime(0.1); # Задержка
        if i < 3:
            pass
            #DistMove(25, 20) # Движение на следующий участок для считывания

def LineAlignment(maxSpeed = 50, alignmentTime = 1000, lineIsForward = True, retention=True, timeOut=5000, debug=False):
    pid_left = pid.PIDController()
    pid_right = pid.PIDController()
    pid_left.setGrains(LINE_ALIGNMENT_KP, 0, LINE_ALIGNMENT_KD)
    pid_right.setGrains(LINE_ALIGNMENT_KP, 0, LINE_ALIGNMENT_KD)
    pid_left.setControlSaturation(-100, 100)
    pid_right.setControlSaturation(-100, 100)
    pid_left.reset()
    pid_right.reset()
    deregFlag = False
    prevTime = 0
    multiplier = 1 if lineIsForward else -1 # lineIsForward - линия спереди, иначе сзади
    startTime = pyb.millis()
    while pyb.millis() - startTime < timeOut:
        currTime = pyb.millis()
        loopTime = currTime - prevTime
        prevTime = pyb.millis()
        lrrls = rcu.GetLightSensor(LEFT_LIGHT_SEN_PORT) # Считать данные с датчика отражения 1 порта
        rrrls = rcu.GetLightSensor(RIGHT_LIGHT_SEN_PORT) # Считать данные с датчика отражения 4 порта
        lrls = mymath.map(lrrls, BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS, 0, 100)
        rrls = mymath.map(rrrls, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS, 0, 100)
        lrls = mymath.constrain(lrls, 0, 100)
        rrls = mymath.constrain(rrls, 0, 100)
        error_l = lrls - REF_LS_TRESHOLD
        error_r = rrls - REF_LS_TRESHOLD
        pid_left.setPoint(error_l)
        pid_right.setPoint(error_r)
        u_left = pid_left.compute(loopTime, 0) * multiplier
        u_right = pid_right.compute(loopTime, 0) * multiplier
        u_left = mymath.constrain(u_left, -maxSpeed, maxSpeed)
        u_right = mymath.constrain(u_right, -maxSpeed, maxSpeed)
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, u_left)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, u_right)

        if debug:
            rcu.SetLCDFilledRectangle2(50, 1, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 1, "lrrls: " + str(lrrls), 0xFFE0, 0x0000, 0)
            rcu.SetLCDFilledRectangle2(50, 20, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 20, "rrrls: " + str(rrrls), 0xFFE0, 0x0000, 0)
            rcu.SetLCDFilledRectangle2(160, 1, 100, 15, 0x0000)
            rcu.SetDisplayStringXY(120, 1, "lrls: " + str(lrls), 0xFFE0, 0x0000, 0)
            rcu.SetLCDFilledRectangle2(160, 20, 100, 15, 0x0000)
            rcu.SetDisplayStringXY(120, 20, "rrls: " + str(rrls), 0xFFE0, 0x0000, 0)

            rcu.SetLCDFilledRectangle2(1, 40, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 40, "error_l: " + str(error_l), 0xFFE0, 0x0000, 0)
            rcu.SetLCDFilledRectangle2(1, 60, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 60, "error_r: " + str(error_r), 0xFFE0, 0x0000, 0)
            rcu.SetLCDFilledRectangle2(1, 80, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 80, "u_left: " + str(u_left), 0xFFE0, 0x0000, 0)
            rcu.SetLCDFilledRectangle2(1, 100, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 100, "u_right: " + str(u_right), 0xFFE0, 0x0000, 0)

            rcu.SetLCDFilledRectangle2(35, 120, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 120, "dt: " + str(loopTime), 0xFFE0, 0x0000, 0)
        
        rcu.SetWaitForTime(0.01)

    # Удерживаем моторы, если нужно
    if retention:
        rcu.SetMotorServo(CHASSIS_LEFT_MOT_PORT, maxSpeed, 0)
        rcu.SetMotorServo(CHASSIS_RIGHT_MOT_PORT, maxSpeed, 0)
    else:
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, 0)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, 0)


def WallAlignment(distanceToWall, maxSpeed=50, regulationTime=1000, retention=True, timeOut=5000, debug=False):
    LS_ERR_TRESHOLD = 5 # Пороговое значение, что робот практически достиг уставки
    pid_left = pid.PIDController()
    pid_right = pid.PIDController()
    pid_left.setGrains(WALL_ALIGNMENT_KP, 0, WALL_ALIGNMENT_KD)
    pid_right.setGrains(WALL_ALIGNMENT_KP, 0, WALL_ALIGNMENT_KD)
    pid_left.setControlSaturation(-100, 100)
    pid_right.setControlSaturation(-100, 100)
    pid_left.reset()
    pid_right.reset()
    deregFlag = False
    prevTime = 0
    startTime = pyb.millis()
    while pyb.millis() - startTime < timeOut:
        currTime = pyb.millis()
        loopTime = currTime - prevTime
        prevTime = pyb.millis()
        lls = rcu.GetLaserDist(LEFT_LASER_SEN_PORT, 0)
        rls = rcu.GetLaserDist(RIGHT_LASER_SEN_PORT, 0)
        error_l = lls - distanceToWall
        error_r = rls - distanceToWall
        if not(deregFlag) and abs(error_l) <= LS_ERR_TRESHOLD and abs(error_r) <= LS_ERR_TRESHOLD:
            deregFlag = True
            deregStartTime = pyb.millis()
        if deregFlag and pyb.millis() - deregStartTime >= regulationTime:
            break
        pid_left.setPoint(error_l)
        pid_right.setPoint(error_r)
        u_left = pid_left.compute(loopTime, 0)
        u_right = pid_right.compute(loopTime, 0)
        u_left = mymath.constrain(u_left, -maxSpeed, maxSpeed)
        u_right = mymath.constrain(u_right, -maxSpeed, maxSpeed)
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, u_left)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, u_right)

        if debug:
            rcu.SetLCDFilledRectangle2(35, 0, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 0, "lls: " + str(lls), 0xFFE0, 0x0000, 0)
            rcu.SetLCDFilledRectangle2(35, 20, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 20, "lsr: " + str(rls), 0xFFE0, 0x0000, 0)
            rcu.SetLCDFilledRectangle2(35, 40, 60, 15, 0x0000)
            rcu.SetDisplayStringXY(1, 40, "dt: " + str(loopTime), 0xFFE0, 0x0000, 0)

        rcu.SetWaitForTime(0.01)

    # Удерживаем моторы, если нужно
    if retention:
        rcu.SetMotorServo(CHASSIS_LEFT_MOT_PORT, maxSpeed, 0)
        rcu.SetMotorServo(CHASSIS_RIGHT_MOT_PORT, maxSpeed, 0)
    else:
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, 0)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, 0)

def SyncChassisMove(lenght, speed, retention=True):
    prevTime = 0
    # Цикл синхронизации движения колеса к колесу
    while True:
        currTime = pyb.millis()
        loopTime = currTime - prevTime
        prevTime = pyb.millis()
        elm = rcu.GetMotorCode(CHASSIS_LEFT_MOT_PORT)
        erm = rcu.GetMotorCode(CHASSIS_RIGHT_MOT_PORT)
        if (elm + erm) / 2 >= lenght:
            break
        error_left = erm - elm
        error_right = elm - erm
        lm_speed = speed + error_left * SYNC_MOTORS_MOVE_KP
        rm_speed = speed + error_right * SYNC_MOTORS_MOVE_KP
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, lm_speed)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, rm_speed)
        rcu.SetWaitForTime(0.01)
    
    # Удерживаем моторы, если нужно
    if retention:
        rcu.SetMotorServo(CHASSIS_LEFT_MOT_PORT, speed, 0)
        rcu.SetMotorServo(CHASSIS_RIGHT_MOT_PORT, speed, 0)
    else:
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, 0)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, 0)

def MotorStraightAngle(port_left_motor, port_right_motor, speed, angle, retention):
    #global MOTOR_STRAIGHT_TIME_OUT

    elm_angle = rcu.GetMotorCode(CHASSIS_LEFT_MOT_PORT) + angle # Значение энкодера левого мотора с нужным углом
    erm_angle = rcu.GetMotorCode(CHASSIS_RIGHT_MOT_PORT) + angle # Значение энкодера правого мотора с нужным углом

    MOT_ENC_ERR_TRESHOLD = 20 # Пороговое значение для определения, что моторы достигли точки
    ELM_ANGL_LEFT_RANGE = elm_angle - MOT_ENC_ERR_TRESHOLD
    ELM_ANGL_RIGHT_RANGE = elm_angle + MOT_ENC_ERR_TRESHOLD
    ELM_ANGL_LEFT_RANGE = erm_angle - MOT_ENC_ERR_TRESHOLD
    ELM_ANGL_RIGHT_RANGE = erm_angle + MOT_ENC_ERR_TRESHOLD

    rcu.SetMotorStraightAngle(port_left_motor, port_right_motor, speed, angle) # Запустить моторы на нужный тик энкодера

    # Ждём достижения проезда
    startTime = pyb.millis()
    while True:
        if pyb.millis() - startTime >= MOTOR_STRAIGHT_TIME_OUT:
            break
        if ELM_ANGL_LEFT_RANGE <= rcu.GetMotorCode(CHASSIS_LEFT_MOT_PORT) <= ELM_ANGL_RIGHT_RANGE and ELM_ANGL_LEFT_RANGE <= rcu.GetMotorCode(CHASSIS_RIGHT_MOT_PORT) <= ELM_ANGL_RIGHT_RANGE:
            break
        rcu.SetWaitForTime(0.005)

    rcu.Set3CLed(LED_PORT, 4) # Сигнал на лампу, что завершили

    # Удерживаем моторы, если нужно
    if retention:
        rcu.SetMotorServo(port_left_motor, speed, 0)
        rcu.SetMotorServo(port_right_motor, speed, 0)
    else:
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, 0)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, 0)

def PauseUntilMotorStalled(motorPort, timeOut=3000):
    startTime = pyb.millis()
    rcu.SetWaitForTime(0.050) # Время после которого начинаем проверять
    previous = rcu.GetMotorCode(motorPort)
    stall = 0
    while pyb.millis() - startTime < timeOut:
        current = rcu.GetMotorCode(motorPort)
        if abs(current - previous) < 1:
            stall += 1
            if stall > 2:
                break
        else:
            stall = 0
            previous = current
        rcu.SetWaitForTime(0.005) # Ожидание между измерениями

# Функция решения задачи
def Solve():
    ### Мотор маркера манипулятора установить в крайнее положение и сбросить
    rcu.SetMotor(PEN_MANIP_MOTOR_PORT, 40)
    PauseUntilMotorStalled(PEN_MANIP_MOTOR_PORT)
    rcu.SetMotor(PEN_MANIP_MOTOR_PORT, 0)
    rcu.SetWaitForTime(0.01)
    rcu.SetMotorCode(PEN_MANIP_MOTOR_PORT) # Сбросить энкодер мотора в линейном перемещении манипулятора
    rcu.Set3CLed(LED_PORT, 2) # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(LED_PORT, 0) # Сигнал на лампу, что завершили

    ### Мотор линейного перемещения манипулятора установить в крайнее левое положение и сбросить
    rcu.SetMotor(PEN_MANIP_LINEAR_MOTOR_PORT, -40)
    PauseUntilMotorStalled(PEN_MANIP_LINEAR_MOTOR_PORT)
    rcu.SetMotor(PEN_MANIP_LINEAR_MOTOR_PORT, 0)
    rcu.SetWaitForTime(0.01)
    rcu.SetMotorCode(PEN_MANIP_LINEAR_MOTOR_PORT) # Сбросить энкодер мотора в линейном перемещении манипулятора
    rcu.Set3CLed(LED_PORT, 1) # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(LED_PORT, 0) # Сигнал на лампу, что завершили

    #rcu.SetMotorStraight(1, 4, 20)
    #SyncChassisMove(360, 20, True)
    #MotorStraightAngle(CHASSIS_LEFT_MOT_PORT, CHASSIS_RIGHT_MOT_PORT, 25, 360, True)
    #rcu.Set3CLed(LED_PORT, 1) # Сигнал на лампу, что завершили
    #rcu.SetWaitForTime(0.1)
    #rcu.Set3CLed(LED_PORT, 0) # Сигнал на лампу, что завершили

    ### Вращение мотора в режиме сервопривода вверх
    rcu.SetMotorServo(PEN_MANIP_MOTOR_PORT, -50, 105)
    rcu.SetWaitForAngle(PEN_MANIP_MOTOR_PORT, -50, 105)

    ### Выравнивание у стороны куба
    WallAlignment(DIST_TO_CUBE_SIDE, 40, regulationTime=3000, debug=False)
    rcu.Set3CLed(LED_PORT, 3) # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(LED_PORT, 0) # Сигнал на лампу, что завершили
    
    ### Чертим линии
    rcu.SetMotorServo(PEN_MANIP_MOTOR_PORT, 50, 60)
    rcu.SetWaitForAngle(PEN_MANIP_MOTOR_PORT, 50, 60)

    rcu.SetWaitForTime(0.5)

    WallAlignment(DIST_TO_CUBE_SIDE + 40, 40, regulationTime=500, debug=False)

    rcu.SetMotorServo(PEN_MANIP_MOTOR_PORT, -50, 60)
    rcu.SetWaitForAngle(PEN_MANIP_MOTOR_PORT, -50, 60)

    #ReadBarCode() # Считать штрих код с листка
    #result = ConvBinary2DecimalCode(barcode_bin_array) # Узнаём и записываем в переменную число от штрихкода

# Главная функция
def Main():
    #LineAlignment(40, 3000, True)
    Solve()
    #thread.start_new_thread(Telemetry,())
    #thread.start_new_thread(Solve,())

    while True:
        pass

Main() # Запуск главной функции