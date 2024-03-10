from set_var import *
import _thread as thread
import time

import rcu
import pyb
import math
import mymath
import tools
import motors
import pid

bar_side = [[3, 1], [1, 3]]

# Считать штрихкод
def ReadBarCode():
    global bar_side
    for i in range(2):
        bar1 = ''
        bar2 = ''
        for j in range(2):
            lrrls = rcu.GetLightSensor(LEFT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 1 порта
            rrrls = rcu.GetLightSensor(RIGHT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 4 порта
            lrls = mymath.map(lrrls, BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS, 0, 100)
            rrls = mymath.map(rrrls, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS, 0, 100)
            # Левый датчик
            if lrls > REF_LS_TRESHOLD:
                bar1 += '0'  # Белый
            else:
                bar1 += '1'  # Чёрный
            # Правый датчик
            if rrls > REF_LS_TRESHOLD:
                bar2 += '0'  # Белый
            else:
                bar2 += '1'
            rcu.SetWaitForTime(0.1)  # Задержка
            if i < 3:
                motors.DistMove(25, 20) # Движение на следующий участок для считывания
        bar_side.append([int(bar1, 2), int(bar2, 2)])


# Движение по линии до перекрёстка
def LineFollowToIntersaction(speed, retention=True, debug=False):
    pid_line = pid.PIDController()
    pid_line.setGrains(LW_2S_KP, 0, LW_2S_KD)
    pid_line.setControlSaturation(-100, 100)
    pid_line.reset()

    prev_time = 0
    while True:
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = curr_time
        lrrls = rcu.GetLightSensor(LEFT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 1 порта
        rrrls = rcu.GetLightSensor(RIGHT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 4 порта
        lrls = tools.GetNormRefLineSensor(lrrls, BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS)
        rrls = tools.GetNormRefLineSensor(rrrls, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS)
        if lrls < REF_LS_TRESHOLD and\
                rrls < REF_LS_TRESHOLD:
            break
        error = lrls - rrls
        pid_line.setPoint(error)
        u = pid_line.compute(dt, 0)
        pwr_left_mot = speed + u
        pwr_right_mot = speed - u
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, pwr_left_mot)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, pwr_right_mot)

        if debug:
            rcu.SetLCDClear(0x0000)
            rcu.SetDisplayStringXY(1, 1, "lrls: " + str(lrls), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 20, "rrls: " + str(rrls), 0xFFE0, 0x0000, 0)

        tools.PauseUntilTime(curr_time, 1)

    motors.ChassisStop(retention)  # Команда остановки моторов шасси

    rcu.Set3CLed(LED_PORT, 1) # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(LED_PORT, 0) # Сигнал на лампу, что завершили


# Движение по линии на расстояние
def LineFollowToDist(dist, speed, retention=True, debug=False):
    elm_prev = rcu.GetMotorCode(CHASSIS_LEFT_MOT_PORT)  # Значение с левого энкодера изначально
    erm_prev = rcu.GetMotorCode(CHASSIS_RIGHT_MOT_PORT)  # Значение с правого энкодера изначально
    angle = (dist / (math.pi * WHEELS_D)) * MOT_ENC_RESOLUTION  # Расчёт угла поворота на дистанцию
    # Сколько нужно пройти моторам включая накрученное до этого
    lm_rotate = angle + elm_prev  # Считаем для левого мотора сколько в итоге нужно повернуться
    rm_rotate = angle + erm_prev  # Считаем для правого мотора сколько в итоге нужно повернуться
    
    pid_line = pid.PIDController()
    pid_line.setGrains(LW_2S_KP, 0, LW_2S_KD)
    pid_line.setControlSaturation(-100, 100)
    pid_line.reset()

    prev_time = 0
    while True:
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = curr_time
        lrrls = rcu.GetLightSensor(LEFT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 1 порта
        rrrls = rcu.GetLightSensor(RIGHT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 4 порта
        lrls = tools.GetNormRefLineSensor(lrrls, BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS)
        rrls = tools.GetNormRefLineSensor(rrrls, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS)
        elm = rcu.GetMotorCode(CHASSIS_LEFT_MOT_PORT)
        erm = rcu.GetMotorCode(CHASSIS_RIGHT_MOT_PORT)
        if elm >= lm_rotate and\
                erm >= rm_rotate:
            break
        error = lrls - rrls
        pid_line.setPoint(error)
        u = pid_line.compute(dt, 0)
        pwr_left_mot = speed + u
        pwr_right_mot = speed - u
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, pwr_left_mot)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, pwr_right_mot)

        if debug:
            rcu.SetLCDClear(0x0000)
            rcu.SetDisplayStringXY(1, 1, "lm_rotate: " + str(lm_rotate), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 20, "rm_rotate: " + str(rm_rotate), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 40, "elm: " + str(elm), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 60, "erm: " + str(erm), 0xFFE0, 0x0000, 0)

        tools.PauseUntilTime(curr_time, 5)

    motors.ChassisStop(retention)  # Команда остановки моторов шасси


# Движение по линии до перекрёстка
def LineFollowToRighIntersaction(speed, retention=True, debug=False):
    pid_line = pid.PIDController()
    pid_line.setGrains(LW_LS_KP, 0, LW_LS_KD)
    pid_line.setControlSaturation(-100, 100)
    pid_line.reset()

    prev_time = 0
    while True:
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = curr_time
        lrrls = rcu.GetLightSensor(LEFT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 1 порта
        rrrls = rcu.GetLightSensor(RIGHT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 4 порта
        lrls = tools.GetNormRefLineSensor(lrrls, BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS)
        rrls = tools.GetNormRefLineSensor(rrrls, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS)
        if REF_LS_LINE_SET_POINT - 30 <= lrls <= REF_LS_LINE_SET_POINT + 30 and\
                rrls < REF_LS_TRESHOLD:
            rcu.Set3CLed(LED_PORT, 1)
            rcu.SetWaitForTime(0.05)
            rcu.Set3CLed(LED_PORT, 0)
            break
        error = lrls - REF_LS_LINE_SET_POINT
        pid_line.setPoint(error)
        u = pid_line.compute(dt, 0)
        pwr_left_mot = speed + u
        pwr_right_mot = speed - u
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, pwr_left_mot)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, pwr_right_mot)

        if debug:
            rcu.SetLCDClear(0x0000)
            rcu.SetDisplayStringXY(1, 1, "lrls: " + str(lrls), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 20, "rrls: " + str(rrls), 0xFFE0, 0x0000, 0)

        tools.PauseUntilTime(curr_time, 5)

    motors.ChassisStop(retention)  # Команда остановки моторов шасси

    rcu.Set3CLed(LED_PORT, 1)  # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(LED_PORT, 0)  # Сигнал на лампу, что завершили


# Выравнивание на линии перпендикулярно
def LineAlignment(maxSpeed=50, alignmentTime=1000, lineIsForward=True, retention=True, timeOut=4000, debug=False):
    pid_left = pid.PIDController()
    pid_right = pid.PIDController()
    pid_left.setGrains(LINE_ALIGNMENT_KP, 0, LINE_ALIGNMENT_KD)
    pid_right.setGrains(LINE_ALIGNMENT_KP, 0, LINE_ALIGNMENT_KD)
    pid_left.setControlSaturation(-100, 100)
    pid_right.setControlSaturation(-100, 100)
    pid_left.reset()
    pid_right.reset()

    multiplier = 1 if lineIsForward else -1  # lineIsForward - линия спереди, иначе сзади

    prev_time = 0
    start_time = pyb.millis()
    while pyb.millis() - start_time < timeOut:
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = curr_time
        lrrls = rcu.GetLightSensor(LEFT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 1 порта
        rrrls = rcu.GetLightSensor(RIGHT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 4 порта
        lrls = tools.GetNormRefLineSensor(lrrls, BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS)
        rrls = tools.GetNormRefLineSensor(rrrls, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS)
        error_l = lrls - REF_LS_LINE_SET_POINT
        error_r = rrls - REF_LS_LINE_SET_POINT
        pid_left.setPoint(error_l)
        pid_right.setPoint(error_r)
        u_left = pid_left.compute(dt, 0) * multiplier
        u_right = pid_right.compute(dt, 0) * multiplier
        u_left = mymath.constrain(u_left, -maxSpeed, maxSpeed)
        u_right = mymath.constrain(u_right, -maxSpeed, maxSpeed)
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, u_left)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, u_right)

        if debug:
            rcu.SetLCDClear(0x0000)
            rcu.SetDisplayStringXY(1, 1, "lrls: " + str(lrls), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 20, "rrls: " + str(rrls), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 40, "error_l: " + str(error_l), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 60, "error_r: " + str(error_r), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 80, "u_left: " + str(u_left), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 100, "u_right: " + str(u_right), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 120, "dt: " + str(dt), 0xFFE0, 0x0000, 0)

        tools.PauseUntilTime(curr_time, 5)

    motors.ChassisStop(retention)  # Команда остановки моторов шасси


# Выравнивание у стенки лазерными датчиками
def WallAlignment(distanceToWall, maxSpeed=50, regulationTime=1000, retention=True, timeOut=5000, debug=False):
    pid_left = pid.PIDController()
    pid_right = pid.PIDController()
    pid_left.setGrains(WALL_ALIGNMENT_KP, 0, WALL_ALIGNMENT_KD)
    pid_right.setGrains(WALL_ALIGNMENT_KP, 0, WALL_ALIGNMENT_KD)
    pid_left.setControlSaturation(-100, 100)
    pid_right.setControlSaturation(-100, 100)
    pid_left.reset()
    pid_right.reset()

    if timeOut < regulationTime:
        timeOut = regulationTime
    dereg_flag = False

    prev_time = 0
    start_time = pyb.millis()
    while pyb.millis() - start_time < timeOut:
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = curr_time
        lls = rcu.GetLaserDist(LEFT_LASER_SEN_PORT, 0)
        rls = rcu.GetLaserDist(RIGHT_LASER_SEN_PORT, 0)
        error_l = lls - distanceToWall
        error_r = rls - distanceToWall
        if not (dereg_flag) and\
                abs(error_l) <= LS_WALL_ERR_TRESHOLD and\
                abs(error_r) <= LS_WALL_ERR_TRESHOLD:
            dereg_flag = True
            dereg_start_time = pyb.millis()
        if dereg_flag and\
                pyb.millis() - dereg_start_time >= regulationTime:
            break
        pid_left.setPoint(error_l)
        pid_right.setPoint(error_r)
        u_left = pid_left.compute(dt, 0)
        u_right = pid_right.compute(dt, 0)
        u_left = mymath.constrain(u_left, -maxSpeed, maxSpeed)
        u_right = mymath.constrain(u_right, -maxSpeed, maxSpeed)
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, u_left)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, u_right)

        if debug:
            rcu.SetLCDClear(0x0000)
            rcu.SetDisplayStringXY(1, 0, "lls: " + str(lls), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 20, "lsr: " + str(rls), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 40, "dt: " + str(dt), 0xFFE0, 0x0000, 0)

        tools.PauseUntilTime(curr_time, 5)

    motors.ChassisStop(retention)  # Команда остановки моторов шасси


def ResetMotors():
    rcu.SetMotor(PEN_MANIP_MOTOR_PORT, 40)
    motors.PauseUntilMotorStalled(PEN_MANIP_MOTOR_PORT)
    rcu.SetMotor(PEN_MANIP_MOTOR_PORT, 0)
    rcu.SetWaitForTime(0.01)
    rcu.SetMotorCode(PEN_MANIP_MOTOR_PORT)  # Сбросить энкодер мотора в линейном перемещении манипулятора
    rcu.Set3CLed(LED_PORT, 2)  # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(LED_PORT, 0)  # Сигнал на лампу, что завершили

    ### Мотор линейного перемещения манипулятора установить в крайнее левое положение и сбросить
    rcu.SetMotor(PEN_MANIP_LINEAR_MOTOR_PORT, -50)
    motors.PauseUntilMotorStalled(PEN_MANIP_LINEAR_MOTOR_PORT)
    rcu.SetMotor(PEN_MANIP_LINEAR_MOTOR_PORT, 0)
    rcu.SetWaitForTime(0.01)
    rcu.SetMotorCode(PEN_MANIP_LINEAR_MOTOR_PORT)  # Сбросить энкодер мотора в линейном перемещении манипулятора
    rcu.Set3CLed(LED_PORT, 1)  # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(LED_PORT, 0)  # Сигнал на лампу, что завершили

    ### Вращение мотора в режиме сервопривода вверх
    rcu.SetMotorServo(PEN_MANIP_MOTOR_PORT, -50, 105)
    rcu.SetWaitForAngle(PEN_MANIP_MOTOR_PORT, -50, 105)


def MoveManipLinear(position):
    if position == 0: # Положение маркера влево
        rcu.SetWaitForAngle(PEN_MANIP_LINEAR_MOTOR_PORT, 50, 0)
    if position == 1: # Положение маркера в центр
        rcu.SetWaitForAngle(PEN_MANIP_LINEAR_MOTOR_PORT, 50, 150)
    if position == 2: # Положение маркера вправо
        rcu.SetWaitForAngle(PEN_MANIP_LINEAR_MOTOR_PORT, 50, 150)
    rcu.SetLCDClear(0x0000)
    rcu.SetDisplayStringXY(1, 1, "enc: " + str(rcu.GetMotorCode(PEN_MANIP_LINEAR_MOTOR_PORT)), 0xFFE0, 0x0000, 0)
    rcu.Set3CLed(LED_PORT, 1)  # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(LED_PORT, 0)  # Сигнал на лампу, что завершили


# Функция решения задачи
def Solve():
    ### Мотор маркера манипулятора установить в крайнее положение и сбросить
    ResetMotors()
    # rcu.SetMotorStraight(1, 4, 20)
    # motors.SyncChassisMove(360, 20, True)
    # motors.MotorStraightAngle(25, 360, True)
    # rcu.Set3CLed(LED_PORT, 1) # Сигнал на лампу, что завершили
    # rcu.SetWaitForTime(0.1)
    # rcu.Set3CLed(LED_PORT, 0) # Сигнал на лампу, что завершили

    ### Выравнивание у стороны куба
    WallAlignment(DIST_TO_CUBE_SIDE, 40, regulationTime=3000, debug=False)
    rcu.Set3CLed(LED_PORT, 3)  # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(LED_PORT, 0)  # Сигнал на лампу, что завершили

    ### Чертим линии
    rcu.SetMotorServo(PEN_MANIP_MOTOR_PORT, 50, 60)
    rcu.SetWaitForAngle(PEN_MANIP_MOTOR_PORT, 50, 60)

    rcu.SetWaitForTime(0.5)

    WallAlignment(DIST_TO_CUBE_SIDE + 40, 40, regulationTime=500, debug=False)

    rcu.SetMotorServo(PEN_MANIP_MOTOR_PORT, -50, 60)
    rcu.SetWaitForAngle(PEN_MANIP_MOTOR_PORT, -50, 60)

    # ReadBarCode() # Считать штрих код с листка
    # result = ConvBinary2DecimalCode(barcode_bin_array) # Узнаём и записываем в переменную число от штрихкода

def CalibrateLineSensor(start_t):
    lrrls = rcu.GetLightSensor(LEFT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 1 порта
    rrrls = rcu.GetLightSensor(RIGHT_LIGHT_SEN_PORT)  # Считать данные с датчика отражения 4 порта
    lrls = tools.GetNormRefLineSensor(lrrls, BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS)
    rrls = tools.GetNormRefLineSensor(rrrls, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS)
    rcu.SetLCDClear(0x0000)
    rcu.SetDisplayStringXY(1, 1, "lrls: " + str(lrls), 0xFFE0, 0x0000, 0)
    rcu.SetDisplayStringXY(1, 20, "rrls: " + str(rrls), 0xFFE0, 0x0000, 0)
    rcu.SetDisplayStringXY(150, 1, "lrrls: " + str(lrrls), 0xFFE0, 0x0000, 0)
    rcu.SetDisplayStringXY(150, 20, "rrrls: " + str(rrrls), 0xFFE0, 0x0000, 0)
    time.sleep(0.01)

# Главная функция
def Main():
    bar_side.sort()
    LineFollowToIntersaction(30)
    motors.DistMove(30, 30)
    if bar_side[0][0] != 0:
        motors.SpinTurn(-90, 30)
        LineFollowToDist(100, 30)
        for i in range(2):
            for j in range(bar_side[i][0]*2):
                if j % 2 == 0:
                    LineFollowToRighIntersaction(30)
                    motors.DistMove(30, 30)
                    motors.SpinTurn(90, 30)
                    LineFollowToDist(100, 30)
                else:
                    LineFollowToIntersaction(30)
                    motors.DistMove(30, 30)
            motors.SpinTurn(90, 40)
            WallAlignment(DIST_TO_CUBE_SIDE)
            motors.DistMove(-50, -30)
            motors.SpinTurn(-90, 30)
            LineFollowToDist(100, 30)
    else:
        LineFollowToIntersaction(-30)
        motors.DistMove(30, 30)
        motors.SpinTurn(-90, 30)

    # rcu.Set3CLed(LED_PORT, 1)
    # rcu.SetWaitForTime(0.1)
    # rcu.Set3CLed(LED_PORT, 0)
    # motors.DistMove(25, 30)

    # motors.SpinTurn(90, 25)
    # rcu.Set3CLed(LED_PORT, 1)
    # rcu.SetWaitForTime(0.1)
    # rcu.Set3CLed(LED_PORT, 0)
    # LineFollowToDist(100, 30)
    # rcu.Set3CLed(LED_PORT, 1)
    # rcu.SetWaitForTime(0.1)
    # rcu.Set3CLed(LED_PORT, 0)
    # thread.start_new_thread(tools.Telemetry,())
    # thread.start_new_thread(Solve,())
    start_t = pyb.millis()
    while True:
        # CalibrateLineSensor(start_t)
        pass


Main()  # Запуск главной функции