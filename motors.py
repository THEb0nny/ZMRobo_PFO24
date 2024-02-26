import rcu
import pyb
import tools


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


def SyncChassisMovement(lenght, speed, retention=True):
    prevTime = 0
    # Цикл синхронизации движения колеса к колесу
    while True:
        currTime = pyb.millis()
        dt = currTime - prevTime
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
        tools.PauseUntilTime(currTime, 5)
    
    # Удерживаем моторы, если нужно
    if retention:
        rcu.SetMotorServo(CHASSIS_LEFT_MOT_PORT, speed, 0)
        rcu.SetMotorServo(CHASSIS_RIGHT_MOT_PORT, speed, 0)
    else:
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, 0)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, 0)

# Движение прямолинейно на значение энкодера
def MotorStraightAngle(speed, angle, retention):
    #global MOTOR_STRAIGHT_TIME_OUT

    elm_angle = rcu.GetMotorCode(CHASSIS_LEFT_MOT_PORT) + angle # Значение энкодера левого мотора с нужным углом
    erm_angle = rcu.GetMotorCode(CHASSIS_RIGHT_MOT_PORT) + angle # Значение энкодера правого мотора с нужным углом

    MOT_ENC_ERR_TRESHOLD = 20 # Пороговое значение для определения, что моторы достигли точки
    ELM_ANGL_LEFT_RANGE = elm_angle - MOT_ENC_ERR_TRESHOLD
    ELM_ANGL_RIGHT_RANGE = elm_angle + MOT_ENC_ERR_TRESHOLD
    ELM_ANGL_LEFT_RANGE = erm_angle - MOT_ENC_ERR_TRESHOLD
    ELM_ANGL_RIGHT_RANGE = erm_angle + MOT_ENC_ERR_TRESHOLD

    rcu.SetMotorStraightAngle(CHASSIS_LEFT_MOT_PORT, CHASSIS_RIGHT_MOT_PORT, speed, angle) # Запустить моторы на нужный тик энкодера

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

# Поворот относительно центра на угол
def SpinTurn(deg, speed):
    pass