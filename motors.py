import rcu
import pyb
import tools
import solve_painter

def PauseUntilMotorStalled(motorPort, timeOut=3000):
    stall = 0
    enc_previous = rcu.GetMotorCode(motorPort)
    start_time = pyb.millis()
    rcu.SetWaitForTime(0.050) # Время после которого начинаем проверять
    while pyb.millis() - start_time < timeOut:
        enc_current = rcu.GetMotorCode(motorPort)
        if abs(enc_current - enc_previous) < 1:
            stall += 1
            if stall > 2:
                break
        else:
            stall = 0
            enc_previous = enc_current
        rcu.SetWaitForTime(0.005) # Ожидание между измерениями


def SyncChassisMovement(lenght: int, speed: int, retention=True):
    prev_time = 0
    while True: # Цикл синхронизации движения колеса к колесу
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = pyb.millis()
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
def MotorStraightAngle(speed:int, angle:int, retention=True):
    # MOTOR_STRAIGHT_TIME_OUT = 1000

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
        # if pyb.millis() - startTime >= MOTOR_STRAIGHT_TIME_OUT:
        #     break
        elm = rcu.GetMotorCode(CHASSIS_LEFT_MOT_PORT)
        erm = rcu.GetMotorCode(CHASSIS_RIGHT_MOT_PORT)
        if ELM_ANGL_LEFT_RANGE <= elm <= ELM_ANGL_RIGHT_RANGE and ELM_ANGL_LEFT_RANGE <= erm <= ELM_ANGL_RIGHT_RANGE:
            break
        rcu.SetWaitForTime(0.005)

    # Удерживаем моторы, если нужно
    if retention:
        rcu.SetMotorServo(port_left_motor, speed, 0)
        rcu.SetMotorServo(port_right_motor, speed, 0)
    else:
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, 0)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, 0)


# Движение на расстояние в мм
def DistMove(dist: int, speed: int, retention=True):
    calc_mot_rotate = (dist / (math.pi * WHEELS_D)) * MOT_ENC_RESOLUTION # Расчёт угла поворота на дистанцию


# Поворот относительно центра на угол
def SpinTurn(deg: float, speed: int):
    calc_mot_rotate = (deg * WHEELS_W) / WHEELS_D # Расчитать градусы для поворота в градусы для мотора
    rcu.SetCarTurn(CHASSIS_LEFT_MOT_PORT, CHASSIS_RIGHT_MOT_PORT, speed, calc_mot_rotate)


# Остановка моторов шасси
def ChassisStop(retention: bool):
    # Удерживаем моторы, если нужно
    if retention:
        rcu.SetMotorServo(CHASSIS_LEFT_MOT_PORT, maxSpeed, 0)
        rcu.SetMotorServo(CHASSIS_RIGHT_MOT_PORT, maxSpeed, 0)
    else:
        rcu.SetMotor(CHASSIS_LEFT_MOT_PORT, 0)
        rcu.SetMotor(CHASSIS_RIGHT_MOT_PORT, 0)