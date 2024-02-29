import rcu
import pyb
import tools
import math

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


def SyncChassisMovement(lenght, speed, retention=True):
    import solve_painter as main
    prev_time = 0
    while True: # Цикл синхронизации движения колеса к колесу
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = curr_time
        elm = rcu.GetMotorCode(main.CHASSIS_LEFT_MOT_PORT)
        erm = rcu.GetMotorCode(main.CHASSIS_RIGHT_MOT_PORT)
        if (elm + erm) / 2 >= lenght:
            break
        error_left = erm - elm
        error_right = elm - erm
        lm_speed = speed + error_left * main.SYNC_MOTORS_MOVE_KP
        rm_speed = speed + error_right * main.SYNC_MOTORS_MOVE_KP
        rcu.SetMotor(main.CHASSIS_LEFT_MOT_PORT, lm_speed)
        rcu.SetMotor(main.CHASSIS_RIGHT_MOT_PORT, rm_speed)
        tools.PauseUntilTime(curr_time, 5)
    
    # Удерживаем моторы, если нужно
    ChassisStop(retention)


# Движение прямолинейно на значение энкодера
def MotorStraightAngle(speed, angle, retention=True):
    import solve_painter as main
    MOT_ENC_ERR_TRESHOLD = 15 # Пороговое значение для определения, что моторы достигли точки
    # MOTOR_STRAIGHT_TIME_OUT = 1000

    elm_angle = rcu.GetMotorCode(main.CHASSIS_LEFT_MOT_PORT) + angle # Значение энкодера левого мотора с нужным углом
    erm_angle = rcu.GetMotorCode(main.CHASSIS_RIGHT_MOT_PORT) + angle # Значение энкодера правого мотора с нужным углом
    
    ELM_ANGL_LEFT_RANGE = elm_angle - MOT_ENC_ERR_TRESHOLD
    ELM_ANGL_RIGHT_RANGE = elm_angle + MOT_ENC_ERR_TRESHOLD
    ELM_ANGL_LEFT_RANGE = erm_angle - MOT_ENC_ERR_TRESHOLD
    ELM_ANGL_RIGHT_RANGE = erm_angle + MOT_ENC_ERR_TRESHOLD

    rcu.SetMotorStraightAngle(main.CHASSIS_LEFT_MOT_PORT, main.CHASSIS_RIGHT_MOT_PORT, speed, angle) # Запустить моторы на нужный тик энкодера

    # Ждём достижения проезда
    start_time = pyb.millis()
    while True:
        # if pyb.millis() - start_time >= MOTOR_STRAIGHT_TIME_OUT:
        #     break
        elm = rcu.GetMotorCode(main.CHASSIS_LEFT_MOT_PORT)
        erm = rcu.GetMotorCode(main.CHASSIS_RIGHT_MOT_PORT)
        if ELM_ANGL_LEFT_RANGE <= elm <= ELM_ANGL_RIGHT_RANGE and ELM_ANGL_LEFT_RANGE <= erm <= ELM_ANGL_RIGHT_RANGE:
            break
        rcu.SetWaitForTime(0.005)

    # Удерживаем моторы, если нужно
    ChassisStop(retention)


# Движение на расстояние в мм
def DistMove(dist, speed, retention=True):
    import solve_painter as main
    calc_mot_rotate = (dist / (math.pi * main.WHEELS_D)) * main.MOT_ENC_RESOLUTION  # Расчёт угла поворота на дистанцию
    MotorStraightAngle(speed, calc_mot_rotate, retention)


# Поворот относительно центра на угол
def SpinTurn(deg, speed):
    import solve_painter as main 
    calc_mot_rotate = ((deg * main.WHEELS_W) / main.WHEELS_D) * (main.MOT_ENC_RESOLUTION / 360)  # Расчитать градусы для поворота в градусы для мотора
    rcu.SetCarTurn(main.CHASSIS_LEFT_MOT_PORT, main.CHASSIS_RIGHT_MOT_PORT, speed, calc_mot_rotate)


# Остановка моторов шасси
def ChassisStop(retention, maxSpeed=50):
    import solve_painter as main
    # Удерживаем моторы, если нужно
    if retention:
        rcu.SetMotorServo(main.CHASSIS_LEFT_MOT_PORT, maxSpeed, 0)
        rcu.SetMotorServo(main.CHASSIS_RIGHT_MOT_PORT, maxSpeed, 0)
    else:
        rcu.SetMotor(main.CHASSIS_LEFT_MOT_PORT, 0)
        rcu.SetMotor(main.CHASSIS_RIGHT_MOT_PORT, 0)