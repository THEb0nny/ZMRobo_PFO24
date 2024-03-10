import pid
import rcu
import pyb
import tools
import math
import mymath
import adv_mot_ctrls as adv
import set_var as main


# Пауза до определения стопора мотора
def PauseUntilMotorStalled(motorPort, timeOut=3000):
    stall = 0
    enc_previous = rcu.GetMotorCode(motorPort)
    start_time = pyb.millis()
    rcu.SetWaitForTime(0.050) # Время после которого начинаем проверять, для того, чтобы мотор успел стартануть
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


def SyncAccelerate(minPwr, maxPwr, accelDist, decelDist, totalDist, retention=True):
    adv.AccTwoEncConfig(minPwr, maxPwr, accelDist, decelDist, totalDist)

    pid_acc = pid.PIDController()
    pid_acc.setGrains(main.SYNC_MOTORS_KP, 0, main.SYNC_MOTORS_KD)
    pid_acc.setControlSaturation(-100, 100)
    pid_acc.reset()

    prev_time = 0
    while True:
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = curr_time

        elm = rcu.GetMotorCode(main.CHASSIS_LEFT_MOT_PORT)
        erm = rcu.GetMotorCode(main.CHASSIS_RIGHT_MOT_PORT)
        pwrOut, done = adv.AccTwoEnc(elm, erm)

        if done:
            break

        error = adv.GetErrorSyncMotorsInPwr(elm, erm, pwrOut, pwrOut)
        pid_acc.setPoint(error)
        U = pid_acc.compute(dt, 0)
        pwr_left, pwr_right = adv.GetPwrSyncMotorsInPwr(U, pwrOut, pwrOut)

        rcu.SetMotor(main.CHASSIS_LEFT_MOT_PORT, pwr_left)
        rcu.SetMotor(main.CHASSIS_RIGHT_MOT_PORT, pwr_right)
        tools.PauseUntilTime(curr_time, 5)

    ChassisStop(retention)


# Синхронизированное движение с установленными скоростями
def SyncChassisMovement(speedL, speedR, tics, retention=True):
    adv.SyncMotorsConfig(speedL, speedR)

    elm_prev = rcu.GetMotorCode(main.CHASSIS_LEFT_MOT_PORT)
    erm_prev = rcu.GetMotorCode(main.CHASSIS_RIGHT_MOT_PORT)

    pid_sync = pid.PIDController()
    pid_sync.setGrains(main.SYNC_MOTORS_KP, 0, main.SYNC_MOTORS_KD)
    pid_sync.setControlSaturation(-100, 100)
    pid_sync.reset()

    prev_time = pyb.millis()
    while True:
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = curr_time

        elm = rcu.GetMotorCode(main.CHASSIS_LEFT_MOT_PORT) - elm_prev
        erm = rcu.GetMotorCode(main.CHASSIS_RIGHT_MOT_PORT) - erm_prev

        # if tics - main.MOT_ENC_THRESHOLD <= (elm + erm) / 2 <= tics + main.MOT_ENC_THRESHOLD:
        #     break
        if tics <= (elm + erm) / 2:
            break

        error = adv.GetErrorSyncMotors(elm, erm)
        pid_sync.setPoint(error)
        U = pid_sync.compute(dt, 0)
        lm_speed, rm_speed = adv.GetPwrSyncMotors(U)
        rcu.SetMotor(main.CHASSIS_LEFT_MOT_PORT, lm_speed)
        rcu.SetMotor(main.CHASSIS_RIGHT_MOT_PORT, rm_speed)

        tools.PauseUntilTime(curr_time, 5)

    ChassisStop(retention)

    rcu.Set3CLed(main.LED_PORT, 1)  # Сигнал на лампу, что завершили
    rcu.SetWaitForTime(0.1)
    rcu.Set3CLed(main.LED_PORT, 0)


def SyncChassisTurn(speed, tics, retention=True, debug=False):
    if tics == 0 or speed == 0:
        ChassisStop(retention)
        return

    elm_prev = rcu.GetMotorCode(main.CHASSIS_LEFT_MOT_PORT)
    erm_prev = rcu.GetMotorCode(main.CHASSIS_RIGHT_MOT_PORT)
    if tics < 0:
        adv.SyncMotorsConfig(-speed, speed)
    elif tics > 0:
        adv.SyncMotorsConfig(speed, -speed)

    pid_sync = pid.PIDController()
    pid_sync.setGrains(main.SYNC_MOTORS_KP, 0, main.SYNC_MOTORS_KD)
    pid_sync.setControlSaturation(-100, 100)
    pid_sync.reset()

    prev_time = pyb.millis()
    while True:
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = curr_time

        elm = rcu.GetMotorCode(main.CHASSIS_LEFT_MOT_PORT) - elm_prev
        erm = rcu.GetMotorCode(main.CHASSIS_RIGHT_MOT_PORT) - erm_prev

        error_l = tics - elm
        error_r = tics * -1 - erm
        total_error = -(error_l - error_r)
        error = adv.GetErrorSyncMotorsTurn(elm, erm, tics)
        pid_sync.setPoint(error)

        if math.fabs(total_error) <= main.TURN_MOT_ENC_THRESHOLD:
            break

        U = pid_sync.compute(dt, 0)
        lm_speed, rm_speed = adv.GetPwrSyncMotorsTurn(U, tics)
        lm_speed = mymath.constrain(lm_speed, -speed, speed)
        rm_speed = mymath.constrain(rm_speed, -speed, speed)
        rcu.SetMotor(main.CHASSIS_LEFT_MOT_PORT, lm_speed)
        rcu.SetMotor(main.CHASSIS_RIGHT_MOT_PORT, rm_speed)

        if debug:
            rcu.SetLCDClear(0x0000)
            rcu.SetDisplayStringXY(1, 1, "error_l: " + str(error_l), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 20, "error_r: " + str(error_r), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 40, "total_error: " + str(total_error), 0xFFE0, 0x0000, 0)
            rcu.SetDisplayStringXY(1, 60, "U: " + str(U), 0xFFE0, 0x0000, 0)

        tools.PauseUntilTime(curr_time, 5)

    ChassisStop(retention)


# Движение на расстояние в мм
def DistMove(dist, speed, retention=True):
    calc_mot_rotate = (dist / (math.pi * main.WHEELS_D)) * main.MOT_ENC_RESOLUTION  # Расчёт угла поворота на дистанцию
    # MotorStraightAngle(speed, calc_mot_rotate, retention)
    SyncChassisMovement(speed, speed, calc_mot_rotate)


# Поворот относительно центра на угол
def SpinTurn(deg, speed):
    calc_mot_rotate = ((deg * main.WHEELS_W) / main.WHEELS_D) * (main.MOT_ENC_RESOLUTION / 360)  # Расчитать градусы для поворота в градусы для мотора
    SyncChassisTurn(speed, calc_mot_rotate)


# Остановка моторов шасси
def ChassisStop(retention, maxSpeed=50):
    # Удерживаем моторы, если нужно
    if retention:
        rcu.SetMotorServo(main.CHASSIS_LEFT_MOT_PORT, maxSpeed, 0)
        rcu.SetMotorServo(main.CHASSIS_RIGHT_MOT_PORT, maxSpeed, 0)
    else:
        rcu.SetMotor(main.CHASSIS_LEFT_MOT_PORT, 0)
        rcu.SetMotor(main.CHASSIS_RIGHT_MOT_PORT, 0)