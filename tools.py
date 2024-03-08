import rcu
import pyb
import mymath


def PauseUntilTime(startTime, ms):
    if startTime == 0:
        startTime = pyb.millis()
    waitCompletionTime = startTime + ms
    while pyb.millis() < waitCompletionTime:
        pass


def GetNormRefLineSensor(refRawVal, bRefRawVal, wRefRawValCS):
    refValue = mymath.map(refRawVal, bRefRawVal, wRefRawValCS, 0, 100)
    refValue = mymath.constrain(refValue, 0, 100)
    return refValue


# Функция для печати значений на экран
def Telemetry():
    global BLACK_REF_RAW_L_LS, WHITE_REF_RAW_L_LS, BLACK_REF_RAW_R_LS, WHITE_REF_RAW_R_LS
    global LEFT_LIGHT_SEN_PORT, RIGHT_LIGHT_SEN_PORT
    global LEFT_LASER_SEN_PORT, RIGHT_LASER_SEN_PORT
    global CHASSIS_LEFT_MOT_PORT, CHASSIS_RIGHT_MOT_PORT, PEN_MANIP_LINEAR_MOTOR_PORT
    prev_time = 0
    while True:
        curr_time = pyb.millis()
        dt = curr_time - prev_time
        prev_time = pyb.millis()
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
        tools.PauseUntilTime(currTime, 50)