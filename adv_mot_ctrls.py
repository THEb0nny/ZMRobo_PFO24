import math

pwr = 0
syncVLeft, syncVRight = 0, 0
syncVLeftSign, syncVRightSign = 0, 0
acc2_min_pwr, acc2_max_pwr = 0, 0
acc2_accel_dist, acc2_decel_dist, acc2_total_dist = 0, 0, 0
acc2_is_neg = 0

def SyncMotorsConfig(vLeft, vRight):
    global syncVLeft, syncVRight, syncVRightSign, syncVLeftSign
    syncVLeft = vLeft
    syncVRight = vRight
    syncVLeftSign = math.fabs(vLeft + 1) - math.fabs(vLeft)
    syncVRightSign = math.fabs(vRight + 1) - math.fabs(vRight)

def GetErrorSyncMotors(eLeft, eRight):
    return (syncVRight * eLeft) - (syncVLeft * eRight)

def GetPwrSyncMotors(U):
    pwr_left = syncVLeft - syncVRightSign * U
    pwr_right = syncVRight + syncVLeftSign * U
    return pwr_left, pwr_right

def GetErrorSyncMotorsInPwr(eLeft, eRight, vLeft, vRight):
    return (vRight * eLeft) - (vLeft * eRight)

def GetPwrSyncMotorsInPwr(U, vLeft, vRight):
    pwr_left = vLeft - (math.fabs(vRight + 1) - math.fabs(vRight)) * U
    pwr_right = vRight - (math.fabs(vLeft + 1) - math.fabs(vLeft)) * U
    return pwr_left, pwr_right

def AccTwoEncConfig(minPwr, maxPwr, accelDist, decelDist, totalDist):
    global acc2_max_pwr, acc2_min_pwr, acc2_accel_dist, acc2_decel_dist, acc2_total_dist
    acc2_min_pwr = math.fabs(minPwr)
    acc2_max_pwr = math.fabs(maxPwr)
    acc2_accel_dist = accelDist
    acc2_decel_dist = decelDist
    acc2_total_dist = totalDist
    if minPwr < 0:
        acc2_is_neg = 1
    else:
        acc2_is_neg = 0

def AccTwoEnc(elm, erm):
    global pwr

    done = False
    pwrOut = 0
    currEnc = (math.fabs(elm) + math.fabs(erm)) / 2

    if currEnc > acc2_total_dist:
        done = True
    elif currEnc > acc2_total_dist / 2:
        if acc2_decel_dist == 0:
            pwr = acc2_max_pwr
        else:
            pwr = (acc2_max_pwr - acc2_min_pwr) / math.pow(acc2_decel_dist, 2) * math.pow(currEnc - acc2_total_dist, 2) + acc2_min_pwr
        done = False
    else:
        if acc2_accel_dist == 0:
            pwr = acc2_max_pwr
        else:
            pwr = (acc2_max_pwr - acc2_min_pwr) / math.pow(acc2_accel_dist, 2) * math.pow(currEnc - 0, 2) + acc2_min_pwr
        done = False

    if pwr < acc2_min_pwr:
        pwr = acc2_min_pwr
    elif pwr > acc2_max_pwr:
        pwr = acc2_max_pwr

    if acc2_is_neg == 1:
        pwrOut = 0 - pwr
    else:
        pwrOut = pwr

    return pwrOut, done