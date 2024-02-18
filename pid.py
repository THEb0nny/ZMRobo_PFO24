import mymath

class PIDController():

    def __init__(self):
        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.kt = 0.5
        self.b = 1
        self.ulow = 0
        self.uhigh = 0
        self.N = 2
        self.ysp = 0
        self.y = 0
        self.u = 0
        self.reset()

    def reset(self):
        self.I = 0
        self.D = 0

    def setGrains(self, kp, ki, kd, b=1):
        kp = max(0, kp)
        ki = max(0, ki)
        kd = max(0, kd)
        b = mymath.constrain(b, 0, 1)

        selfI += self.kp * (self.b * self.ysp - self.y) - kp * (b * self.ysp - self.y)

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.b = b

    def setControlSaturation(self, low, high):
        self.ulow = low
        self.uhigh = high

    def setDerivativeFilter(self, N):
        self.N = mymath.constrain(self, N, 2, 20)

    def setPoint(self, ysp):
        self.ysp = ysp
    
    def compute(self, timestep, y):
        h = timestep / 1000.0
        K = self.Kp
        #e = self.ysp - y

        P = K * (self.b * self.ysp - y)

        if (self.kD):
            Td = self.kd / K
            ad = (2 * Td - self.N * h) / (2 * Td + self.N * h)
            bd = 2 * K * self.N * Td / (2 * Td + self.N * h)
            self.D = ad * self.D - bd * (y - self.y)

        v = P + self.I + self.D

        u = mymath.constrain(v, self.ulow, self.uhigh) if self.ulow < self.uhigh else v

        if self.ki:
            Ti = K / self.ki
            Tt = self.td * h / Ti
            bi = self.ki * h
            br = h / Tt
            self.I += bi * (self.ysp - y) + br * (u - v)

        self.y = y
        self.u = u

        return self.u