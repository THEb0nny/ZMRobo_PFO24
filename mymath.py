# value: значение для переноса
# fromLow: нижняя граница текущего диапазона
# fromHigh: верхняя граница текущего диапазона
# toLow: нижняя граница нового диапазона, в который переноситься значение
# toHigh: верхняя граница нового диапазона
def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min