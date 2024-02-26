# value: значение для переноса
# fromLow: нижняя граница текущего диапазона
# fromHigh: верхняя граница текущего диапазона
# toLow: нижняя граница нового диапазона, в который переноситься значение
# toHigh: верхняя граница нового диапазона
def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


#x: проверяемое значение, любой тип
#a: нижняя граница области допустимых значений, любой тип
#b: верхняя граница области допустимых значений, любой тип
def constrain(value, min_value, max_value):
    if value < min_value:
        return min_value
    elif value > max_value:
        return max_value
    else:
        return value