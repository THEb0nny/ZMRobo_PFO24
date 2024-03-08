### Коэффициенты регуляторов
# Синхронизации моторов
SYNC_MOTORS_KP = 0.02
SYNC_MOTORS_KD  = 0

# Выравнивание у стены
WALL_ALIGNMENT_KP = 1.6
WALL_ALIGNMENT_KD = 4

# Выравнивание на линии
LINE_ALIGNMENT_KP = 0.55
LINE_ALIGNMENT_KD = 0.6

# Движение по линии двумя датчиками
LW_2S_KP = 0.5
LW_2S_KD = 0

# Движение по линии левым датчиком
LW_LS_KP = 0.5
LW_LS_KD = 0

# Движение по линии правым датчиком
LW_RS_KP = 0.5
LW_RS_KD = 0

### Номера портов моторов
CHASSIS_LEFT_MOT_PORT = 2  # Разъём левого мотора в шасси
CHASSIS_RIGHT_MOT_PORT = 3  # Разъём правого мотора в шасси
PEN_MANIP_LINEAR_MOTOR_PORT = 1  # Разъём мотора линейного перемещения в манипуляторе маркера
PEN_MANIP_MOTOR_PORT = 4  # Разъём мотора с маркером

### Номера портов сенсоров
LEFT_LIGHT_SEN_PORT = 1  # Левый датчик отражения
RIGHT_LIGHT_SEN_PORT = 4  # Правый датчик отпражения
LEFT_LASER_SEN_PORT = 2  # Левый лазерный датчик
RIGHT_LASER_SEN_PORT = 3  # Правый лазерный датчик
LED_PORT = 5  # Порт модуля лампы

### Сырые значения отражения на белом и чёрном левого и правого датчика отражения
BLACK_REF_RAW_L_LS = 1500
BLACK_REF_RAW_R_LS = 1500
WHITE_REF_RAW_L_LS = 2930
WHITE_REF_RAW_R_LS = 2980

### Другое
WHEELS_D = 56  # Диаметр колёс
WHEELS_W = 140  # Расстояние между центрами колёс (колея)

MOT_ENC_RESOLUTION = 780  # Разрешение энкодеров на оборот

REF_LS_TRESHOLD = 40  # Пороговое значение для определения чёрного для датчиков отражения
REF_LS_LINE_SET_POINT = 50  # Пороговое значение для определения чёрного перекрёсткоф датчиков отражения

DIST_TO_CUBE_SIDE = 165  # Дистанция стенки куба для выравнивания

MOT2_ENC_RANGE = 300  # Тиков энкодера от края до края для мотора для горизонтальной перемещении каретки

LS_WALL_ERR_TRESHOLD = 5  # Пороговое значение, что робот практически достиг уставки в виде расстояния до стены

MOT_ENC_THRESHOLD = 5  # Пороговое значение для определения достижения нужного значения энкодера при движении
TURN_MOT_ENC_THRESHOLD = 50