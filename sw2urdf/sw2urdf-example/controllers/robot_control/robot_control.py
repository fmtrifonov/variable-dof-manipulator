from controller import Robot, Keyboard

# Шаг времени симуляции (совпадает с basicTimeStep в WorldInfo)
TIME_STEP = 32

robot = Robot()
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

# Получаем моторы и датчики по именам из URDF
motors = {}
sensors = {}
for name in ["joint_1", "joint_2", "joint_3", "joint_4"]:
    motors[name] = robot.getDevice(name)
    sensors[name] = robot.getDevice(name + "_sensor")
    sensors[name].enable(TIME_STEP)

# Начальные углы (радианы)
angles = {name: 0.0 for name in motors}

# Шаг изменения угла при нажатии клавиши
STEP = 0.05

print("Управление:")
print("  1 / 2 — joint_1 (влево/вправо)")
print("  3 / 4 — joint_2")
print("  5 / 6 — joint_3")
print("  7 / 8 — joint_4")
print("  R     — сбросить все углы в 0")

while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()

    if key == -1:
        continue

    # Преобразуем код клавиши в символ
    ch = chr(key).lower() if 0 < key < 128 else ""

    if ch == "1":
        angles["joint_1"] += STEP
    elif ch == "2":
        angles["joint_1"] -= STEP
    elif ch == "3":
        angles["joint_2"] += STEP
    elif ch == "4":
        angles["joint_2"] -= STEP
    elif ch == "5":
        angles["joint_3"] += STEP
    elif ch == "6":
        angles["joint_3"] -= STEP
    elif ch == "7":
        angles["joint_4"] += STEP
    elif ch == "8":
        angles["joint_4"] -= STEP
    elif ch == "r":
        for name in angles:
            angles[name] = 0.0

    # Отправляем новые углы в моторы
    for name, angle in angles.items():
        motors[name].setPosition(angle)