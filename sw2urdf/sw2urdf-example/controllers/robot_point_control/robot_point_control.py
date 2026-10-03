from controller import Supervisor, Keyboard
import math

# ---------- Настройки ----------
TIME_STEP = 32
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4"]
END_EFFECTOR_DEF = "tip"

# Стартовая цель
TARGET = [0.3, 0.0, 0.4]

MAX_ITER  = 40
TOLERANCE = 0.01
DELTA     = 0.05

# Шаг перемещения цели с клавиатуры, м
TARGET_STEP = 0.02

# ---------- Инициализация ----------
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

motors = {}
for n in JOINT_NAMES:
    motors[n] = robot.getDevice(n)
    motors[n].setVelocity(1.0)

end_effector = robot.getFromDef(END_EFFECTOR_DEF)
if end_effector is None:
    print(f"ОШИБКА: не найдено DEF-имя '{END_EFFECTOR_DEF}'")
    exit(1)


# ---------- Маркер цели ----------
def create_or_move_marker():
    node = robot.getFromDef("TARGET_MARKER")
    if node is None:
        root = robot.getRoot()
        children = root.getField("children")
        children.importMFNodeFromString(-1, f"""
        DEF TARGET_MARKER Solid {{
          translation {TARGET[0]} {TARGET[1]} {TARGET[2]}
          children [
            Shape {{
              geometry Sphere {{ radius 0.01 }}
              appearance PBRAppearance {{ baseColor 1 0 0 }}
            }}
          ]
        }}
        """)
    else:
        node.getField("translation").setSFVec3f(TARGET)


create_or_move_marker()


def distance_to_target():
    p = end_effector.getPosition()
    return math.sqrt(
        (p[0] - TARGET[0]) ** 2 +
        (p[1] - TARGET[1]) ** 2 +
        (p[2] - TARGET[2]) ** 2
    )


def step():
    if robot.step(timestep) == -1:
        exit(0)


def handle_keyboard():
    """Читает клавиши и двигает TARGET. Возвращает True, если цель изменилась."""
    changed = False
    key = keyboard.getKey()
    while key != -1:
        ch = chr(key).lower() if 0 < key < 128 else ""
        if ch == "w":
            TARGET[1] += TARGET_STEP
            changed = True
        elif ch == "s":
            TARGET[1] -= TARGET_STEP
            changed = True
        elif ch == "a":
            TARGET[0] -= TARGET_STEP
            changed = True
        elif ch == "d":
            TARGET[0] += TARGET_STEP
            changed = True
        elif ch == "q":
            TARGET[2] += TARGET_STEP
            changed = True
        elif ch == "e":
            TARGET[2] -= TARGET_STEP
            changed = True
        elif ch == "r":
            TARGET[0] = 0.3
            TARGET[1] = 0.0
            TARGET[2] = 0.4
            changed = True
        key = keyboard.getKey()

    if changed:
        create_or_move_marker()
        print(f"Новая цель: [{TARGET[0]:.3f}, {TARGET[1]:.3f}, {TARGET[2]:.3f}]")

    return changed


# ---------- CCD с параллельным опросом клавиатуры ----------
def ccd_step_once():
    """Один полный проход CCD. Возвращает True, если было улучшение."""
    d = distance_to_target()
    improved = False

    for name in reversed(JOINT_NAMES):
        # Проверяем клавиатуру между суставами, чтобы UI не «залипал»
        if handle_keyboard():
            d = distance_to_target()

        motor = motors[name]
        current = motor.getTargetPosition()

        motor.setPosition(current + DELTA)
        step()
        d_plus = distance_to_target()

        motor.setPosition(current - DELTA)
        step()
        d_minus = distance_to_target()

        if d_plus < d and d_plus <= d_minus:
            motor.setPosition(current + DELTA)
            step()
            d = distance_to_target()
            improved = True
        elif d_minus < d:
            motor.setPosition(current - DELTA)
            step()
            d = distance_to_target()
            improved = True
        else:
            motor.setPosition(current)
            step()

    return improved


# ---------- Основной цикл ----------
step()
print("Управление целью: WASD — по X/Y, Q/E — по Z, R — сброс.")
print(f"Стартовая цель: {TARGET}")

while robot.step(timestep) != -1:
    # Если цель сдвинули — перезапускаем CCD до сходимости
    if handle_keyboard():
        pass

    # Несколько итераций CCD за один цикл симуляции
    for _ in range(3):
        improved = ccd_step_once()
        if not improved:
            break