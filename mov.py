import numpy as np
import matplotlib.pyplot as plt

# Estructura Point
class Point:
    def __init__(self, x, y):
        self.Xcord = x
        self.Ycord = y

# Parámetros de simulación
lookahead_dist = 5
base_speed = 1.0
turn_k = 0.1

def generateBezierPath(points, control_dist=100, steps=50):
    if len(points) < 2:
        return []

    path = []
    for i in range(len(points) - 1):
        P0, P3 = points[i], points[i + 1]
        # Definir las direcciones de los puntos intermedios para los controles
        if i == 0:
            sh = np.radians(0)  # Ángulo de inicio
        else:
            sh = np.radians(45)  # Control del ángulo intermedio (para visualización, ajustable)

        if i == len(points) - 2:
            eh = np.radians(90)  # Ángulo final
        else:
            eh = np.radians(45)  # Ángulo intermedio (para visualización, ajustable)

        P1 = Point(P0.Xcord + np.cos(sh) * control_dist, P0.Ycord + np.sin(sh) * control_dist)
        P2 = Point(P3.Xcord - np.cos(eh) * control_dist, P3.Ycord - np.sin(eh) * control_dist)

        for t in range(steps + 1):
            t /= steps
            x = (1 - t)**3 * P0.Xcord + 3 * (1 - t)**2 * t * P1.Xcord + 3 * (1 - t) * t**2 * P2.Xcord + t**3 * P3.Xcord
            y = (1 - t)**3 * P0.Ycord + 3 * (1 - t)**2 * t * P1.Ycord + 3 * (1 - t) * t**2 * P2.Ycord + t**3 * P3.Ycord
            path.append(Point(x, y))
    return path

def distanceTo(a, b):
    return np.hypot(b.Xcord - a.Xcord, b.Ycord - a.Ycord)

def normalizeAngle(angle):
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi
    return angle

def getLookahead(current, path):
    for point in path:
        if distanceTo(current, point) > lookahead_dist:
            return point
    return path[-1]

def simulatePurePursuit(path, initial_pos, initial_heading_deg):
    x, y = initial_pos.Xcord, initial_pos.Ycord
    heading = np.radians(initial_heading_deg)
    positions = [Point(x, y)]

    for _ in range(1000):
        current = Point(x, y)
        target = getLookahead(current, path)
        dx = target.Xcord - x
        dy = target.Ycord - y

        target_angle = np.arctan2(dy, dx)
        angle_error = normalizeAngle(target_angle - heading)

        turn = turn_k * angle_error
        left_speed = base_speed - turn
        right_speed = base_speed + turn
        avg_speed = (left_speed + right_speed) / 2

        x += avg_speed * np.cos(heading) * 0.5
        y += avg_speed * np.sin(heading) * 0.5
        heading += turn * 0.5

        positions.append(Point(x, y))

        if distanceTo(current, path[-1]) < 5:
            break

    return positions

# Variables para puntos seleccionados
points = []  # Lista de puntos que forman el camino

fig, ax = plt.subplots()
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_title("Click izquierdo: agregar puntos | Click derecho: reiniciar")
ax.set_aspect('equal', adjustable='box')
ax.grid(True)

def on_click(event):
    global points

    if event.button == 1:  # Click izquierdo
        new_point = Point(event.xdata, event.ydata)
        points.append(new_point)
        ax.plot(new_point.Xcord, new_point.Ycord, 'bo', markersize=10)
        print(f"Punto agregado: ({new_point.Xcord:.2f}, {new_point.Ycord:.2f})")
        
        # Si se tienen al menos 2 puntos, dibujar la trayectoria
        if len(points) > 1:
            draw_path()
        
    elif event.button == 3:  # Click derecho
        print("Reiniciando puntos...")
        reset_all()

    plt.draw()

def draw_path():
    if len(points) < 2:
        return  # No se puede generar un camino con menos de 2 puntos

    # Recálculo de la curva Bézier con los puntos actuales
    path = generateBezierPath(points)
    sim = simulatePurePursuit(path, points[0], 0)

    # Limpiar el gráfico y volver a dibujar la curva y simulación
    ax.clear()
    ax.set_xlim(-120, 120)
    ax.set_ylim(-120, 120)
    ax.set_title("Click izquierdo: agregar puntos | Click derecho: reiniciar")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

    ax.plot([p.Xcord for p in path], [p.Ycord for p in path], 'k--', label='Curva Bézier')
    ax.plot([p.Xcord for p in sim], [p.Ycord for p in sim], 'b-', label='Ruta del robot')
    ax.legend()

def reset_all():
    global points
    points = []
    ax.clear()
    ax.set_xlim(-120, 120)
    ax.set_ylim(-120, 120)
    ax.set_title("Click izquierdo: agregar puntos | Click derecho: reiniciar")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

# Conectar clics
fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()
