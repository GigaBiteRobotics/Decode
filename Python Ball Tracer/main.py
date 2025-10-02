import math
import pygame
import tkinter as tk
from tkinter import ttk
import threading

# Physics constants (will be set by sliders)
GRAVITY = 9.81  # m/s^2
drag = 0.0      # drag coefficient

# Pygame setup
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)

# Initial positions (in meters, but mapped to pixels)
cannon_pos = [100, HEIGHT - 100]
target_pos = [600, HEIGHT - 300]

# Parameters (will be set by sliders)
initial_speed = 60.0  # m/s

def calculate_elevation(cannon, target, speed, gravity):
    dx = (target[0] - cannon[0]) / 10.0  # scale pixels to meters
    dy = (cannon[1] - target[1]) / 10.0  # scale pixels to meters
    g = gravity
    v = speed
    part = v**4 - g * (g * dx**2 + 2 * dy * v**2)
    if part < 0:
        return None  # No solution
    if dx < 0:
        return None  # Vertical shot, undefined angle
    angle1 = math.atan((v**2 + math.sqrt(part)) / (g * dx))
    angle2 = math.atan((v**2 - math.sqrt(part)) / (g * dx))
    return math.degrees(angle1), math.degrees(angle2)

def get_trajectory(cannon, angle_deg, speed, gravity, drag):
    points = []
    angle_rad = math.radians(angle_deg)
    v_x = speed * math.cos(angle_rad)
    v_y = speed * math.sin(angle_rad)
    x0, y0 = cannon
    t = 0
    while True:
        # Apply drag: v = v0 * exp(-drag * t)
        v_x_drag = v_x * math.exp(-drag * t)
        v_y_drag = v_y * math.exp(-drag * t)
        x = x0 + v_x_drag * t * 10  # scale meters to pixels
        y = y0 - (v_y_drag * t - 0.5 * gravity * t**2) * 10
        if x > WIDTH or y > HEIGHT:
            break
        if y >= HEIGHT:
            break
        points.append((int(x), int(y)))
        t += 0.05
    return points

# Tkinter GUI for sliders
def start_tk():
    def update_values(*args):
        global initial_speed, GRAVITY, drag
        initial_speed = speed_var.get()
        GRAVITY = gravity_var.get()
        drag = drag_var.get()

    root = tk.Tk()
    root.title("Ball Physics Controls")
    speed_var = tk.DoubleVar(value=60.0)
    gravity_var = tk.DoubleVar(value=9.81)
    drag_var = tk.DoubleVar(value=0.0)
    speed_var.trace_add('write', update_values)
    gravity_var.trace_add('write', update_values)
    drag_var.trace_add('write', update_values)
    ttk.Label(root, text="Speed (m/s)").pack()
    ttk.Scale(root, from_=10, to=100, variable=speed_var, orient='horizontal').pack(fill='x')
    ttk.Label(root, text="Gravity (m/s²)").pack()
    ttk.Scale(root, from_=1, to=20, variable=gravity_var, orient='horizontal').pack(fill='x')
    ttk.Label(root, text="Drag (0-0.2)").pack()
    ttk.Scale(root, from_=0, to=0.2, variable=drag_var, orient='horizontal').pack(fill='x')
    root.mainloop()

tk_thread = threading.Thread(target=start_tk, daemon=True)
tk_thread.start()

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Cannon Elevation & Trajectory")
clock = pygame.time.Clock()

running = True
dragging = False

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()
            if abs(mx - target_pos[0]) < 20 and abs(my - target_pos[1]) < 20:
                dragging = True
        elif event.type == pygame.MOUSEBUTTONUP:
            dragging = False

    if dragging:
        mx, my = pygame.mouse.get_pos()
        target_pos = [mx, my]

    screen.fill(WHITE)
    pygame.draw.circle(screen, BLUE, cannon_pos, 15)
    pygame.draw.circle(screen, RED, target_pos, 15)
    pygame.draw.line(screen, BLACK, cannon_pos, target_pos, 2)

    result = calculate_elevation(cannon_pos, target_pos, initial_speed, GRAVITY)
    if result:
        angle1, angle2 = result
        traj_points = get_trajectory(cannon_pos, angle1, initial_speed, GRAVITY, drag)
        pygame.draw.lines(screen, GREEN, False, traj_points, 2)
        font = pygame.font.SysFont(None, 24)
        txt = font.render(f"Elevation: {angle1:.2f}° (speed {initial_speed:.1f} m/s, gravity {GRAVITY:.2f}, drag {drag:.3f})", True, BLACK)
        screen.blit(txt, (10, 10))
        txt3 = font.render("Use sliders to change speed, gravity, drag. Drag red target.", True, BLACK)
        screen.blit(txt3, (10, 70))
        eq_font = pygame.font.SysFont(None, 20)
        eq1 = eq_font.render("Angle: θ = atan((v² ± sqrt(v⁴ - g(gx² + 2yv²))) / (g x))", True, BLACK)
        eq2 = eq_font.render("Trajectory: x = x₀ + vₓ t, y = y₀ + v_y t - 0.5 g t²", True, BLACK)
        screen.blit(eq1, (10, 100))
        screen.blit(eq2, (10, 120))
    else:
        font = pygame.font.SysFont(None, 24)
        txt = font.render("No solution: target out of range for current speed.", True, RED)
        screen.blit(txt, (10, 10))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()