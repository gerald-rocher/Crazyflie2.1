from controller import Supervisor
import random
import math
import numpy as np

# Crée une instance du superviseur
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Get the drone by its DEF name
drone = supervisor.getFromDef("CRAZYFLIE")

if drone is None:
    print("⚠️ Could not find the drone (DEF CRAZYFLIE). Check your .wbt file.")
else:
    print("✅ Dynamic wind initialized for CRAZYFLIE.")

# Wind parameters
base_strength = 0.05     # mean wind force (N)
gust_strength = 0.1      # gust amplitude (N)
gust_frequency = 0.5     # gust frequency (Hz)
direction = [1, 0, 0]    # wind direction (X axis)

# Main loop
while supervisor.step(timestep) != -1 and drone is not None:
    t = supervisor.getTime()

    # Oscillating wind + random noise
    gust = gust_strength * math.sin(2 * math.pi * gust_frequency * t)
    noise = [random.uniform(-0.02, 0.02) for _ in range(3)]

    wind_force = [
        direction[0] * (base_strength + gust) + noise[0],
        direction[1] * (base_strength + gust) + noise[1],
        direction[2] * (base_strength + gust) + noise[2],
    ]

    # Apply force to the drone in world coordinates
    drone.addForce(wind_force, False)

    # Print to console
    #if int(t) % 1 == 0:
    #    print(f"[t={t:.1f}s] wind_force = {wind_force}")

