import numpy as np
import matplotlib.pyplot as plt
from bracos import *




if __name__ == "__main__":

    alpha_max = np.radians(300)  # aceleração angular máxima (rad/s²)
    omega_max = np.radians(90)  # velocidade angular máxima (rad/s)

    thetas_inicial = np.radians([80, 20])  # posição inicial (rad)

    theta1_final = np.radians(60)  # deslocamento angular desejado (rad)
    theta2_final = np.radians(30)  # deslocamento angular desejado (rad)

    posicao_final = np.array([theta1_final, theta2_final])

    theta1, theta2, omega1, omega2, tempo_total = mover(
        'd', thetas_inicial, posicao_final, omega_max, alpha_max
        )

    tempo = np.linspace(0, tempo_total, 500)
    theta1_vals = [theta1(t) for t in tempo]
    theta2_vals = [theta2(t) for t in tempo]
    omega1_vals = [omega1(t) for t in tempo]
    omega2_vals = [omega2(t) for t in tempo]

    plt.figure(figsize=(10, 5))
    plt.subplot(2, 1, 1)
    plt.plot(tempo, np.degrees(theta1_vals), label='θ1 (°)')
    plt.plot(tempo, np.degrees(theta2_vals), label='θ2 (°)')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Ângulo (°)')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(tempo, np.degrees(omega1_vals), label='ω1 (°/s)')
    plt.plot(tempo, np.degrees(omega2_vals), label='ω2 (°/s)')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Velocidade Angular (°/s)')
    plt.legend()
    plt.grid(True)
    plt.show()