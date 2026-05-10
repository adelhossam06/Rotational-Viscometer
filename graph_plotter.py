import serial
import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial('COM4', 9600)

gamma_data = []
taw_data = []

plt.ion()
fig, ax = plt.subplots()

while True:
    try:
        data = ser.readline().decode().strip()
        gamma, taw = data.split(",")
        gamma = float(gamma)
        taw = float(taw)

        gamma_data.append(gamma)
        taw_data.append(taw)

        sorted_data = sorted(zip(gamma_data, taw_data))
        x = np.array([p[0] for p in sorted_data])
        y = np.array([p[1] for p in sorted_data])

        if len(x) > 3:
            coeffs = np.polyfit(x, y, 2)
            curve = np.poly1d(coeffs)
            x_smooth = np.linspace(min(x), max(x), 200)
            y_smooth = curve(x_smooth)

            ax.clear()
            ax.scatter(x, y)
            ax.plot(x_smooth, y_smooth)
            ax.set_xlabel("Gamma")
            ax.set_ylabel("Taw")
            plt.draw()
            plt.pause(0.01)
    except:
        pass
