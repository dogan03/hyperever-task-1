import matplotlib.pyplot as plt
import pandas as pd

data = pd.read_csv("mpc_data.csv")
fig, axes = plt.subplots(3, 1, figsize=(10, 8))

axes[0].plot(data["time"], data["theta"] * 180 / 3.14159)
axes[0].set_ylabel("Pole Angle (deg)")
axes[0].grid()

axes[1].plot(data["time"], data["x"])
axes[1].set_ylabel("Cart Position")
axes[1].grid()

axes[2].plot(data["time"], data["force"])
axes[2].set_ylabel("Control Force")
axes[2].set_xlabel("Time (s)")
axes[2].grid()

plt.tight_layout()
plt.savefig("mpc_results.png")
plt.show()
