import yaml
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

with open("./assets/scan.yaml", "r") as f:
    docs = list(yaml.safe_load_all(f))
    data = docs[0]   # <-- IMPORTANT FIX

angle_min = data["angle_min"]
angle_increment = data["angle_increment"]
ranges_raw = data["ranges"]

ranges = []
for r in ranges_raw:
    if isinstance(r, str):   # skips '...'
        continue
    ranges.append(float(r))

ranges = np.array(ranges)
angles = angle_min + np.arange(len(ranges)) * angle_increment

mask = np.isfinite(ranges)
x = ranges[mask] * np.cos(angles[mask])
y = ranges[mask] * np.sin(angles[mask])

plt.figure(figsize=(6,6))
plt.scatter(x, y, s=2)
plt.axis("equal")
plt.grid(True)
plt.savefig("scan.png", dpi=200)
plt.close()

print("Saved scan.png")
