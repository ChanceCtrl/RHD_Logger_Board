import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("data0016.csv")

# Keep only rows where Command == 0 and Reg == 17
filtered = df[(df["Command"] == 0) & (df["Reg"] == 17)]

plt.figure()

plt.plot(filtered["Timestamp"], filtered["RHD_A"], label="RHD_A")

# plt.plot(filtered["Timestamp"], filtered["RHD_B"], label="RHD_B")

plt.xlabel("uS")
plt.ylabel("int-ish16")
plt.legend()
plt.show()
