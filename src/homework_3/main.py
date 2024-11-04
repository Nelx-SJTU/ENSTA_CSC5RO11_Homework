import numpy as np

gamma = 0.9
x = 0.25
y = 0.25

# Initial guesses for V* values for each state
V0, V1, V2, V3 = 0, 0, 0, 0

# Iterative approach
for i in range(1000):
    V0_new = max(gamma * V1, gamma * V2)
    V1_new = gamma * ((1 - x) * V1 + x * V3)
    V2_new = 1 + gamma * ((1 - y) * V0 + y * V3)
    V3_new = 10 + gamma * V0

    if abs(V0_new-V0)<0.0001 and abs(V1_new-V1)<0.0001 and abs(V2_new-V2)<0.0001 and abs(V3_new-V3)<0.0001:
        print("Finish in iteration =", i)
        break

    V0, V1, V2, V3 = V0_new, V1_new, V2_new, V3_new

print("V0 = ", V0)
print("V1 = ", V1)
print("V2 = ", V2)
print("V3 = ", V3)
