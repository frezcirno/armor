# %%
import math

g = 9.8

# %%
# y是相对坐标！！！
def get_k1(bulletSpeed, pitch, x, y):
    vx0 = bulletSpeed * math.cos(pitch)
    vy0 = bulletSpeed * math.sin(pitch)
    t0 = (vy0 + math.sqrt(vy0 * vy0 - 2 * g * y)) / g
    k1 = 2 * (vx0 * t0 - x) / (x * x)
    alpha = 0.01  # 类似学习率
    t = 0
    while True:
        t_ = (math.exp(x * k1) - 1) / (k1 * vx0)
        y_ = vy0 * t_ - 0.5 * g * t_ * t_
        diff = y - y_
        k1 -= alpha * diff
        # cout << k1 << " " << diff << endl
        t += 1
        if not (abs(diff) >= 0.001 and t < 1000):
            break  # 误差小于1mm
    return k1

# %%
get_k1(12, 10, 5, 0.5)

# %%
