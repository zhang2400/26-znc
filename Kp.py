# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np

e = 2.7182818284590452353602874713527

left_range = -20000
right_range = 20000

x1 = np.linspace(left_range, right_range, 1000)
x2 = np.linspace(left_range, right_range, 1000)

kp = 0.7 * (np.tanh(abs(x2) / 8000)) + 0.3
kd = 0.6 * (np.tanh(abs(x1) / 10000)) + 0.4

plt.figure(figsize=(8, 6))  # 设置图像大小
plt.xlabel('df')
plt.ylabel('ratio')
plt.plot(x1, kd, color="red", label="kd")
plt.plot(x2, kp, color="blue", label="kp")

plt.grid(True, linestyle='-', alpha=0.6)  # 添加网格线
plt.legend(loc='upper right', fontsize=16, bbox_to_anchor=(1, 1))

plt.savefig("output.png")  # 保存到文件
print("图像已保存为 output.png")

