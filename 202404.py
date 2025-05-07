import matplotlib

# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

x = np.arange(0, 1.5, 0.1)

y1 = np.sin(x)
plt.plot(x, y1, label='sin(x)')

y2 = np.cos(x)
plt.plot(x, y2, label='cos(x)')

y3 = np.tan(x)
plt.plot(x, y3, label='tan(x)')

plt.xlabel("x")
plt.ylabel("Wave Function")
plt.title("Sine, Cosine, and Tangent Waves")
plt.legend()
plt.show()
