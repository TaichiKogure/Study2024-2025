import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Define the func(x,y)
def func(x, y):
    return x * y


figure = plt.figure()
ax = figure.add_subplot(111, projection='3d')

x = np.linspace(-5, 5, 100)
y = np.linspace(-5, 5, 100)

x, y = np.meshgrid(x, y)
z = func(x, y)

ax.plot_surface(x, y, z, color='b')
plt.show()
