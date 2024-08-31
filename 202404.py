import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Define the func(x,y)
def func(x, y):
    return x * y + 3 * x * x


figure = plt.figure()
ax = figure.add_subplot(111, projection='3d')

x = np.linspace(-5, 5, 100)
y = np.linspace(-5, 5, 100)

x, y = np.meshgrid(x, y)
z = func(x, y)

ax.plot_surface(x, y, z, color='y')
plt.show()

#%%

x = 5
y = -6

x_plus_y = x + y
x_minus_y = x - y
x_times_y = x * y
x_divided_by_y = x / y

fifth_power_of_x = x ** 5
square_root_of_x = x ** 0.5

print(fifth_power_of_x)
print(square_root_of_x)
#%%
z = 1+3
print(z)

