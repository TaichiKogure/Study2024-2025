import matplotlib

matplotlib.use('Qt5Agg')  # または 'Agg', 'WebAgg', 'TkAgg', など
import matplotlib.pyplot as plt

import numpy as np


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
#
# x = 5
# y = -6
#
# x_plus_y = x + y
# x_minus_y = x - y
# x_times_y = x * y
# x_divided_by_y = x / y
#
# fifth_power_of_x = x ** 5
# square_root_of_x = x ** 0.5
#
# print(fifth_power_of_x)
# print(square_root_of_x)
# #%%
# from flask import Flask
#
# app = Flask(__name__)
#
#
# @app.route('/')
# def hello_world():
#     return 'Hello, World!'
#
#
# if __name__ == "__main__":
#     app.run(debug=True)
#
