# AIFB Lesson2 Minimizing Costs in Energy Consumption of a DataCenter with Deep Q-Learning

#%%
# BUILDING THE ENVIRONMENT IN A CLASS.
import numpy as np


class Environment(Object):
    #INTRODUCING AND INITIALIZING ALL PARAMETERS AND VARIABLES OF THE ENVIRONMENT.
    def __init__(self, optimal_temperature=(18.0, 24.0), initial_month=0, initial_number_users=10,
                 initial_rata_data=60):
        self.monthly_atmospheric_temperature = [1.0, 5.0, 7.0, 10.0, 11.0, 20.0, 23.0, 24.0, 22.0, 10.0, 5.0, 1.0]
        self.initial_month = initial_month
        self.atmospheric_temperature = self.monthly_atomospheric_temperature[initial_month]
        self.optimal_temperature = optimal_temperature
        self.min_temperature = -20
        self.max_temperature = 80
        self.min_number_users = 10
        self.max_number_users = 100
        self.max_update_users = 5
        self.min_rate_data = 20
        self.max_rate_data = 300
        self.max_update_data = 10
        self.initial_number_users = initial_number_users
        self.current_number_users = initial_number_users
        self.initial_rata_data = initial_rata_data
        self.current_rate_data = initial_rata_data
        self.intrinsic_temperature = (self.atmospheric_temperature + 1.25 * self.current_number_users
                                      + 1.25 * self.current_rate_data)
        self.temperature_noai = (self.optimal_temperature[0] + self.optimal_temperature[1] / 2.0)
        self.total_energy_ai = 0.0
        self.total_energy_noai = 0.0
        self.reward = 0.0
        self.game_over = 0
        self.train = 1

    # MAKING A METHOD THAT UPDATES THE ENVIRONMENT RIGHT AFTER THE AI PLAYS AN ACTION
    def update_env(self, direction, energy_ai, month):
        # GETTING THE REWARD
        # Computing the energy spent by the server's cooling system when there is no AI
    # MAKING A METHOD THAT RESETS THE ENVIRONMENT

    # MAKING A METHOD THAT GIVES US AT ANY TIME THE CURRENT STATE, THE LAST REWARD AND WHETHER THE GAME

#%%
