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
        energy_noai = 0
        if (self.temperature_noai < self.optimal_temperature[0]):
            energy_noai = self.optimal_temperature[0] - self.temperature_noai
            self.temperature_noai = self.optimal_temperature[0]
        elif (self.temperature_noai > self.optimal_temperature[1]):
            energy_noai = self.temperature_noai - self.optimal_temperature[1]
            self.temperature_noai = self.optimal_temperature[1]

        # Computing the Reward.
        self.reward = energy_noai - energy_ai

        # Scaling the reward
        self.reward = 1e-3 * self.reward

        # GETTING THE NEXT STATE
        # Updating the atmospheric temperature
        self.atmospheric_temperature =self.monthly_atmospheric_temperature[month]

        # Updating the number of users
        self.current_number_users += np.random.randint(-self.max_update_users, self.max_update_users)
        if (self.current_number_users > self.max_number_users):
            self.current_number_users = self.max_number_users
        elif (self.current_number_users < self.min_number_users):
            self.current_number_users = self.min_number_users

        # Updating the rate of data
        self.current_rate_date += np.random.randint(-self.max_update_data, self.max_update_data)
        if (self.current_rate_data > self.max_rate_data):
            self.current_rate_users = self.max_rate_data
        elif (self.current_rate_data < self.min_rate_data):
            self.current_rate_data = self.min_rate_data

        # Computing the Delta of Intrinsic Temperature
        past_intrinsic_temperature = self.intrinsic_temperature #差分を取るために過去温度を定義
        self.intrinsic_temperature = (self.atmospheric_temperature + 1.25 * self.current_number_users
                                      + 1.25 * self.current_rate_data) #updateenvに合わせたIntrinsictemp
        delta_intrinsic_temperature = self.intrinsic_temperature - past_intrinsic_temperature
        # Computing the Delta of Temperature cause by the AI
        if (direction == -1):
            delta_temperature_ai = -energy_ai  #冷却された場合はマイナス
        elif (direction == 1):
            delta_temperature_ai = energy_ai   #加熱された場合はプラス

        # Updating the new Server's Temperature when there is the AI
        self.temperature_ai += delta_intrinsic_temperature + delta_temperature_ai
        # Updating the new Server's Temperature when there is the no AI
        self.total_energy_noai += delta_intrinsic_temperature

        # GETTING GAME OVER
        if (self.temperature_ai < self.min_temperature):
            if (self.train == 1):
                self.game_over = 1#温度下がりすぎたらしてたらゲームオーバー
            else:
                self.temperature_ai = self.optimal_temperature[0]
                self.total_energy_ai += self.optimal_temperature[0] - self.temperature_ai
                # 18度-Ai制御温度分を消費エネルギとして足す
        elif(self.tempereture_ai > self.max_temperature):
            if (self.train == 1):
                self.game_over = 1#温度超過してたらゲームオーバー
            else:
                self.tempereture_ai = self.optimal_temperature[1]
                self.total_energy_ai += self.tempereture_ai - self.optimal_temperature[1]
                #ai制御温度-24度ぶんを消費エネルギとしてたす。

        # UPDATE THE SCORES
        # Updating the Total Energy spent by the AI
        self.total_energy_ai += energy_ai
        # Updating the Total Energy spent by the server's cooling system when there is no AI
        self.total_energy_noai += energy_noai

        # SCALING THE NEXT STATE
        scaled_temperature_ai = (self.tempereture_ai - self.min_temperature) / (self.max_temperature - self.min_temperature)
        scaled_number_users = (self.current_number_users - self.min_number_users) / (self.max_number_users - self.min_number_users)
        scaled_rate_data = (self.current_rate_data - self.min_rate_data) / (self.max_rate_data - self.min_rate_data)
        next_state = np.matrix([scaled_temperature_ai, scaled_number_users, scaled_rate_data])

        # RETURNING THE NEXT STATE, THE REWARD AND GAME OVER
        return next_state, self.reward, self.game_over



    # MAKING A METHOD THAT RESETS THE ENVIRONMENT

    # MAKING A METHOD THAT GIVES US AT ANY TIME THE CURRENT STATE, THE LAST REWARD AND WHETHER THE GAME

#%%
