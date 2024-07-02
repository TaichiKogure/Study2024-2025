#%%

import numpy as np
import matplotlib.pyplot as plt

# シミュレーションパラメータ
num_simulations = 10000
project_duration_mean = 12  # 平均プロジェクト期間（ヶ月）
project_duration_stddev = 3  # プロジェクト期間の標準偏差
man_hours_mean = 1600  # 平均工数（マンアワー）
man_hours_stddev = 300  # 工数の標準偏差
hourly_rate = 50  # 人件費（1人当たりの時給）

# シミュレーションの実行
project_durations = np.random.normal(project_duration_mean, project_duration_stddev, num_simulations)
man_hours = np.random.normal(man_hours_mean, man_hours_stddev, num_simulations)

# プロジェクトのコスト計算
project_costs = man_hours * hourly_rate

# 結果の表示
plt.figure(figsize=(12, 6))

# プロジェクト期間の分布
plt.subplot(1, 2, 1)
plt.hist(project_durations, bins=50, color='blue', alpha=0.7)
plt.title('R&D Project Duration Simulation')
plt.xlabel('Project Duration (months)')
plt.ylabel('Frequency')

# プロジェクトコストの分布
plt.subplot(1, 2, 2)
plt.hist(project_costs, bins=50, color='green', alpha=0.7)
plt.title('R&D Project Cost Simulation')
plt.xlabel('Project Cost ($)')
plt.ylabel('Frequency')

plt.tight_layout()
plt.show()

# KPIの計算
mean_duration = np.mean(project_durations)
stddev_duration = np.std(project_durations)
mean_cost = np.mean(project_costs)
stddev_cost = np.std(project_costs)

print(f'平均プロジェクト期間: {mean_duration:.2f} ヶ月')
print(f'プロジェクト期間の標準偏差: {stddev_duration:.2f} ヶ月')
print(f'平均プロジェクトコスト: ${mean_cost:.2f}')
print(f'プロジェクトコストの標準偏差: ${stddev_cost:.2f}')