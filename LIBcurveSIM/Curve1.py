import numpy as np
import matplotlib.pyplot as plt

# パラメータ設定
capacity_max_initial = 1500  # 初期最大容量 [mAh]
n_points = 500  # サンプリング点数

# ヒステリシス量（V）
hysteresis_voltage = 0.02  # 充電時: +0.01V, 放電時: -0.01V みたいな

# 内部抵抗 [Ohm]
internal_resistance_initial = 0.05  # 50 mΩ

# サイクル劣化設定
n_cycles = 300  # シミュレーションするサイクル数
capacity_fade_rate = 0.0005  # 1サイクルごとに0.05%の容量減少
resistance_increase_rate = 0.0008  # 1サイクルごとに0.08%の抵抗増加
# 正極 (NMC) OCV
def ocv_positive(soc):
    return 3.7 + 0.5 * (1 - np.exp(-5 * soc))

# 負極 (Graphite) OCV
def ocv_negative(soc):
    return 0.1 + 0.7 * np.exp(-5 * soc)

# サイクル数に応じた容量・抵抗を計算
capacity_max = capacity_max_initial * (1 - capacity_fade_rate * n_cycles)
internal_resistance = internal_resistance_initial * (1 + resistance_increase_rate * n_cycles)

# SOCスイープ
soc_charge = np.linspace(0, 1, n_points)
soc_discharge = np.linspace(1, 0, n_points)

# 電流仮定（mA）
charge_current = 500  # 500 mAで充電
discharge_current = 500  # 500 mAで放電

# 充電カーブ
voltage_charge = (ocv_positive(soc_charge) - ocv_negative(soc_charge)
                  + hysteresis_voltage/2)  # ヒステリシスを追加
voltage_charge += internal_resistance * (charge_current / 1000)  # IRドロップ (正方向)

# 放電カーブ
voltage_discharge = (ocv_positive(soc_discharge) - ocv_negative(soc_discharge)
                     - hysteresis_voltage/2)  # ヒステリシスを追加
voltage_discharge -= internal_resistance * (discharge_current / 1000)  # IRドロップ (逆方向)

# 容量スケール
capacity_charge = soc_charge * capacity_max
capacity_discharge = soc_discharge * capacity_max

# プロット
plt.figure(figsize=(8, 6))
plt.plot(capacity_charge, voltage_charge, label='Charge', color='red')
plt.plot(capacity_discharge, voltage_discharge, label='Discharge', color='blue')
plt.xlabel('Capacity [mAh]')
plt.ylabel('Cell Voltage [V]')
plt.title(f'Simulated Charge/Discharge Curve\n(NMC/Graphite, {n_cycles} cycles)')
plt.grid(True)
plt.legend()
plt.ylim(2.5, 4.3)
plt.xlim(0, capacity_max_initial)
plt.show()
