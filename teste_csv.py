import pandas as pd
import numpy as np
import bracos

# Carrega os pontos do arquivo CSV (desconsidera comentários)
df = pd.read_csv('pontos.csv', comment='#', header=None, names=['x', 'y', 'cotovelo'])

x, y, cotovelo = df['x'].values, df['y'].values, df['cotovelo'].values

theta1, theta2 = zip(*[bracos.cinematica_inversa(xi, yi, cotovelo='cima' if ci == 1 else 'baixo') for xi, yi, ci in zip(x, y, cotovelo)])

df['θ1 (°)'] = np.degrees(theta1)
df['θ2 (°)'] = np.degrees(theta2)

print(round(df, 2))