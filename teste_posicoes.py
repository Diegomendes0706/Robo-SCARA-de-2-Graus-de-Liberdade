from bracos import cinematica_direta, cinematica_inversa
from numpy import radians, degrees, sqrt
import pandas as pd
import numpy as np



if __name__ == "__main__":

    teste_CD = np.array([
        [0, 0],
        [90, 0],
        [0, 90],
        [45, 45],
        [30, 60],
        [90, 90]
    ])

    x, y = cinematica_direta(radians(teste_CD[:, 0]), radians(teste_CD[:, 1]))
    r = sqrt(x**2 + y**2)

    dfCD = pd.DataFrame(
        np.column_stack([teste_CD, x, y, r]),   # junta teste (θ1,θ2) com x,y em um único array
        columns=['theta 1 (°)', 'theta 2 (°)', 'x (m)', 'y (m)', 'r (m)']
    )

    teste_CI = np.array([
        [0.2, 0.15],
        [0.1414, 0.2914],
        [0.1732, 0.25],
        [-0.15, 0.2]
    ])

    theta1, theta2 = zip(*[cinematica_inversa(x, y) for x, y in teste_CI])
    theta1_cima, theta2_cima = zip(*[cinematica_inversa(x, y, cotovelo='cima') for x, y in teste_CI])

    dfCI = pd.DataFrame(
        np.column_stack(
            [teste_CI, degrees(theta1), degrees(theta2), degrees(theta1_cima), degrees(theta2_cima)]
            ),
        columns=['x (m)', 'y (m)', 'θ1 baixo (°)', 'θ2 baixo (°)', 'θ1 cima (°)', 'θ2 cima (°)']
    )


print(dfCD,'\n\n', dfCI)