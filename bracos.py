from numpy import sin, cos, arctan2, sqrt, clip, sign


def cinematica_direta(theta1: float, theta2: float) -> tuple:
    """
    Calcula a posição (x, y) do ponto final do braço robótico
    dado os ângulos das juntas theta1 e theta2.
    """
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
    return x, y


def alcancavel(x: float, y: float) -> bool:
    """
    Verifica se o ponto (x, y) está dentro do alcance do braço robótico.
    """
    distancia = sqrt(x**2 + y**2)

    return (raio_int - TOLERANCIA) <= distancia <= (raio_ext + TOLERANCIA)


def cinematica_inversa(x: float, y: float, cotovelo: str = 'baixo') -> tuple:
    """
    Calcula os ângulos das juntas theta1 e theta2 para alcançar o ponto (x, y).
    """

    if not alcancavel(x, y):
        raise ValueError("O ponto está fora do alcance do braço robótico.")

    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = clip(cos_theta2, -1, 1)
    sen_theta2 = sqrt(max(0, 1 - cos_theta2**2))

    if cotovelo == 'cima':
        sen_theta2 = -sen_theta2

    distancia = sqrt(x**2 + y**2)
    
    # Singularidade na borda externa (braço esticado)
    if abs(distancia - raio_ext) <= TOLERANCIA:
        cos_theta2 = 1.0
        sen_theta2 = 0.0
        
    # Singularidade na borda interna (braço dobrado)
    elif abs(distancia - raio_int) <= TOLERANCIA:
        cos_theta2 = -1.0
        sen_theta2 = 0.0

    theta2 = arctan2(sen_theta2, cos_theta2)

    theta1 = arctan2(y, x) - arctan2(L2 * sin(theta2), L1 + L2 * cos(theta2))

    return theta1, theta2



def perfil_velocidade(
        delta_theta: float, omega_max: float, alpha_max: float
        ) -> tuple:
    """
    Calcula o perfil de velocidade angular.
    """
    sinal = sign(delta_theta)
    delta_theta = abs(delta_theta)

    t_a = omega_max / alpha_max   # tempo de aceleração
    delta_theta_a = 0.5 * alpha_max * t_a**2

    if 2*delta_theta_a < delta_theta:   # perfil trapezoidal
        t_c = (delta_theta - 2*delta_theta_a) / omega_max   # duração da fase de cruzeiro
        tempo_total = 2*t_a + t_c
        omega_pico = omega_max

    else:  # perfil triangular
        t_a = sqrt(delta_theta / alpha_max)
        delta_theta_a = 0.5 * alpha_max * t_a**2  
        t_c = 0
        tempo_total = 2*t_a
        omega_pico = alpha_max * t_a   # pico real: nunca chega em omega_max
    
    def angulo(t: float) -> float:

        t = clip(t, 0, tempo_total)

        if t < t_a:    # fase de aceleração
            theta = 0.5 * alpha_max * t**2

        elif t < (t_a + t_c):    # fase de cruzeiro
            theta = delta_theta_a + omega_pico * (t - t_a)

        else:    # fase de desaceleração
            t_d = t - (t_a + t_c)
            theta = delta_theta_a + (omega_pico * (t_c + t_d)) - (0.5 * alpha_max * t_d**2)

        return sinal*theta

    def velocidade_angular(t: float) -> float:

        t = clip(t, 0, tempo_total)

        if t < t_a:    # fase de aceleração
            omega = alpha_max * t

        elif t < (t_a + t_c):    # fase de cruzeiro
            omega = omega_pico

        else:    # fase de desaceleração
            t_d = t - (t_a + t_c)
            omega = omega_pico - alpha_max * t_d

        return sinal*omega

    return angulo, velocidade_angular, tempo_total



def sincronizar_juntas(delta_theta1: float, delta_theta2: float, omega_max: float, alpha_max: float) -> tuple:
    """
    Sincroniza os perfis das juntas para que terminem ao mesmo tempo.
    """
    _, _, T1 = perfil_velocidade(delta_theta1, omega_max, alpha_max)
    _, _, T2 = perfil_velocidade(delta_theta2, omega_max, alpha_max)

    T = max(T1, T2)
    
    # Se não houver movimento (T = 0), evita erro de divisão
    if T == 0:
        return lambda t: 0.0, lambda t: 0.0, lambda t: 0.0, lambda t: 0.0, 0.0

    fator1 = T1 / T
    fator2 = T2 / T
    
    angulo1_sinc, velocidade1_sinc, _ = perfil_velocidade(delta_theta1, omega_max * fator1, alpha_max * (fator1**2))
    angulo2_sinc, velocidade2_sinc, _ = perfil_velocidade(delta_theta2, omega_max * fator2, alpha_max * (fator2**2))
    
    return angulo1_sinc, angulo2_sinc, velocidade1_sinc, velocidade2_sinc, T


def mover(modo: str, inicio: list, fim: list, omega_max: float, alpha_max: float
          ) -> tuple:
    """
    Move os braços do robô de acordo com o modo especificado.
    """
    modo = modo.lower()
    if modo == 'd' or modo == 'direta':
        delta_theta1 = fim[0] - inicio[0]
        delta_theta2 = fim[1] - inicio[1]

        angulo1, angulo2, velocidade1, velocidade2, tempo_total = sincronizar_juntas(
            delta_theta1, delta_theta2, omega_max, alpha_max
        )

    if modo == 'i' or modo == 'inversa':
        delta_x = fim[0] - inicio[0]
        delta_y = fim[1] - inicio[1]

        delta_theta1, delta_theta2 = cinematica_inversa(delta_x, delta_y)

        angulo1, angulo2, velocidade1, velocidade2, tempo_total = sincronizar_juntas(
            delta_theta1, delta_theta2, omega_max, alpha_max
        )

    return angulo1, angulo2, velocidade1, velocidade2, tempo_total


L1 = 0.2    # comprimento do primeiro braço (metros)
L2 = 0.15   # comprimento do segundo braço (metros)

raio_ext = L1 + L2  # raio externo (metros)
raio_int = abs(L1 - L2)  # raio interno (metros)

TOLERANCIA = 1e-9