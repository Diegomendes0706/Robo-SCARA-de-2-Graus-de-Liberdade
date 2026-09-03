import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from numpy import sin, cos, arctan2, sqrt, clip, sign, degrees, radians
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle




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
    if T == 0:
        return (lambda t: 0.0), (lambda t: 0.0), (lambda t: 0.0), (lambda t: 0.0), 0.0

    # junta parada: não faz sentido gerar perfil pra ela, ela fica em 0 o tempo todo
    if T1 == 0:
        angulo1_sinc, velocidade1_sinc = (lambda t: 0.0), (lambda t: 0.0)
    else:
        fator1 = T1 / T
        angulo1_sinc, velocidade1_sinc, _ = perfil_velocidade(
            delta_theta1, omega_max * fator1, alpha_max * (fator1**2)
        )

    if T2 == 0:
        angulo2_sinc, velocidade2_sinc = (lambda t: 0.0), (lambda t: 0.0)
    else:
        fator2 = T2 / T
        angulo2_sinc, velocidade2_sinc, _ = perfil_velocidade(
            delta_theta2, omega_max * fator2, alpha_max * (fator2**2)
        )

    return angulo1_sinc, angulo2_sinc, velocidade1_sinc, velocidade2_sinc, T


def mover(
        modo: str, inicio: list, fim: list, omega_max: float, 
        alpha_max: float, cotovelo: str = 'baixo'
        ) -> tuple:
    """
    Retorna as funções de ângulo deslocado e velocidade angular de cada junta e o
    tempo total de movimento.
    """
    modo = modo.lower()

    if modo in ['d', 'direta']:
        theta1_ini, theta2_ini = inicio
        delta_theta1 = fim[0] - inicio[0]
        delta_theta2 = fim[1] - inicio[1]

    elif modo in ['i', 'inversa']:
        theta1_ini, theta2_ini = cinematica_inversa(inicio[0], inicio[1], cotovelo)
        theta1_fim, theta2_fim = cinematica_inversa(fim[0], fim[1], cotovelo)
        
        delta_theta1 = theta1_fim - theta1_ini
        delta_theta2 = theta2_fim - theta2_ini  

    else:
        raise ValueError("Modo inválido. Use 'direta' ('d') ou 'inversa' ('i').")

    angulo_girado1, angulo_girado2, velocidade1, velocidade2, tempo_total = sincronizar_juntas(
    delta_theta1, delta_theta2, omega_max, alpha_max
    )

    angulo1 = lambda t: angulo_girado1(t) + theta1_ini
    angulo2 = lambda t: angulo_girado2(t) + theta2_ini

    return angulo1, angulo2, velocidade1, velocidade2, tempo_total


class RoboSCARA:

    def __init__(self, root):

        self.root = root

        self.root.protocol("WM_DELETE_WINDOW", self._ao_fechar)

        self.root.title("SCARA 2GDL - Controle e Cinemática")

        self.root.geometry("1200x750")

        self.theta1_atual = 0.0
        self.theta2_atual = 0.0

        self.movendo = False

        self.pontos = []

        self.rastro_x = []
        self.rastro_y = []

        self.frame_principal = ttk.Frame(
            root
        )

        self.frame_principal.pack(
            fill=tk.BOTH,
            expand=True
        )

        self.frame_grafico = ttk.Frame(
            self.frame_principal
        )

        self.frame_grafico.pack(
            side=tk.LEFT,
            fill=tk.BOTH,
            expand=True
        )

        self.figura, self.ax = plt.subplots(
            figsize=(7, 7)
        )

        self.canvas = FigureCanvasTkAgg(
            self.figura,
            master=self.frame_grafico
        )

        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH,
            expand=True
        )

        self.frame_controle = ttk.Frame(
            self.frame_principal,
            padding=10
        )

        self.frame_controle.pack(
            side=tk.RIGHT,
            fill=tk.Y
        )

        self.criar_controles()

        self.desenhar_robo()


    def _ao_fechar(self):
        self.movendo = False   # impede que animar()/animar_sequencia() se reagendem
        self.root.quit()       # sai do mainloop
        self.root.destroy()    # destrói a janela e libera os recursos


    def criar_controles(self):

        ttk.Label(
            self.frame_controle,
            text="Modo de Cinemática",
            font=("Arial", 12, "bold")
        ).pack(
            pady=(0, 5)
        )

        self.modo = tk.StringVar(
            value="Direta (CD)"
        )

        self.combo_modo = ttk.Combobox(
            self.frame_controle,
            textvariable=self.modo,
            values=[
                "Direta (CD)",
                "Inversa (CI)"
            ],
            state="readonly",
            width=20
        )

        self.combo_modo.pack(
            pady=5
        )

        self.combo_modo.bind(
            "<<ComboboxSelected>>",
            self.atualizar_modo
        )

        self.frame_parametros = ttk.LabelFrame(
            self.frame_controle,
            text="Comando",
            padding=10
        )

        self.frame_parametros.pack(
            fill=tk.X,
            pady=10
        )

        # theta1
        ttk.Label(
            self.frame_parametros,
            text="θ1 (graus):"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            pady=4
        )

        self.entry_theta1 = ttk.Entry(
            self.frame_parametros,
            width=12
        )

        self.entry_theta1.grid(
            row=0,
            column=1
        )

        self.entry_theta1.insert(
            0,
            "0"
        )

        # theta2
        ttk.Label(
            self.frame_parametros,
            text="θ2 (graus):"
        ).grid(
            row=1,
            column=0,
            sticky="w",
            pady=4
        )

        self.entry_theta2 = ttk.Entry(
            self.frame_parametros,
            width=12
        )

        self.entry_theta2.grid(
            row=1,
            column=1
        )

        self.entry_theta2.insert(
            0,
            "0"
        )

        # x
        ttk.Label(
            self.frame_parametros,
            text="x (m):"
        ).grid(
            row=2,
            column=0,
            sticky="w",
            pady=4
        )

        self.entry_x = ttk.Entry(
            self.frame_parametros,
            width=12
        )

        self.entry_x.grid(
            row=2,
            column=1
        )

        self.entry_x.insert(
            0,
            "0.25"
        )

        # y
        ttk.Label(
            self.frame_parametros,
            text="y (m):"
        ).grid(
            row=3,
            column=0,
            sticky="w",
            pady=4
        )

        self.entry_y = ttk.Entry(
            self.frame_parametros,
            width=12
        )

        self.entry_y.grid(
            row=3,
            column=1
        )

        self.entry_y.insert(
            0,
            "0.05"
        )

        # cotovelo
        ttk.Label(
            self.frame_parametros,
            text="Cotovelo:"
        ).grid(
            row=4,
            column=0,
            sticky="w",
            pady=4
        )

        self.cotovelo = tk.StringVar(
            value="baixo"
        )

        self.combo_cotovelo = ttk.Combobox(
            self.frame_parametros,
            textvariable=self.cotovelo,
            values=[
                "baixo",
                "cima"
            ],
            state="readonly",
            width=10
        )

        self.combo_cotovelo.grid(
            row=4,
            column=1
        )
        
        self.frame_velocidade = ttk.LabelFrame(
            self.frame_controle,
            text="Perfil de Movimento",
            padding=10
        )

        self.frame_velocidade.pack(
            fill=tk.X,
            pady=5
        )

        ttk.Label(
            self.frame_velocidade,
            text="ω_max (rad/s):"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            pady=4
        )

        self.entry_omega = ttk.Entry(
            self.frame_velocidade,
            width=12
        )

        self.entry_omega.grid(
            row=0,
            column=1
        )

        self.entry_omega.insert(
            0,
            "1.0"
        )

        ttk.Label(
            self.frame_velocidade,
            text="αmax (rad/s²):"
        ).grid(
            row=1,
            column=0,
            sticky="w",
            pady=4
        )

        self.entry_alpha = ttk.Entry(
            self.frame_velocidade,
            width=12
        )

        self.entry_alpha.grid(
            row=1,
            column=1
        )

        self.entry_alpha.insert(
            0,
            "2.0"
        )



        self.btn_mover = ttk.Button(
            self.frame_controle,
            text="▶ Mover",
            command=self.executar_movimento
        )

        self.btn_mover.pack(
            fill=tk.X,
            pady=5
        )

        self.btn_carregar = ttk.Button(
            self.frame_controle,
            text="📂 Carregar pontos",
            command=self.carregar_pontos
        )

        self.btn_carregar.pack(
            fill=tk.X,
            pady=5
        )

        self.btn_executar = ttk.Button(
            self.frame_controle,
            text="▶ Executar",
            command=self.executar_pontos
        )

        self.btn_executar.pack(
            fill=tk.X,
            pady=5
        )
        
        self.frame_status = ttk.LabelFrame(
            self.frame_controle,
            text="Status",
            padding=10
        )

        self.frame_status.pack(
            fill=tk.BOTH,
            expand=True,
            pady=10
        )

        self.status = tk.StringVar()

        self.label_status = ttk.Label(
            self.frame_status,
            textvariable=self.status,
            justify=tk.LEFT,
            wraplength=250
        )

        self.label_status.pack(
            anchor="w"
        )

        self.atualizar_status()



    def atualizar_modo(self, event=None):

        if self.modo.get() == "Direta (CD)":

            self.entry_theta1.config(
                state="normal"
            )

            self.entry_theta2.config(
                state="normal"
            )

            self.entry_x.config(
                state="disabled"
            )

            self.entry_y.config(
                state="disabled"
            )

            self.combo_cotovelo.config(
                state="disabled"
            )

        else:

            self.entry_theta1.config(
                state="disabled"
            )

            self.entry_theta2.config(
                state="disabled"
            )

            self.entry_x.config(
                state="normal"
            )

            self.entry_y.config(
                state="normal"
            )

            self.combo_cotovelo.config(
                state="readonly"
            )


    def desenhar_robo(self):

        self.ax.clear()

        theta1 = self.theta1_atual
        theta2 = self.theta2_atual

        # Juntas
        x0 = 0
        y0 = 0

        x1 = L1 * cos(theta1)
        y1 = L1 * sin(theta1)

        x2 = (x1 + L2 * cos(theta1 + theta2))

        y2 = (y1+ L2 * sin(theta1 + theta2))

        self.rastro_y.append(y2)
        self.rastro_x.append(x2)


        circulo_ext = Circle(
            (0, 0),
            raio_ext,
            fill=False,
            linestyle="--"
        )

        circulo_int = Circle(
            (0, 0),
            raio_int,
            fill=False,
            linestyle="--"
        )

        self.ax.add_patch(
            circulo_ext
        )

        self.ax.add_patch(
            circulo_int
        )


        self.ax.plot(
            [x0, x1],
            [y0, y1],
            linewidth=6,
            solid_capstyle="round"
        )

        self.ax.plot(
            [x1, x2],
            [y1, y2],
            linewidth=6,
            solid_capstyle="round"
        )
        

        self.ax.plot(
            x0,
            y0,
            "o",
            markersize=12
        )

        self.ax.plot(
            x1,
            y1,
            "o",
            markersize=10
        )

        self.ax.plot(
            x2,
            y2,
            "o",
            markersize=8
        )
        
        self.ax.plot(
            x2,
            y2,
            "x",
            markersize=12,
            markeredgewidth=3
        )

        self.ax.plot(
            self.rastro_x, self.rastro_y,
            "-", color="red", linewidth=1, alpha=0.4,
        )


        limite = raio_ext + 0.05

        self.ax.axhline(
            0,
            linewidth=1
        )

        self.ax.axvline(
            0,
            linewidth=1
        )

        self.ax.set_xlim(
            -limite,
            limite
        )

        self.ax.set_ylim(
            -limite,
            limite
        )

        self.ax.set_aspect(
            "equal"
        )

        self.ax.grid(
            True,
            alpha=0.3
        )

        self.ax.set_xlabel(
            "x (m)"
        )

        self.ax.set_ylabel(
            "y (m)"
        )

        self.ax.set_title(
            "Robô SCARA 2GDL"
        )

        self.canvas.draw_idle()



    def atualizar_status(self):

        x, y = cinematica_direta(
            self.theta1_atual,
            self.theta2_atual
        )

        alcance = alcancavel(x, y)

        if alcance:
            alcance_txt = "ALCANÇÁVEL"
        else:
            alcance_txt = "FORA DO ALCANCE"

        self.status.set(
            f"θ1 = {degrees(self.theta1_atual):.2f}°\n"
            f"θ2 = {degrees(self.theta2_atual):.2f}°\n\n"
            f"X = {x:.4f} m\n"
            f"Y = {y:.4f} m\n\n"
            f"Status: {alcance_txt}"
        )



    def executar_movimento(self):

        if self.movendo:
            return

        try:

            omega_max = float(
                self.entry_omega.get()
            )

            alpha_max = float(
                self.entry_alpha.get()
            )

            if omega_max <= 0 or alpha_max <= 0:
                raise ValueError("ω_max e α_max devem ser positivos.")


            if self.modo.get() == "Direta (CD)":

                theta1_fim = radians(
                    float(
                        self.entry_theta1.get()
                    )
                )

                theta2_fim = radians(
                    float(
                        self.entry_theta2.get()
                    )
                )

                inicio = [self.theta1_atual, self.theta2_atual]

                fim = [theta1_fim, theta2_fim]

                modo = "direta"

            else:

                x = float(
                    self.entry_x.get()
                )

                y = float(
                    self.entry_y.get()
                )

                if not alcancavel(x, y):

                    messagebox.showerror(
                        "Ponto inválido",
                        "O ponto informado está fora "
                        "do espaço de trabalho."
                    )

                    return

                modo = "inversa"

                inicio = cinematica_direta(
                    self.theta1_atual,
                    self.theta2_atual
                )

                fim = [x, y]

            (
                self.func_theta1,
                self.func_theta2,
                self.func_vel1,
                self.func_vel2,
                self.tempo_movimento
            ) = mover(
                modo,
                inicio,
                fim,
                omega_max,
                alpha_max,
                self.cotovelo.get()
            )

            self.tempo_atual = 0.0

            self.movendo = True

            self.animar()

        except Exception as erro:

            messagebox.showerror(
                "Erro",
                str(erro)
            )


    def animar(self):

        if not self.movendo:
            return

        t = self.tempo_atual

        if t >= self.tempo_movimento:

            self.theta1_atual = (
                self.func_theta1(
                    self.tempo_movimento
                )
            )

            self.theta2_atual = (
                self.func_theta2(
                    self.tempo_movimento
                )
            )

            self.movendo = False

            self.desenhar_robo()

            self.atualizar_status()

            return
        
        self.theta1_atual = (self.func_theta1(t))

        self.theta2_atual = (self.func_theta2(t))

        self.desenhar_robo()

        self.atualizar_status()

        self.tempo_atual += 1 / QPS

        self.root.after(
            int(1000 / QPS),
            self.animar
        )



    def carregar_pontos(self):
        arquivo = filedialog.askopenfilename(
            title="Selecionar arquivo de pontos",
            filetypes=[("Arquivo texto", "*.txt"), ("Arquivo CSV", "*.csv"), ("Todos", "*.*")],
        )
        if not arquivo:
            return

        self.pontos.clear()
        avisos = []

        with open(arquivo, "r", encoding="utf-8") as f:
            for n_linha, linha in enumerate(f, start=1):
                linha = linha.strip()

                if not linha or linha.startswith("#"):
                    continue  # pula linhas vazias e comentários

                valores = linha.replace(",", " ").split()
                if len(valores) < 2:
                    avisos.append(f"linha {n_linha}: formato inválido, ignorada")
                    continue

                try:
                    x = float(valores[0])
                    y = float(valores[1])
                except ValueError:
                    avisos.append(f"linha {n_linha}: x/y inválidos, ignorada")
                    continue

                if not alcancavel(x, y):
                    avisos.append(f"linha {n_linha}: ({x}, {y}) fora do alcance, ignorado")
                    continue

                # terceira coluna opcional: 0 = baixo (padrão), 1 = cima
                cotovelo = "cima" if len(valores) >= 3 and valores[2].strip() == "1" else "baixo"
                self.pontos.append((x, y, cotovelo))

        msg = f"{len(self.pontos)} ponto(s) carregado(s)."
        if avisos:
            msg += "\n\nAvisos:\n" + "\n".join(avisos)
        messagebox.showinfo("Pontos carregados", msg)


    def executar_pontos(self):

        if not self.pontos:

            messagebox.showwarning(
                "Aviso",
                "Nenhum ponto foi carregado."
            )

            return

        if self.movendo:
            return

        self.indice_ponto = 0

        self.executar_proximo_ponto()


    def executar_proximo_ponto(self):

        if self.indice_ponto >= len(self.pontos):

            self.status.set("Execução concluída!")

            return

        x, y, cotovelo_ponto = self.pontos[
            self.indice_ponto
        ]

        self.entry_x.config(
            state="normal"
        )

        self.entry_y.config(
            state="normal"
        )

        self.entry_x.delete(
            0,
            tk.END
        )

        self.entry_x.insert(
            0,
            str(x)
        )

        self.entry_y.delete(
            0,
            tk.END
        )

        self.entry_y.insert(
            0,
            str(y)
        )

        self.entry_x.config(
            state="disabled"
        )

        self.entry_y.config(
            state="disabled"
        )

        try:

            omega_max = float(
                self.entry_omega.get()
            )

            alpha_max = float(
                self.entry_alpha.get()
            )

            inicio = cinematica_direta(
                self.theta1_atual,
                self.theta2_atual
            )

            fim = [x, y]

            (
                self.func_theta1,
                self.func_theta2,
                _,
                _,
                self.tempo_movimento
            ) = mover(
                "inversa",
                inicio,
                fim,
                omega_max,
                alpha_max,
                cotovelo_ponto
            )

            self.tempo_atual = 0

            self.movendo = True

            self.animar_sequencia()

        except Exception as erro:

            messagebox.showerror(
                "Erro",
                str(erro)
            )

    def animar_sequencia(self):

        if not self.movendo:
            return

        t = self.tempo_atual

        if t >= self.tempo_movimento:

            self.theta1_atual = (
                self.func_theta1(
                    self.tempo_movimento
                )
            )

            self.theta2_atual = (
                self.func_theta2(
                    self.tempo_movimento
                )
            )

            self.desenhar_robo()

            self.atualizar_status()

            self.movendo = False

            self.indice_ponto += 1

            self.root.after(
                300,
                self.executar_proximo_ponto
            )

            return

        self.theta1_atual = (self.func_theta1(t))

        self.theta2_atual = (self.func_theta2(t))

        self.desenhar_robo()

        self.atualizar_status()

        self.tempo_atual += dt

        self.root.after(
            int(1000*dt),
            self.animar_sequencia
        )




L1 = 0.2    # comprimento do primeiro braço (metros)
L2 = 0.15   # comprimento do segundo braço (metros)

raio_ext = L1 + L2    # raio externo (metros)
raio_int = abs(L1 - L2)    # raio interno (metros)

TOLERANCIA = 1e-9

QPS = 50    # 50 quadros por segundo
dt = 1 / QPS    # intervalo de tempo entre quadros (segundos)

if __name__ == "__main__":

    root = tk.Tk()

    app = RoboSCARA(root)

    app.atualizar_modo()

    root.mainloop()