from numpy import sin, cos, arctan2, sqrt, clip, sign, degrees, radians
import pandas as pd
import csv
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

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



def sincronizar_juntas(
        delta_theta1: float, delta_theta2: float, omega_max: float, alpha_max: float
        ) -> tuple:
    """
    Sincroniza os perfis das juntas para que terminem ao mesmo tempo.
    Retorna as funções de ângulo e velocidade angular de cada junta e o
    tempo total de movimento.
    """
    _, _, T1 = perfil_velocidade(delta_theta1, omega_max, alpha_max)
    _, _, T2 = perfil_velocidade(delta_theta2, omega_max, alpha_max)

    T = max(T1, T2)
    
    # Se não houver movimento (T = 0), evita erro de divisão
    if T == 0:
        return lambda t: 0.0, lambda t: 0.0, lambda t: 0.0, lambda t: 0.0, 0.0

    fator1 = T1 / T
    fator2 = T2 / T
    
    angulo1_sinc, velocidade1_sinc, _ = perfil_velocidade(
        delta_theta1, omega_max * fator1, alpha_max * (fator1**2)
        )
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


# ==============================================================================
# INTERFACE GRÁFICA (TKINTER)
# ==============================================================================
class Tela:

    def __init__(self, root):
        self.root = root
        self.root.title("Simulador de um Robô SCARA de 2 Graus de Liberdade")
        self.root.geometry("900x600")
        self.root.resizable(False, False)

        self.theta1_atual = 0.0
        self.theta2_atual = 0.0

        self.rastro_pontos = []

        self.pontos_carregados = []

        self.animating = False
        self.dt = 0.02    # 50 FPS (20ms)

        self._configurar_interface()
        self._desenhar_espaco_e_robo()
        self._atualizar_interface_valores()

    def _configurar_interface(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        lbl_canvas = ttk.Label(
            left_frame,
            text="AREA DE DESENHO",
            font=("Arial", 12),
        )

        lbl_canvas.pack(anchor=tk.NW, pady=(0, 5))

        self.canvas_size = 500
        self.canvas = tk.Canvas(
            left_frame,
            width=self.canvas_size,
            height=self.canvas_size,
            bg="white",
            relief="sunken",
            bd=2,
        )
        self.canvas.pack()

        # Fator de escala: mapeia metros para pixels (origem no centro do canvas)
        self.scale = (self.canvas_size / 2) / 0.42
        self.cx = self.canvas_size / 2
        self.cy = self.canvas_size / 2

        # DIREITA: Painel de Controle
        right_frame = ttk.Frame(main_frame, width=320)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y)

        title_panel = ttk.Label(
            right_frame,
            text="Painel de Controle",
            font=("Arial", 16, "bold"),
        )
        title_panel.pack(anchor=tk.NW, pady=(0, 10))

        # 1. Comando por Juntas (CD)
        cd_frame = ttk.LabelFrame(
            right_frame, text="Comando por juntas (CD)", padding=8
        )
        cd_frame.pack(fill=tk.X, pady=5)

        ttk.Label(cd_frame, text="θ1 (°):").grid(
            row=0, column=0, sticky=tk.W, padx=2
        )
        self.ent_theta1 = ttk.Entry(cd_frame, width=8)
        self.ent_theta1.grid(row=0, column=1, padx=5)

        ttk.Label(cd_frame, text="θ2 (°):").grid(
            row=0, column=2, sticky=tk.W, padx=2
        )
        self.ent_theta2 = ttk.Entry(cd_frame, width=8)
        self.ent_theta2.grid(row=0, column=3, padx=5)

        # 2. Comando por Posição (CI)
        ci_frame = ttk.LabelFrame(
            right_frame, text="Comando por posição (CI)", padding=8
        )
        ci_frame.pack(fill=tk.X, pady=5)

        ttk.Label(ci_frame, text="x (m):").grid(
            row=0, column=0, sticky=tk.W, padx=2
        )
        self.ent_x = ttk.Entry(ci_frame, width=8)
        self.ent_x.grid(row=0, column=1, padx=5)

        ttk.Label(ci_frame, text="y (m):").grid(
            row=0, column=2, sticky=tk.W, padx=2
        )
        self.ent_y = ttk.Entry(ci_frame, width=8)
        self.ent_y.grid(row=0, column=3, padx=5)

        ttk.Label(ci_frame, text="cotovelo:").grid(
            row=1, column=0, sticky=tk.W, pady=5
        )

        self.var_cotovelo = tk.StringVar(value="baixo")

        rb_baixo = ttk.Radiobutton(
            ci_frame,
            text="baixo",
            variable=self.var_cotovelo,
            value="baixo",
        )

        rb_cima = ttk.Radiobutton(
            ci_frame, text="cima", variable=self.var_cotovelo, value="cima"
        )

        rb_baixo.grid(row=1, column=1, columnspan=2, sticky=tk.W)
        rb_cima.grid(row=1, column=3, sticky=tk.W)

        # 3. Limites Cinematicos
        lim_frame = ttk.LabelFrame(right_frame, text="Limites", padding=8)
        lim_frame.pack(fill=tk.X, pady=5)

        ttk.Label(lim_frame, text="ω_max (°/s):").grid(
            row=0, column=0, sticky=tk.W
        )

        self.ent_vmax = ttk.Entry(lim_frame, width=8)
        self.ent_vmax.insert(0, "90.0")
        self.ent_vmax.grid(row=0, column=1, padx=5)

        ttk.Label(lim_frame, text="α_max (°/s²):").grid(
            row=0, column=2, sticky=tk.W
        )

        self.ent_amax = ttk.Entry(lim_frame, width=8)
        self.ent_amax.insert(0, "300.0")
        self.ent_amax.grid(row=0, column=3, padx=5)

        # 4. Botões de Ação
        btn_frame = ttk.Frame(right_frame)
        btn_frame.pack(fill=tk.X, pady=10)

        self.btn_mover = tk.Button(
            btn_frame,
            text="Mover",
            bg="#2e7d32",
            fg="white",
            font=("Arial", 10, "bold"),
            command=self._cmd_mover,
        )

        self.btn_mover.pack(fill=tk.X, pady=2)

        self.btn_carregar = tk.Button(
            btn_frame,
            text="Carregar pontos",
            bg="#0277bd",
            fg="white",
            font=("Arial", 10, "bold"),
            command=self._cmd_carregar_pontos,
        )

        self.btn_carregar.pack(fill=tk.X, pady=2)

        self.btn_executar = tk.Button(
            btn_frame,
            text="Executar trajetória",
            bg="#d84315",
            fg="white",
            font=("Arial", 10, "bold"),
            command=self._cmd_executar_trajetoria,
        )
        self.btn_executar.pack(fill=tk.X, pady=2)

        # BARRA DE STATUS (Rodapé)
        self.lbl_status = ttk.Label(
            main_frame,
            text="Status: Pronto",
            relief="sunken",
            anchor=tk.W,
            padding=5,
        )
        self.lbl_status.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))

    def _to_canvas(self, x, y):
        """Converte coordenadas cartesianas (m) para coordenadas do Canvas (px)."""
        px = self.cx + x * self.scale
        py = self.cy - y * self.scale
        return px, py

    def _desenhar_espaco_e_robo(self):
        self.canvas.delete("all")

        # 1. Desenhar Eixos Cartesiano (x, y)
        self.canvas.create_line(
            0, self.cy, self.canvas_size, self.cy, fill="#cccccc", dash=(2, 4)
        )
        self.canvas.create_line(
            self.cx, 0, self.cx, self.canvas_size, fill="#cccccc", dash=(2, 4)
        )

        # 2. Coroa Circular (Espaço de Trabalho)
        x_ext_min, y_ext_min = self._to_canvas(-raio_ext, raio_ext)
        x_ext_max, y_ext_max = self._to_canvas(raio_ext, -raio_ext)
        self.canvas.create_oval(
            x_ext_min,
            y_ext_min,
            x_ext_max,
            y_ext_max,
            outline="#2e7d32",
            width=2,
        )

        x_int_min, y_int_min = self._to_canvas(-raio_int, raio_int)
        x_int_max, y_int_max = self._to_canvas(raio_int, -raio_int)
        self.canvas.create_oval(
            x_int_min,
            y_int_min,
            x_int_max,
            y_int_max,
            outline="#2e7d32",
            width=1,
            dash=(4, 4),
        )

        # 3. Rastro da Ferramenta
        if len(self.rastro_pontos) > 1:
            for i in range(len(self.rastro_pontos) - 1):
                p1 = self._to_canvas(*self.rastro_pontos[i])
                p2 = self._to_canvas(*self.rastro_pontos[i + 1])
                self.canvas.create_line(p1, p2, fill="#e65100", width=2)

        # 4. Posição dos elos do Braço Robótico
        x1, y1 = L1 * cos(self.theta1_atual), L1 * sin(self.theta1_atual)
        xf, yf = cinematica_direta(self.theta1_atual, self.theta2_atual)

        p0 = self._to_canvas(0, 0)
        p1 = self._to_canvas(x1, y1)
        p2 = self._to_canvas(xf, yf)

        # Elos
        self.canvas.create_line(p0, p1, fill="#1b5e20", width=5)
        self.canvas.create_line(p1, p2, fill="#1b5e20", width=4)

        # Juntas (Origem, Cotovelo, Ferramenta)
        self.canvas.create_oval(
            p0[0] - 5,
            p0[1] - 5,
            p0[0] + 5,
            p0[1] + 5,
            fill="black",
            outline="black",
        )
        self.canvas.create_oval(
            p1[0] - 5,
            p1[1] - 5,
            p1[0] + 5,
            p1[1] + 5,
            fill="black",
            outline="black",
        )
        self.canvas.create_oval(
            p2[0] - 6, 
            p2[1] - 6, 
            p2[0] + 6, 
            p2[1] + 6, 
            fill="#d84315", 
            outline=""
        )

    def _atualizar_interface_valores(self):
        """Atualiza as caixas de texto com a pose atual em tempo real."""
        x, y = cinematica_direta(self.theta1_atual, self.theta2_atual)
        t1_deg = degrees(self.theta1_atual)
        t2_deg = degrees(self.theta2_atual)

        self.ent_theta1.delete(0, tk.END)
        self.ent_theta1.insert(0, f"{t1_deg:.2f}")

        self.ent_theta2.delete(0, tk.END)
        self.ent_theta2.insert(0, f"{t2_deg:.2f}")

        self.ent_x.delete(0, tk.END)
        self.ent_x.insert(0, f"{x:.4f}")

        self.ent_y.delete(0, tk.END)
        self.ent_y.insert(0, f"{y:.4f}")

        eh_alcancavel = alcancavel(x, y)

        self.lbl_status.config(
            text=f"Pose: (θ1 = {t1_deg:.1f} °, θ2 = {t2_deg:.1f} °) | (x = {x:.2f} m, y = {y:.2f} m) | "
            f"Alcançável: {'Sim' if eh_alcancavel else 'Não'}"
        )

    def _obter_limites(self):
        try:
            v_deg = float(self.ent_vmax.get())
            a_deg = float(self.ent_amax.get())
            return radians(v_deg), radians(a_deg)
        except ValueError:
            messagebox.showerror(
                "Erro de Entrada",
                "Insira valores numéricos válidos para os limites de velocidade e aceleração.",
            )
            return None, None

    def _animar_movimento(self, func_t1, func_t2, tempo_total, callback_fim=None):
        """Anima o movimento suavemente recalculando a pose frame a frame."""
        self.animating = True
        t_atual = 0.0

        def passo():
            nonlocal t_atual
            if t_atual <= tempo_total and self.animating:
                self.theta1_atual = func_t1(t_atual)
                self.theta2_atual = func_t2(t_atual)

                pos_ferramenta = cinematica_direta(
                    self.theta1_atual, self.theta2_atual
                )
                self.rastro_pontos.append(pos_ferramenta)

                self._desenhar_espaco_e_robo()
                self._atualizar_interface_valores()

                t_atual += self.dt
                self.root.after(int(self.dt * 1000), passo)
            else:
                self.animating = False
                if callback_fim:
                    callback_fim()

        passo()

    def _cmd_mover(self):
        if self.animating:
            return

        w_max, a_max = self._obter_limites()
        if w_max is None:
            return

        # Tenta ler CI primeiro, se falhar ou não bater, lê CD
        try:
            x_alvo = float(self.ent_x.get())
            y_alvo = float(self.ent_y.get())
            cotovelo = self.var_cotovelo.get()

            if not alcancavel(x_alvo, y_alvo):
                messagebox.showwarning(
                    "Fora de Alcance",
                    f"Ponto ({x_alvo}, {y_alvo}) está fora da coroa do espaço de trabalho!",
                )
                return

            t1_fim, t2_fim = cinematica_inversa(x_alvo, y_alvo, cotovelo)
        except ValueError:
            # Caso não seja número no X,Y lê o ângulo diretamente
            try:
                t1_fim = radians(float(self.ent_theta1.get()))
                t2_fim = radians(float(self.ent_theta2.get()))
            except ValueError:
                messagebox.showerror(
                    "Erro", "Verifique as coordenadas/ângulos digitados."
                )
                return

        # Mover via juntas sincronizadas
        f_t1, f_t2, _, _, T = mover(
            "direta",
            [self.theta1_atual, self.theta2_atual],
            [t1_fim, t2_fim],
            w_max,
            a_max,
        )

        self._animar_movimento(f_t1, f_t2, T)

    def _cmd_carregar_pontos(self):
        filepath = filedialog.askopenfilename(
            filetypes=[("Arquivos CSV", "*.csv"), ("Todos os arquivos", "*.*")]
        )
        if not filepath:
            return

        self.pontos_carregados.clear()
        try:
            with open(filepath, "r") as f:
                reader = csv.reader(f)
                for line in reader:
                    if not line or line[0].strip().startswith("#"):
                        continue
                    x = float(line[0].strip())
                    y = float(line[1].strip())
                    cotovelo_flag = int(line[2].strip())
                    cotovelo_str = "cima" if cotovelo_flag == 1 else "baixo"
                    self.pontos_carregados.append((x, y, cotovelo_str))

            messagebox.showinfo(
                "Sucesso",
                f"{len(self.pontos_carregados)} pontos carregados do arquivo!",
            )
            self.lbl_status.config(
                text=f"Status: {len(self.pontos_carregados)} pontos CSV prontos para execução."
            )
        except Exception as e:
            messagebox.showerror(
                "Erro de Leitura", f"Erro ao ler arquivo CSV:\n{e}"
            )

    def _cmd_executar_trajetoria(self):
        if self.animating:
            return

        if not self.pontos_carregados:
            messagebox.showwarning(
                "Nenhum Ponto",
                "Carregue um arquivo CSV de pontos primeiro!",
            )
            return

        w_max, a_max = self._obter_limites()
        if w_max is None:
            return

        self.rastro_pontos.clear()
        queue = list(self.pontos_carregados)

        def processar_proximo_ponto():
            if not queue:
                self.lbl_status.config(
                    text="Status: Trajetória do arquivo finalizada com sucesso!"
                )
                return

            x, y, cotovelo = queue.pop(0)

            # Validação de alcançabilidade antes da execução
            if not alcancavel(x, y):
                self.lbl_status.config(
                    text=f"Status: Ponto ({x:.2f}, {y:.2f}) ignorado (FORA DO ESPAÇO!)."
                )
                self.root.after(500, processar_proximo_ponto)
                return

            try:
                t1_fim, t2_fim = cinematica_inversa(x, y, cotovelo)
                f_t1, f_t2, _, _, T = mover(
                    "direta",
                    [self.theta1_atual, self.theta2_atual],
                    [t1_fim, t2_fim],
                    w_max,
                    a_max,
                )
                self._animar_movimento(
                    f_t1, f_t2, T, callback_fim=processar_proximo_ponto
                )
            except Exception as ex:
                print(f"Erro ao calcular ponto ({x}, {y}): {ex}")
                processar_proximo_ponto()

        processar_proximo_ponto()


L1 = 0.2    # comprimento do primeiro braço (metros)
L2 = 0.15   # comprimento do segundo braço (metros)

raio_ext = L1 + L2  # raio externo (metros)
raio_int = abs(L1 - L2)  # raio interno (metros)

TOLERANCIA = 1e-9

# ==============================================================================
# INICIALIZAÇÃO DA APLICAÇÃO
# ==============================================================================
if __name__ == "__main__":
    root = tk.Tk()
    tela = Tela(root)
    root.mainloop()


