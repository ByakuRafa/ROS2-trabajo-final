import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from collections import deque

VIZINHOS = ((-1, 0), (1, 0), (0, -1), (0, 1))  # cima, baixo, esquerda, direita

class TurtlebotCtrl(Node):

    def __init__(self):
        super().__init__("TurtlebotCtrl")
        self.odometria, self.laser = Odometry(), LaserScan()
        self.tem_odometria = self.tem_laser = False

        
        mapa_txt = """
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 0 0 0 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
        0 0 0 0 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
        0 0 0 0 0 1 1 1 0 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 1 1 1 0
        0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0
        0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0
        0 1 1 0 1 1 1 0 0 0 0 1 1 1 1 1 1 1 1 0
        0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0
        0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        """
        self.mapa = np.array([[int(v) for v in linha.split()] for linha in mapa_txt.strip().splitlines()])
        self.resolucao = 4.0
        self.origem_x, self.origem_y = self.mapa.shape[0] / 2.0, self.mapa.shape[1] / 2.0

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.cb_odometria, 10)
        self.create_subscription(LaserScan, "/scan", self.cb_laser, 10)
        self.create_timer(0.1, self.laco_de_controle)

        # Define eixos de varrreguta
        self.eixo_varredura, self.eixo_faixa, self.sentido_varredura = (0, 1), (1, 0), 1
        self.alvo, self.lista_negra, self.historico_alvos = None, set(), deque(maxlen=6)
        self.pintado_no_reset, self.tentativas_travado = -1, 0

        # State macbinne com 5 estados
        # PLAN   -> precisa decidir o próximo alvo
        # TURN   -> girando no lugar até apontar para alvo_angular
        # DRIVE  -> andando reto em direção ao alvo_angular
        # BACKUP -> recuando porque ficou cercado
        # STUCK  -> nada mais alcançável, parado de vez
        self.modo, self.alvo_angular = "PLAN", 0.0
        self.recuo = self.ticks_perto = 0
        self.historico_posicoes = deque(maxlen=30)

        self.dist_desacelera, self.dist_parada = 0.35, 0.18
        self.tolerancia_giro, self.ganho_giro, self.vel_giro_max = math.radians(6), 2.2, 1.0

    # -Cpnversao de grade para mapa e viceversa, clamp para limites do mapa

    def mundo_para_grade(self, x, y):
        """(x, y) do mundo -> (linha, coluna) da grade, com clamp nos limites do mapa."""
        i = int(round(-x * self.resolucao + self.origem_x))
        j = int(round(-y * self.resolucao + self.origem_y))
        return max(0, min(i, self.mapa.shape[0] - 1)), max(0, min(j, self.mapa.shape[1] - 1))

    def guinada(self):
        """Ângulo de guinada (yaw) atual, a partir do quaternion da odometria."""
        q = self.odometria.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def livre(self, celula):
        i, j = celula
        return 0 <= i < self.mapa.shape[0] and 0 <= j < self.mapa.shape[1] and celula not in self.lista_negra


    # 0 = frente, +pi/2 = esquerda, -pi/2 = direita, pi = tras
    def folga_no_angulo(self, angulo_rel, largura=math.radians(50)):
        """Menor distância do laser num setor de 'largura' centrado em angulo_rel,
        relativo à frente real do robô (0=frente, +pi/2=esq., -pi/2=dir., pi=trás)."""
        leituras = np.array(self.laser.ranges)
        if leituras.size == 0:
            return 10.0
        validas = np.where((leituras > 0.01) & np.isfinite(leituras), leituras, 10.0)
        n, inc = len(validas), self.laser.angle_increment
        if not inc:
            return float(np.min(validas))
        i0 = int(round((angulo_rel - largura / 2 - self.laser.angle_min) / inc)) % n
        i1 = int(round((angulo_rel + largura / 2 - self.laser.angle_min) / inc)) % n
        setor = validas[i0:i1 + 1] if i0 <= i1 else np.concatenate((validas[i0:], validas[:i1 + 1]))
        return float(np.min(setor)) if setor.size else 10.0

    def folga_frontal(self):
        return self.folga_no_angulo(0.0, largura=math.radians(60))

    def imprime_mapa(self):
        print("\n".join(" ".join(str(v) for v in linha) for linha in self.mapa), "\n")

    # Busca em grade bsf e planejamento de varredura
    def _bfs(self, inicio, alvo=None, aceita=None):
        """Busca em largura a partir de 'inicio'. Com 'alvo': para ao achá-lo e
        devolve o caminho até lá. Com 'aceita': para na 1a célula que satisfizer
        a condição e devolve só a célula."""
        pai = {inicio: None}
        fila = deque([inicio])
        while fila:
            celula = fila.popleft()
            if alvo is not None and celula == alvo:
                caminho = []
                while celula is not None:
                    caminho.append(celula)
                    celula = pai[celula]
                return caminho[::-1]
            if aceita is not None and aceita(celula):
                return celula
            for di, dj in VIZINHOS:
                prox = (celula[0] + di, celula[1] + dj)
                if self.livre(prox) and prox not in pai:
                    pai[prox] = celula
                    fila.append(prox)
        return [inicio] if alvo is not None else None

    def mais_proxima_nao_pintada(self, inicio):
        return self._bfs(inicio, aceita=lambda c: self.mapa[c[0]][c[1]] == 1 and c not in self.lista_negra)

    def caminho_ate(self, inicio, destino):
        return self._bfs(inicio, alvo=destino)

    # sistema de planejamento de varredura

    def escaneia_faixa(self, celula):
        """Anda pros dois lados ao longo do eixo_faixa (raio r = 1, 2, 3...),
        checando primeiro o lado '+' e depois o '-' a cada raio, até achar uma
        célula não pintada ou os dois lados ficarem bloqueados."""
        ri, rj = self.eixo_faixa
        bloqueado = {1: False, -1: False}
        r = 1
        while r < max(self.mapa.shape) and not all(bloqueado.values()):
            for sinal in (1, -1):
                if bloqueado[sinal]:
                    continue
                passo = (celula[0] + sinal * ri * r, celula[1] + sinal * rj * r)
                if not self.livre(passo):
                    bloqueado[sinal] = True
                elif self.mapa[passo[0]][passo[1]] == 1:
                    return passo
            r += 1
        return None

    def proximo_alvo(self, celula):
        di, dj = self.eixo_varredura
        frente = (celula[0] + di * self.sentido_varredura, celula[1] + dj * self.sentido_varredura)
        if self.livre(frente):
            return frente, False

        faixa = self.escaneia_faixa(celula)
        if faixa:
            return faixa, True  # se aceito inverte sentido_varredura

        destino = self.mais_proxima_nao_pintada(celula)
        if destino is None:
            return None, False
        caminho = self.caminho_ate(celula, destino)
        return (caminho[1] if len(caminho) > 1 else caminho[0]), False

    # Calcula o angulo absoluto em relacao a grid do mapa
    def angulo_para_passo(self, celula, candidato):
        di, dj = candidato[0] - celula[0], candidato[1] - celula[1]
        if di:
            return math.pi if di > 0 else 0.0
        if dj:
            return -math.pi / 2 if dj > 0 else math.pi / 2
        return self.guinada()

    # funcao de recuo que abandona o alvo atual e entra no modo BACKUP por alguns ticks
    def inicia_recuo(self):
        if self.alvo:
            self.lista_negra.add(self.alvo)
        self.alvo = None
        self.historico_alvos.clear()
        self.historico_posicoes.clear()
        self.recuo, self.ticks_perto, self.modo = 10, 0, "BACKUP"

    # funcao de planejamento que decide o próximo passo a dar, com base no mapa e no sensor
    def planeja_proximo(self, celula):
        """Sempre tenta na prioridade da grade (em frente -> muda de faixa ->
        BFS), mas só aceita um passo que o LASER confirme como viável — o mapa
        é só um plano aproximado, quem manda é o sensor. Se mapa e sensor
        divergirem, aprende (lista_negra) e tenta de novo; nunca recua só
        porque "tava livre" no mapa — recuar é exclusivo de estar cercado."""
        for _ in range(8):
            candidato, inverte_sentido = self.proximo_alvo(celula)

            if candidato is None:
                restantes = int(np.count_nonzero(self.mapa == 1))
                if restantes == 0:
                    return False
                pintado = int(np.count_nonzero(self.mapa == 2))
                if pintado != self.pintado_no_reset:
                    self.pintado_no_reset, self.lista_negra, self.tentativas_travado = pintado, set(), 0
                    continue
                self.tentativas_travado += 1
                if self.tentativas_travado >= 3:
                    self.modo = "STUCK"
                else:
                    self.inicia_recuo()
                return False

            if candidato in self.historico_alvos:
                self.lista_negra.update(self.historico_alvos)
                self.historico_alvos.clear()
                continue

            alvo_ang = self.angulo_para_passo(celula, candidato)
            rel = math.atan2(math.sin(alvo_ang - self.guinada()), math.cos(alvo_ang - self.guinada()))
            if self.folga_no_angulo(rel, largura=math.radians(20)) < self.dist_parada + 0.1:
                self.lista_negra.add(candidato)
                continue

            if inverte_sentido:
                self.sentido_varredura *= -1
            self.alvo, self.alvo_angular, self.modo = candidato, alvo_ang, "TURN"
            self.historico_alvos.append(candidato)
            self.tentativas_travado = 0
            self.historico_posicoes.clear()
            return True

        self.inicia_recuo()
        return False

    # loop principal

    def _publica(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.pub_cmd_vel.publish(msg)

    def laco_de_controle(self):
        if not (self.tem_odometria and self.tem_laser):
            return

        x, y = self.odometria.pose.pose.position.x, self.odometria.pose.pose.position.y
        celula = self.mundo_para_grade(x, y)

        # Pintura: célula alcançada que ainda era "a pintar" (1) vira "pintada" (2)
        if self.mapa[celula[0], celula[1]] == 1:
            self.mapa[celula[0], celula[1]] = 2
            self.imprime_mapa()

        # --- faz o robo recuar e gira até ficar livre
        if self.modo == "BACKUP":
            self.recuo -= 1
            self._publica(-0.1, 1.0)
            if self.recuo <= 0:
                self.modo = "PLAN"
            return

        # --- completou todos os pontos alcancaveis
        if self.modo == "STUCK":
            restantes = int(np.count_nonzero(self.mapa == 1))
            self._publica()
            self.get_logger().info(f"Parado: {restantes} células ainda não pintadas e inalcançáveis")
            return

        # --- Sem alvo definido,planeja  enqautno nao esta trancado ou em backup
        if self.modo == "PLAN" or self.alvo is None:
            if not self.planeja_proximo(celula):
                if self.modo not in ("BACKUP", "STUCK"):
                    restantes = int(np.count_nonzero(self.mapa == 1))
                    self._publica()
                    if restantes == 0:
                        self.get_logger().info("Grid completo: todos os 1 viraram 2")
                return

        erro = math.atan2(math.sin(self.alvo_angular - self.guinada()), math.cos(self.alvo_angular - self.guinada()))

        # --- Gira até estar alinhado
        if self.modo == "TURN":
            if abs(erro) < self.tolerancia_giro:
                self.modo = "DRIVE"
            else:
                self._publica(0.0, max(-self.vel_giro_max, min(self.vel_giro_max, self.ganho_giro * erro)))
                return

        # --- QUando alinhado, avanca
        if celula == self.alvo:
            self.alvo, self.modo = None, "PLAN"
            self._publica()
            return

        folga = self.folga_frontal()
        if folga < self.dist_parada:
            self.ticks_perto += 1
            if self.ticks_perto > 5:
                self.alvo, self.modo, self.ticks_perto = None, "PLAN", 0
            self._publica()
            return
        self.ticks_perto = 0

        # railguard de progresso caso fique travado
        self.historico_posicoes.append((x, y))
        if len(self.historico_posicoes) == self.historico_posicoes.maxlen:
            sx, sy = self.historico_posicoes[0]
            if math.hypot(x - sx, y - sy) < 0.1:
                self.inicia_recuo()
                return

        velocidade = 0.15 if folga > self.dist_desacelera else 0.15 * (folga - self.dist_parada) / (self.dist_desacelera - self.dist_parada)
        self._publica(max(0.03, velocidade), max(-0.3, min(0.3, 0.8 * erro)))

    def cb_odometria(self, msg):
        self.odometria, self.tem_odometria = msg, True

    def cb_laser(self, msg):
        self.laser, self.tem_laser = msg, True


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotCtrl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()