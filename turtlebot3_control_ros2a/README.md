# Desafio Gazebo ROS 2 - Projeto de Robôs II (PRIA)

Sistema robótico autônomo desenvolvido em ROS 2 para monitoramento e exploração completa de um cenário em grade (grid). A solução integra percepção reativa (LiDAR), localização (Odometria) e uma máquina de estados finita para controle do movimento de um Turtlebot simulado no Gazebo.

## Pré-requisitos e Dependências

O nó foi desenvolvido para o ecossistema ROS 2 (recomendado Humble) e depende dos seguintes pacotes e bibliotecas:

* **ROS 2 Packages:** `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`
* **Simulação:** Pacotes do Turtlebot3 (ex: `turtlebot3_gazebo`) e Gazebo.
* **Python 3:** Biblioteca `numpy`.
* **Middleware (Recomendado):** Cyclone DDS (para evitar erros de *sequence size exceeds remaining buffer* na troca de mensagens de sensores locais).

## Compilação

1. Clone ou mova o pacote `turtlebot3_control_ros2` para o diretório `src` do seu workspace ROS 2 (ex: `~/ros2_ws/src`).
2. Acesse a raiz do workspace e recompile o pacote:

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_control_ros2

```

3. Atualize as variáveis de ambiente do terminal:

```bash
source install/setup.bash

```

## Execução

A inicialização do sistema requer dois terminais distintos.

**Terminal 1: Inicialização da Simulação**
Carregue o ambiente do Gazebo e o modelo do robô (substitua o comando de launch pelo arquivo específico do cenário do desafio, se aplicável):

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

```

**Terminal 2: Inicialização do Nó Autônomo**
Execute o controlador com o parâmetro `use_sim_time` ativado para sincronizar o relógio interno com o Gazebo:

```bash
ros2 run turtlebot3_control_ros2 turtlebot_ctrl --ros-args -p use_sim_time:=true

```

## Arquitetura da Solução

O sistema foi arquitetado para operar sem teleoperação ou rotinas puramente temporizadas, dividindo a lógica em quatro pilares fundamentais:

* **Máquina de Estados Finita (FSM):** O comportamento de movimento do robô é governado por cinco estados estritos:
* `PLAN`: Avalia a matriz e os sensores para definir a próxima coordenada alvo.
* `TURN`: Aplica controle proporcional para alinhar o *yaw* do robô com o vetor do alvo.
* `DRIVE`: Acelera em linha reta até atingir as coordenadas da célula de destino.
* `BACKUP`: Estado emergencial de recuo reativo para desvencilhamento.
* `STUCK`: Condição de parada segura quando a missão é concluída ou os caminhos restantes são fisicamente inalcançáveis.


* **Navegação Híbrida (Varredura e BFS):** O robô prioriza a exploração utilizando um padrão de varredura em "cobra" (boustrophedon) ao longo da matriz. Quando as faixas adjacentes estão bloqueadas, o sistema utiliza o algoritmo de Busca em Largura (BFS) para rotear o caminho mais curto através do espaço livre até a próxima célula não visitada (marcada como `1`).
* **Percepção Reativa Dinâmica (LiDAR):** A Odometria posiciona o robô na matriz teórica, mas o sensor LiDAR dita a viabilidade física. O nó filtra os raios do `/scan` para criar um "cone de visão frontal". Se a matriz indica caminho livre, mas o LiDAR detecta colisão iminente, a célula é inserida em uma Lista Negra dinâmica, forçando o recálculo imediato da rota.
* **Watchdog de Estagnação:** Um histórico das posições recentes do robô é mantido em um `deque`. Se a FSM estiver no modo `DRIVE` mas o deslocamento físico for inferior a 10 cm após um tempo limite, o sistema detecta travamento físico (quinas, atrito) e força o estado `BACKUP` para recuperar a manobrabilidade.