# Projeto Pratico - Estrutura Virtual LIMO + Bebop 2

Este repositorio agora contem dois pipelines separados para o mesmo experimento:

- `python/`: implementacao principal em Python, com simulacao e ROS1 opcional;
- `matlab/`: implementacao equivalente em MATLAB, tambem com simulacao e ROS1 opcional.

O objetivo em ambos os casos e controlar uma formacao virtual entre um LIMO diferencial e um Bebop 2, incluindo:

- laço externo cinematico da formacao;
- laço interno dinamico do LIMO;
- laço interno dinamico do Bebop 2;
- desvio de obstaculo por projecao em espaco nulo;
- geracao automatica de logs, metricas e graficos.

## Estrutura do repositorio

```text
.
├── README.md
├── Especificacao do projeto.pdf
├── formacao.png
├── python/
│   ├── formation_controller.py
│   ├── plot_results.py
│   ├── requirements.txt
│   └── resultados/
└── matlab/
    ├── formation_controller.m
    ├── ROS_crazyflie_limo_e_joystick.m
    └── resultados/
```

## Modelo adotado

### Formacao virtual

Estado da formacao:

`q_f = [x_f, y_f, z_f, rho_f, alpha_f, beta_f]^T`

Referencia usada nos dois pipelines:

- `x_f = 0.75 sin(2 pi t / 40)`
- `y_f = 0.75 sin(4 pi t / 40)`
- `z_f = 0`
- `rho_f = 1.5`
- `alpha_f = 0`
- `beta_f = pi / 2`

Com `beta_f = pi / 2`, o drone deve ficar verticalmente acima do ponto de controle do LIMO.

### LIMO

O ponto de controle do LIMO foi considerado a `a = 0.10 m` a frente do centro de gravidade.

Parametros dinamicos usados:

`theta = [0.1521, 0.0953, 0.0031, 0.9840, -0.0451, 1.6422]^T`

### Bebop 2

Parametros dinamicos usados:

- `Ku = diag(0.8417, 0.8354, 3.9660, 9.8524)`
- `Kv = diag(0.18227, 0.17095, 4.0010, 4.7295)`

## Desvio de obstaculo

Foi implementada uma subtarefa prioritaria sobre o ponto de controle do LIMO:

- centro do obstaculo: `(-0.2, 0.425)` m;
- raio fisico: `0.15` m;
- zona de influencia: `0.50` m.

Quando o ponto de controle do LIMO entra na zona de influencia, o controlador injeta uma velocidade radial de repulsao. A tarefa principal da formacao e projetada no espaco nulo dessa subtarefa.

## Pipeline Python

Arquivos principais:

- `python/formation_controller.py`
- `python/plot_results.py`

### Dependencias

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r python/requirements.txt
```

### Simulacao

```bash
cd python
python3 formation_controller.py --duration 40
```

Com figuras interativas ao final:

```bash
cd python
python3 formation_controller.py --duration 40 --plot
```

### ROS1

```bash
source /opt/ros/<distro>/setup.bash
source ~/catkin_ws/devel/setup.bash
source .venv/bin/activate
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=<ip_do_seu_computador>
cd python
python3 formation_controller.py --ros --limo-ns L1 --drone-ns B1 --takeoff --duration 40
```

Topicos usados no Python:

- pose LIMO: `/vrpn_client_node/L1/pose`
- cmd LIMO: `/L1/cmd_vel`
- pose drone: `/vrpn_client_node/B1/pose`
- cmd drone: `/B1/cmd_vel`
- takeoff drone: `/B1/takeoff`
- land drone: `/B1/land`

### Analise offline no Python

```bash
cd python
python3 plot_results.py resultados/<pasta>/resultado_formacao.npz
```

## Pipeline MATLAB

Arquivos principais:

- `matlab/formation_controller.m`

Arquivo historico:

- `matlab/ROS_crazyflie_limo_e_joystick.m`

Esse ultimo arquivo nao implementa o controlador deste projeto. Ele foi mantido apenas como referencia antiga de uso de ROS no MATLAB.

### O que o `formation_controller.m` faz

- roda em `simulation` ou `ros`;
- usa o mesmo modelo e a mesma referencia do pipeline Python;
- salva log em `.mat` sem depender de nenhum outro script MATLAB;
- gera `metrics.csv` e os 7 graficos automaticamente ao final;
- possui parada de emergencia por teclado em uma janela de monitoramento;
- pode enviar `takeoff` e `land` no modo ROS.

O script e totalmente autosuficiente: controle, logging, metricas e plots estao todos implementados dentro de `matlab/formation_controller.m`.

### Configuracao rapida no MATLAB

Abra `matlab/formation_controller.m` e execute diretamente para usar a configuracao padrao. Se quiser sobrescrever parte da configuracao, defina uma struct `config` no workspace antes de executar o script. O script faz merge dessa struct com os valores padrao.

#### Variaveis de configuracao

##### Gerais

- `config.mode`: Modo de operacao (`'simulation'` ou `'ros'`). Padrao: `'simulation'`
- `config.duration`: Duracao do experimento em segundos. Padrao: `40.0`
- `config.rate_hz`: Taxa de controle em Hz. Padrao: `30.0`
- `config.real_time`: Executa em tempo real (true) ou o mais rapido possivel (false). Padrao: `false`
- `config.takeoff`: Envia comando de takeoff automaticamente no inicio (apenas modo ROS). Padrao: `false`
- `config.land_on_finish`: Envia comando de land ao final se houve takeoff. Padrao: `true`
- `config.show_monitor`: Exibe janela de monitoramento para parada de emergencia. Padrao: `true`
- `config.show_figures`: Exibe figuras ao final da execucao. Padrao: `false`
- `config.save_logs`: Salva arquivo .mat com dados do experimento. Padrao: `true`
- `config.analyze_logs`: Gera metricas e graficos automaticamente. Padrao: `true`
- `config.output_logfile`: Caminho customizado para o arquivo de log. Padrao: `''` (gera automaticamente)
- `config.results_dir`: Diretorio para salvar resultados. Padrao: `'matlab/resultados'`

##### ROS

- `config.ros.master_uri`: URI do master ROS. Padrao: `'http://192.168.0.100:11311'`
- `config.ros.pose_prefix`: Prefixo dos topicos de pose do OptiTrack. Padrao: `'/vrpn_client_node'`
- `config.ros.limo_ns`: Namespace do LIMO. Padrao: `'L1'`
- `config.ros.drone_ns`: Namespace do drone. Padrao: `'B1'`
- `config.ros.pose_timeout_s`: Timeout para poses do OptiTrack em segundos. Padrao: `0.50`
- `config.ros.takeoff_delay_s`: Tempo de espera apos takeoff em segundos. Padrao: `3.0`
- `config.ros.reinitialize`: Executa rosshutdown antes de conectar. Padrao: `false`
- `config.ros.only_limo`: Usa apenas LIMO real, drone simulado (modo ROS). Padrao: `false`
- `config.ros.only_bebop`: Usa apenas Bebop real, LIMO simulado (modo ROS). Padrao: `false`

##### Poses iniciais (simulacao)

- `config.initial.limo_pose`: Pose inicial do LIMO `[x, y, yaw]` em metros/rad. Padrao: `[0.4, -0.25, 0.0]`
- `config.initial.drone_pose`: Pose inicial do drone `[x, y, z, yaw]` em metros/rad. Padrao: `[0.4, 0.05, 1.5, 0.0]`

##### Parametros do LIMO

- `config.limo.a`: Distancia do ponto de controle ao centro de gravidade em metros. Padrao: `0.10`
- `config.limo.theta`: Parametros dinamicos `[theta1, theta2, theta3, theta4, theta5, theta6]`. Padrao: `[0.1521, 0.0953, 0.0031, 0.9840, -0.0451, 1.6422]`
- `config.limo.kinematic_gains`: Ganhos cinematicos `[kx, ky]`. Padrao: `[1.2, 1.2]`
- `config.limo.kinematic_limits`: Limites cinematicos `[lx, ly]`. Padrao: `[0.30, 0.30]`
- `config.limo.dynamic_gains`: Ganhos dinamicos (matriz diagonal 2x2, usado apenas com `command_type='dynamic'`). Padrao: `diag([2.2, 2.0])`
- `config.limo.command_limits`: Limites de comando `[u_max, omega_max]`. Padrao: `[1.0, 1.5]`
- `config.limo.command_type`: Tipo de comando enviado ao LIMO. `'velocity'` (padrao) envia a referencia cinematica direto no `cmd_vel` — correto para o LIMO real cujo `cmd_vel` espera velocidade. `'dynamic'` aplica o controlador dinamico (comportamento antigo) — use apenas se o driver aceitar torque/aceleracao. Padrao: `'velocity'`

##### Parametros do Bebop 2

- `config.drone.ku`: Matriz de ganhos Ku (diagonal 4x4). Padrao: `diag([0.8417, 0.8354, 3.9660, 9.8524])`
- `config.drone.kv`: Matriz de ganhos Kv (diagonal 4x4). Padrao: `diag([0.18227, 0.17095, 4.0010, 4.7295])`
- `config.drone.kinematic_gains`: Ganhos cinematicos `[kx, ky, kz, kyaw]`. Padrao: `[1.0, 1.0, 1.4, 1.0]`
- `config.drone.kinematic_limits`: Limites cinematicos `[lx, ly, lz, lyaw]`. Padrao: `[0.7, 0.7, 0.6, 0.7]`
- `config.drone.dynamic_gains`: Ganhos dinamicos (matriz diagonal 4x4). Padrao: `diag([1.0, 1.0, 1.0, 1.0])`
- `config.drone.command_limits`: Limites de comando `[vx, vy, vz, omega_yaw]`. Padrao: `[1.0, 1.0, 1.0, 1.0]`
- `config.drone.yaw_reference`: Yaw de referencia do drone em radianos. Padrao: `0.0`

##### Parametros da formacao

- `config.formation.control_gains`: Ganhos de controle da formacao `[kx, ky, kz, krho, kalpha, kbeta]`. Padrao: `[1.2, 1.2, 1.0, 0.0, 0.0, 0.0]`
- `config.formation.control_limits`: Limites de controle da formacao. Padrao: `[0.25, 0.25, 0.10, 0.00, 0.00, 0.00]`
- `config.formation.obstacle_center`: Centro do obstaculo `[x, y]` em metros. Padrao: `[-0.2, 0.425]`
- `config.formation.obstacle_radius`: Raio fisico do obstaculo em metros. Padrao: `0.15`
- `config.formation.obstacle_influence_radius`: Raio de influencia do obstaculo em metros. Padrao: `0.60`
- `config.formation.obstacle_gain`: Ganho de repulsao do obstaculo. Padrao: `3.0`
- `config.formation.obstacle_speed_limit`: Limite de velocidade radial de repulsao. Padrao: `0.7`

#### Exemplos de uso

##### Uso simples (valores padrao)

Basta abrir o arquivo e executar sem criar config antes:

```matlab
% No MATLAB, simplesmente rode:
run('formation_controller.m')
```

##### Simulacao customizada

```matlab
config = struct();
config.mode = 'simulation';
config.duration = 60;
config.show_figures = true;
config.limo.kinematic_gains = [2.0, 2.0];  % Ganhos mais agressivos
run('formation_controller.m')
```

##### ROS com ambos os robos

```matlab
config = struct();
config.mode = 'ros';
config.takeoff = true;
config.duration = 40;
config.show_figures = false;
config.ros = struct('master_uri', 'http://192.168.0.100:11311', ...
                    'limo_ns', 'L1', ...
                    'drone_ns', 'B1');
run('formation_controller.m')
```

##### ROS com apenas LIMO

```matlab
config = struct();
config.mode = 'ros';
config.duration = 40;
config.ros = struct('master_uri', 'http://192.168.0.100:11311', ...
                    'limo_ns', 'L1', ...
                    'only_limo', true);
run('formation_controller.m')
```

##### ROS com apenas Bebop

```matlab
config = struct();
config.mode = 'ros';
config.takeoff = true;
config.duration = 40;
config.ros = struct('master_uri', 'http://192.168.0.100:11311', ...
                    'drone_ns', 'B1', ...
                    'only_bebop', true);
run('formation_controller.m')
```

### Simulacao no MATLAB

No diretorio `matlab/`, abra `formation_controller.m` e execute.

O script vai:

- simular os dois robos;
- criar `matlab/resultados/` automaticamente se a pasta nao existir;
- salvar `resultado_formacao.mat` dentro de uma nova subpasta em `matlab/resultados/`;
- gerar `metrics.csv`;
- gerar os arquivos `01_trajetoria_xy.png` ate `07_distancia_obstaculo.png`.

### ROS1 no MATLAB

Requisitos:

- Robotics System Toolbox instalada;
- conectividade com o master ROS em `192.168.0.100`;
- topicos de pose do OptiTrack publicados;
- topicos `cmd_vel`, `takeoff` e `land` disponiveis.

Topicos esperados no MATLAB:

- pose LIMO: `/vrpn_client_node/L1/pose`
- pose drone: `/vrpn_client_node/B1/pose`
- cmd LIMO: `/L1/cmd_vel`
- cmd drone: `/B1/cmd_vel`
- takeoff drone: `/B1/takeoff`
- land drone: `/B1/land`

Para rodar em ROS no MATLAB, veja os exemplos na secao anterior. Os topicos esperados sao:

- pose LIMO: `/vrpn_client_node/L1/pose`
- pose drone: `/vrpn_client_node/B1/pose`
- cmd LIMO: `/L1/cmd_vel`
- cmd drone: `/B1/cmd_vel`
- takeoff drone: `/B1/takeoff`
- land drone: `/B1/land`

O script suporta rodar com apenas um dos robos conectados ao ROS (use `config.ros.only_limo` ou `config.ros.only_bebop`). Veja exemplos de uso completos na secao de configuracao acima.

Nota: Se ambos `only_limo` e `only_bebop` forem `true`, o comportamento e equivalente a ambos `false` (requer os dois robos).

### Parada de emergencia no MATLAB

Durante a execucao, uma janela de monitoramento e criada. Com essa janela selecionada, pressione:

- `Q`
- `Space`
- `Esc`

O controlador interrompe o loop, zera comandos e, se houve `takeoff`, envia `land` ao final.

## Passo a passo do experimento pratico em MATLAB

### 1. Preparacao fisica

- posicione o obstaculo em `(-0.2, 0.425, 0.0)` m;
- deixe o LIMO inicialmente em `(0.4, -0.25, 0.0)` m;
- deixe o drone aproximadamente `0.30 m` a esquerda do LIMO, alinhado com o enunciado;
- confirme cameras e marcadores do OptiTrack.

### 2. Conexao com a rede do laboratorio

- conecte o computador MATLAB a mesma rede do servidor ROS;
- confirme acesso ao master em `192.168.0.100:11311`.

### 3. Verificacao dos topicos

Antes de abrir o ensaio automatico, confirme que existem os topicos:

- `/vrpn_client_node/L1/pose`
- `/vrpn_client_node/B1/pose`
- `/L1/cmd_vel`
- `/B1/cmd_vel`
- `/B1/takeoff`
- `/B1/land`

### 4. Simulacao curta antes do ensaio real

Rode primeiro em simulacao no MATLAB para validar o script e a geracao de logs.

### 5. Ensaios reais em ROS

No MATLAB, ajuste por exemplo:

```matlab
config = struct();
config.mode = 'ros';
config.takeoff = true;
config.duration = 40;
config.ros = struct('master_uri', 'http://192.168.0.100:11311', ...
                    'limo_ns', 'L1', ...
                    'drone_ns', 'B1');
run('formation_controller.m')
```

### 6. Durante o ensaio

- mantenha a janela de emergencia visivel;
- acompanhe a qualidade das poses do OptiTrack;
- observe altura do drone, resposta do LIMO e desvio do obstaculo;
- se necessario, use `Q`, `Space` ou `Esc`.

### 7. Ao final

Os resultados ficam em `matlab/resultados/<pasta_do_experimento>/` com:

- `resultado_formacao.mat`
- `metrics.csv`
- `01_trajetoria_xy.png`
- `02_altitude_drone.png`
- `03_variaveis_formacao.png`
- `04_erros_formacao.png`
- `05_comandos_limo.png`
- `06_comandos_drone.png`
- `07_distancia_obstaculo.png`

## Troubleshooting

### Script roda como simulacao mesmo em modo ROS

Se o script gera resultados mas nao move os robos reais:

1. Execute o script de diagnostico:
   ```matlab
   cd matlab
   run('diagnostico_ros.m')
   ```

2. Execute o teste com debug:
   ```matlab
   cd matlab
   run('teste_ros_debug.m')
   ```

3. Consulte o guia completo: `TROUBLESHOOTING.md`

### MATLAB nao conecta no ROS

- confirme a Robotics System Toolbox;
- confirme o master em `config.ros.master_uri`;
- se necessario, rode `rosshutdown` antes e coloque `config.ros.reinitialize = true`.

### Nao chegam poses no MATLAB

- confira o prefixo `config.ros.pose_prefix`;
- confira namespaces reais do laboratorio;
- valide os topicos no ROS antes do ensaio;
- execute `diagnostico_ros.m` para verificacao completa.

### O LIMO ou o drone nao respondem

- confira se os `cmd_vel` corretos existem;
- confirme os namespaces usados nos drivers;
- no caso do drone, confirme tambem o fluxo de decolagem do laboratorio;
- use o teste manual no guia `TROUBLESHOOTING.md`.

### O drone oscila ou o LIMO fica agressivo

- reduza os ganhos cinemáticos e dinâmicos na struct `config`;
- valide primeiro em simulacao;
- verifique ruido e atraso de pose do OptiTrack.

## Observacoes

- Se a pose do LIMO no OptiTrack nao representar o centro de gravidade, ajuste a funcao local `limo_control_point_from_pose` nos dois pipelines.
- O yaw de referencia do drone foi mantido em `0 rad`.
- O pipeline Python foi mantido funcional e separado do pipeline MATLAB.
- O pipeline MATLAB foi escrito para priorizar clareza e execucao direta no laboratorio, sem `classdef`.
