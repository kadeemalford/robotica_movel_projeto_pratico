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

Abra `matlab/formation_controller.m` e execute diretamente para usar a configuracao padrao. Por padrao, o script usa:

- `config.mode = 'simulation'`
- `config.duration = 40.0`
- `config.rate_hz = 30.0`
- `config.ros.master_uri = 'http://192.168.0.100:11311'`
- `config.ros.pose_prefix = '/vrpn_client_node'`
- `config.ros.limo_ns = 'L1'`
- `config.ros.drone_ns = 'B1'`

Se quiser sobrescrever parte da configuracao, defina uma struct `config` no workspace antes de executar o script. O script faz merge dessa struct com os valores padrao.

Exemplo:

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

Se preferir o uso mais simples, basta abrir o arquivo e executar sem criar nada antes.

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

Para rodar em ROS no MATLAB:

1. Ajuste `config.mode = 'ros'`.
2. Ajuste `config.takeoff = true` se quiser decolar automaticamente.
3. Ajuste namespaces se o laboratorio nao usar `L1` e `B1`.
4. Execute o script.

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

### MATLAB nao conecta no ROS

- confirme a Robotics System Toolbox;
- confirme o master em `config.ros.master_uri`;
- se necessario, rode `rosshutdown` antes e coloque `config.ros.reinitialize = true`.

### Nao chegam poses no MATLAB

- confira o prefixo `config.ros.pose_prefix`;
- confira namespaces reais do laboratorio;
- valide os topicos no ROS antes do ensaio.

### O LIMO ou o drone nao respondem

- confira se os `cmd_vel` corretos existem;
- confirme os namespaces usados nos drivers;
- no caso do drone, confirme tambem o fluxo de decolagem do laboratorio.

### O drone oscila ou o LIMO fica agressivo

- reduza os ganhos cinemáticos e dinâmicos na struct `config`;
- valide primeiro em simulacao;
- verifique ruido e atraso de pose do OptiTrack.

## Observacoes

- Se a pose do LIMO no OptiTrack nao representar o centro de gravidade, ajuste a funcao local `limo_control_point_from_pose` nos dois pipelines.
- O yaw de referencia do drone foi mantido em `0 rad`.
- O pipeline Python foi mantido funcional e separado do pipeline MATLAB.
- O pipeline MATLAB foi escrito para priorizar clareza e execucao direta no laboratorio, sem `classdef`.
