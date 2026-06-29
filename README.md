# Projeto Pratico - Estrutura Virtual LIMO + Bebop 2

Este diretorio contem uma implementacao em Python do controlador pedido no trabalho pratico.

Arquivo principal: `formation_controller.py`

## O que foi implementado

- controlador cinematico da formacao usando estrutura virtual;
- laço interno dinamico para o LIMO diferencial;
- laço interno dinamico para o quadrimotor Bebop 2;
- subtarefa de desvio de obstaculo por projecao em espaco nulo;
- modo simulacao padrao;
- interface ROS1 opcional para experimento real.

## Modelo adotado

### Formacao virtual

Estado da formacao:

`q_f = [x_f, y_f, z_f, rho_f, alpha_f, beta_f]^T`

Referencia do enunciado:

- `x_f = 0.75 sin(2 pi t / 40)`
- `y_f = 0.75 sin(4 pi t / 40)`
- `z_f = 0`
- `rho_f = 1.5`
- `alpha_f = 0`
- `beta_f = pi / 2`

Com `beta_f = pi / 2`, o drone deve ficar verticalmente acima do ponto de controle do LIMO.

### LIMO

O ponto de controle do LIMO foi considerado a `a = 0.10 m` a frente do centro de gravidade, como pedido no enunciado.

O modelo dinamico interno usado foi o mostrado nos slides da disciplina a partir dos parametros identificados:

`theta = [0.1521, 0.0953, 0.0031, 0.9840, -0.0451, 1.6422]^T`

### Bebop 2

Foram reaproveitados os mesmos parametros do projeto `robotica_movel`:

- `Ku = diag(0.8417, 0.8354, 3.9660, 9.8524)`
- `Kv = diag(0.18227, 0.17095, 4.0010, 4.7295)`

## Desvio de obstaculo

Foi implementada uma subtarefa prioritaria sobre o ponto de controle do LIMO:

- obstaculo: centro `(-0.2, 0.425)` m;
- raio fisico `0.15` m;
- zona de influencia `0.50` m.

Quando o ponto do LIMO entra na zona de influencia, o controlador injeta uma velocidade radial de repulsao. A tarefa da formacao e projetada no espaco nulo dessa subtarefa, preservando o movimento tangencial quando possivel.

## Dependencias e ambiente

### Dependencias Python do projeto

Os scripts deste diretorio usam as seguintes bibliotecas Python instaladas via `pip`:

- `numpy`
- `matplotlib`

Instalacao recomendada em ambiente virtual:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### Dependencias do ROS no sistema

Para usar o modo `--ros`, nao basta instalar apenas o `requirements.txt`. Tambem e necessario ter uma instalacao funcional do ROS1 no sistema operacional.

Em geral, isso inclui:

- `roscore`
- `rospy`
- `geometry_msgs`
- `std_msgs`
- ferramentas como `rostopic`

Em Ubuntu com ROS1, esses pacotes normalmente sao instalados globalmente no sistema, por exemplo via `apt`.

Exemplo tipico:

```bash
sudo apt update
sudo apt install ros-<distro>-desktop-full
```

Dependendo da instalacao do laboratorio, tambem podem ser necessarios pacotes especificos adicionais do ecossistema ROS usados pelos drivers do LIMO, do Bebop 2 e do OptiTrack.

Depois da instalacao do ROS, normalmente e preciso carregar o ambiente dele no shell:

```bash
source /opt/ros/<distro>/setup.bash
```

Se houver um workspace ROS do laboratorio, tambem pode ser necessario fazer:

```bash
source ~/catkin_ws/devel/setup.bash
```

### Venv e ROS ao mesmo tempo

O ROS costuma ser instalado globalmente na maquina, fora do `venv`.

Na pratica:

- `numpy` e `matplotlib` ficam no `venv`;
- `rospy`, `geometry_msgs` e `std_msgs` normalmente vem da instalacao global do ROS.

Esses pacotes do ROS podem ficar disponiveis dentro do `venv` se o shell onde o `venv` foi ativado tambem tiver carregado o ambiente do ROS com `source /opt/ros/<distro>/setup.bash`.

Ou seja, um fluxo comum e:

```bash
source /opt/ros/<distro>/setup.bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

ou, se o `venv` ja existir:

```bash
source /opt/ros/<distro>/setup.bash
source .venv/bin/activate
```

O ponto principal e que o `venv` nao instala o ROS por conta propria. Ele apenas convive com a instalacao global do ROS quando o ambiente do sistema foi corretamente carregado.

### Ordem correta para abrir o terminal

### Comandos minimos do dia do experimento

Se o ambiente ja estiver instalado e configurado, o roteiro rapido no laboratorio e:

```bash
source /opt/ros/<distro>/setup.bash
source ~/catkin_ws/devel/setup.bash
source .venv/bin/activate
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=<ip_do_seu_computador>
python3 "formation_controller.py" --ros --limo-ns L1 --drone-ns B1 --takeoff --duration 40
```

Depois, para reprocessar os resultados manualmente, se necessario:

```bash
python3 "plot_results.py" "resultados/<pasta_do_experimento>/resultado_formacao.npz"
```

#### Caso 1: simulacao

Para rodar apenas em simulacao, sem ROS:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
python3 "formation_controller.py" --duration 40
```

Se o `venv` ja existir:

```bash
source .venv/bin/activate
python3 "formation_controller.py" --duration 40
```

#### Caso 2: experimento real com ROS

Para rodar no laboratorio com os robos reais:

```bash
source /opt/ros/<distro>/setup.bash
source .venv/bin/activate
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=<ip_do_seu_computador>
python3 "formation_controller.py" --ros --limo-ns L1 --drone-ns B1 --takeoff --duration 40
```

Se houver um workspace ROS do laboratorio, carregue-o antes de executar o script:

```bash
source /opt/ros/<distro>/setup.bash
source ~/catkin_ws/devel/setup.bash
source .venv/bin/activate
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=<ip_do_seu_computador>
python3 "formation_controller.py" --ros --limo-ns L1 --drone-ns B1 --takeoff --duration 40
```

Resumo da ordem no modo ROS:

- carregar ambiente do ROS;
- carregar workspace ROS, se existir;
- ativar o `venv`;
- exportar variaveis `ROS_MASTER_URI` e `ROS_IP`;
- executar o controlador.

## Como executar

### Simulacao

```bash
python3 "formation_controller.py" --duration 40 --plot
```

Por padrao, cada execucao cria automaticamente uma nova pasta dentro de `resultados/`.

Nessa pasta o script salva:

- `resultado_formacao.npz`
- `metrics.csv`
- `01_trajetoria_xy.png`
- `02_altitude_drone.png`
- `03_variaveis_formacao.png`
- `04_erros_formacao.png`
- `05_comandos_limo.png`
- `06_comandos_drone.png`
- `07_distancia_obstaculo.png`

Sem graficos:

```bash
python3 "formation_controller.py" --duration 10 --no-save
```

### ROS1

```bash
python3 "formation_controller.py" --ros --limo-ns L1 --drone-ns B1 --takeoff
```

Se quiser forcar um nome ou caminho especifico para o log:

```bash
python3 "formation_controller.py" --duration 40 --save "meu_log.npz"
```

Se quiser salvar o `.npz`, mas pular a geracao automatica dos graficos e metricas:

```bash
python3 "formation_controller.py" --duration 40 --no-analyze
```

Topicos usados:

- LIMO pose: `/vrpn_client_node/L1/pose`
- LIMO cmd: `/L1/cmd_vel`
- drone pose: `/vrpn_client_node/B1/pose`
- drone cmd: `/B1/cmd_vel`
- drone takeoff: `/B1/takeoff`
- drone land: `/B1/land`

Os namespaces podem ser alterados por argumento.

## Passo a passo do experimento pratico

Esta secao descreve um procedimento completo para executar o experimento no laboratorio, desde a conexao com os robos ate a coleta dos resultados.

### 1. Preparacao do ambiente

Antes de ligar os robos, prepare o cenario fisico:

- posicione o obstaculo cilindrico com centro em `(-0.2, 0.425, 0.0)` m;
- garanta que a area de voo e a area de deslocamento do LIMO estejam livres;
- confira se as cameras do OptiTrack estao ligadas e calibradas;
- confirme que os marcadores do LIMO e do Bebop 2 estao firmes e visiveis;
- deixe o LIMO inicialmente em `(0.4, -0.25, 0.0)` m, orientado com yaw `0` rad;
- deixe o drone cerca de `0.30 m` a esquerda do LIMO, alinhado com o mesmo eixo, conforme o enunciado.

### 2. Ligar os equipamentos

Ligue os elementos do sistema nesta ordem:

- computador que executara o controlador;
- servidor ROS do laboratorio;
- sistema OptiTrack;
- LIMO;
- Bebop 2.

Se o laboratorio usar roteador ou switch dedicado, confirme tambem que todos os equipamentos estao na mesma rede.

### 3. Conectar o computador a rede do laboratorio

No computador de controle:

- conecte-se a rede usada pelo servidor ROS e pelos robos;
- confirme conectividade com o servidor ROS;
- se necessario, exporte as variaveis do ROS.

Exemplo:

```bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=<ip_do_seu_computador>
```

Se o laboratorio usar `ROS_HOSTNAME` em vez de `ROS_IP`, ajuste conforme a configuracao local.

### 4. Verificar se o ROS esta acessivel

Antes de iniciar o experimento, verifique se o master esta respondendo:

```bash
rostopic list
```

Voce deve conseguir enxergar, ou passar a enxergar apos os launches corretos, topicos como:

- `/vrpn_client_node/L1/pose`
- `/vrpn_client_node/B1/pose`
- `/L1/cmd_vel`
- `/B1/cmd_vel`
- `/B1/takeoff`
- `/B1/land`

Se os namespaces do seu laboratorio forem diferentes, use os nomes reais ao executar o script.

### 5. Subir os drivers e interfaces dos robos

Inicie os nos necessarios do LIMO e do Bebop 2 conforme o procedimento do laboratorio.

O objetivo desta etapa e garantir que:

- o LIMO receba comandos em `/<namespace>/cmd_vel`;
- o Bebop 2 receba comandos em `/<namespace>/cmd_vel`;
- o Bebop 2 aceite `takeoff` e `land`;
- o OptiTrack publique as poses dos dois robos.

Se o laboratorio tiver arquivos `launch` proprios, execute-os agora.

### 6. Confirmar leitura das poses no OptiTrack

Verifique se as poses dos robos estao chegando corretamente:

```bash
rostopic echo /vrpn_client_node/L1/pose
```

```bash
rostopic echo /vrpn_client_node/B1/pose
```

Confirme:

- os valores mudam quando o robo e movido manualmente;
- os eixos estao coerentes com o sistema global do laboratorio;
- o yaw estimado faz sentido;
- nao ha perda frequente de deteccao dos marcadores.

Se a pose do LIMO no OptiTrack nao representar o centro de gravidade do robo, ajuste o codigo antes do experimento, pois o controlador assume esse ponto como base e desloca `0.10 m` para frente para obter o ponto de controle.

### 7. Testar comandos manuais basicos

Antes de rodar o controlador automatico, faca um teste simples de comunicacao.

Para o LIMO, envie um comando pequeno e interrompa em seguida. Exemplo:

```bash
rostopic pub /L1/cmd_vel geometry_msgs/Twist '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
```

Para o drone, apenas confirme que os topicos existem e que o procedimento de decolagem do laboratorio esta funcionando. Se o time tiver um protocolo proprio de seguranca para o Bebop 2, siga esse protocolo antes do modo automatico.

### 8. Validar a simulacao antes do ensaio real

Antes do ensaio pratico, rode uma simulacao curta com o mesmo software:

```bash
python3 "formation_controller.py" --duration 10 --plot
```

Use esta etapa para confirmar que:

- o script executa sem erro;
- o arquivo de log e gerado corretamente;
- a trajetoria da formacao faz sentido;
- o desvio do obstaculo esta ativo.

### 9. Posicionar os robos na configuracao inicial

Logo antes do experimento automatico:

- reposicione o LIMO na pose inicial especificada;
- reposicione o Bebop 2 aproximadamente na posicao inicial esperada;
- confirme novamente que as poses lidas no OptiTrack batem com essas posicoes;
- mantenha uma pessoa pronta para interromper o experimento se necessario.

### 10. Executar o controlador no modo ROS

No diretorio do projeto, execute:

```bash
python3 "formation_controller.py" --ros --limo-ns L1 --drone-ns B1 --takeoff --duration 40
```

Se quiser salvar em um caminho especifico:

```bash
python3 "formation_controller.py" --ros --limo-ns L1 --drone-ns B1 --takeoff --duration 40 --save "resultado_formacao_pratico.npz"
```

Se os namespaces forem outros, troque `L1` e `B1`.

Durante a execucao:

- o script aguardara as poses dos dois robos;
- com `--takeoff`, enviara o comando de decolagem ao drone;
- passara entao a enviar comandos ao LIMO e ao Bebop 2;
- registrara os dados do experimento no arquivo `.npz` especificado.

### 11. Monitorar a execucao com seguranca

Durante o ensaio, acompanhe:

- se o drone mantem altura segura;
- se o LIMO nao sai da area de operacao;
- se o obstaculo esta sendo evitado;
- se ha perda de pose no OptiTrack;
- se os robos mostram comportamento oscilatorio excessivo.

Se algo sair do esperado:

- interrompa o envio de comandos;
- pouse o drone imediatamente, se necessario;
- envie velocidade zero para o LIMO;
- so reinicie o experimento depois de reposicionar os robos e entender a causa.

### 12. Encerrar o experimento

Ao fim da execucao:

- confirme que o arquivo `.npz` foi salvo;
- envie comando de pouso ao drone, caso ele nao tenha sido pousado pelo procedimento do laboratorio;
- envie comando nulo ao LIMO;
- encerre os nos ROS dos robos, se necessario.

Exemplo de pouso manual do drone:

```bash
rostopic pub /B1/land std_msgs/Empty '{}' -1
```

### 13. Conferir os resultados coletados

Ao final do experimento, a pasta gerada em `resultados/` contera o `.npz`, as metricas em CSV e os graficos da analise.

O arquivo `resultado_formacao.npz`, ou o nome definido por `--save`, contem:

- `time`
- `formation_state`
- `formation_reference`
- `limo_pose`
- `drone_pose`
- `limo_command`
- `drone_command`
- `obstacle_distance`

Uma maneira rapida de inspecionar o conteudo e:

```bash
python3 - <<'PY'
import numpy as np
z = np.load('resultado_formacao_pratico.npz')
for k in z.files:
    print(k, z[k].shape)
PY
```

### 14. Gerar graficos apos o experimento

Se quiser primeiro executar o experimento e depois visualizar os dados, voce pode usar o proprio script em simulacao apenas como referencia grafica, ou fazer um script separado de analise. Como alternativa simples, execute um ensaio com graficos habilitados em simulacao:

```bash
python3 "formation_controller.py" --duration 40 --plot
```

Para o experimento real, o mais indicado e manter o log salvo e construir a analise offline para nao sobrecarregar a execucao.

### 15. Checklist resumido

Antes de iniciar:

- rede ROS funcionando;
- OptiTrack publicando poses;
- namespaces corretos;
- obstaculo posicionado;
- robos nas poses iniciais;
- area livre;
- operador pronto para interrupcao.

Ao final:

- drone pousado;
- LIMO parado;
- log `.npz` salvo;
- dados conferidos.

## Troubleshooting

### O script nao recebe pose de um ou dos dois robos

Sintomas:

- o experimento nao sai do estado inicial;
- o script parece travado aguardando dados;
- `rostopic echo` nao mostra mensagens novas.

Verificacoes:

- confirme `ROS_MASTER_URI` e `ROS_IP`;
- execute `rostopic list` e confira se os topicos existem;
- execute `rostopic hz /vrpn_client_node/L1/pose` e `rostopic hz /vrpn_client_node/B1/pose`;
- confira se os nomes `L1` e `B1` sao realmente os namespaces usados no laboratorio;
- verifique se os marcadores estao visiveis no OptiTrack.

### O LIMO ou o drone nao responde aos comandos

Sintomas:

- a pose chega no ROS, mas o robo nao se move;
- o topico `cmd_vel` existe, mas nao ha efeito fisico.

Verificacoes:

- teste um comando manual pequeno com `rostopic pub`;
- confira se o driver do robo foi iniciado corretamente;
- confirme que voce esta publicando no namespace correto;
- no caso do drone, confirme tambem se ele foi decolado com sucesso.

### O drone nao decola com `--takeoff`

Verificacoes:

- confirme se o topico `/<namespace>/takeoff` existe;
- tente publicar manualmente `rostopic pub /B1/takeoff std_msgs/Empty '{}' -1`;
- confirme bateria suficiente e estado operacional do Bebop 2;
- siga o protocolo de seguranca do laboratorio antes de repetir o comando.

### O yaw ou os eixos parecem errados

Sintomas:

- o robo se move em direcao diferente da esperada;
- o drone corrige lateralmente quando deveria corrigir longitudinalmente.

Verificacoes:

- confirme a convencao de eixos do OptiTrack no laboratorio;
- confira se a orientacao estimada faz sentido ao girar o robo manualmente;
- valide se o frame do corpo do robo coincide com o usado no controlador.

Se necessario, ajuste a leitura de yaw e a interpretacao do referencial no codigo.

### O LIMO faz curvas muito bruscas ou oscila

Possiveis causas:

- ganhos muito agressivos;
- velocidade angular estimada com ruido;
- pose do LIMO com jitter no OptiTrack.

Acoes sugeridas:

- reduza `dynamic_gains` e `kinematic_gains` do LIMO;
- reduza `control_limits` da formacao;
- confira a qualidade da estimacao de pose;
- verifique se a posicao inicial esta proxima da configuracao esperada.

### O drone oscila em altitude ou no plano XY

Possiveis causas:

- ganhos altos demais no laço externo ou interno;
- ruido no OptiTrack;
- atraso grande entre medicao e comando.

Acoes sugeridas:

- reduza `kinematic_gains` e `dynamic_gains` do drone;
- confira se a taxa de pose esta adequada;
- tente um experimento curto sem obstaculo para isolar o problema.

### O obstaculo nao esta sendo evitado

Verificacoes:

- confirme se o obstaculo foi colocado no ponto `(-0.2, 0.425)` m;
- confirme se o ponto de controle do LIMO realmente entra na zona de influencia;
- verifique se `obstacle_influence_radius` e `obstacle_gain` estao adequados;
- abra o log e inspecione a variavel `obstacle_distance`.

### O arquivo `.npz` nao foi salvo

Verificacoes:

- confirme permissao de escrita no diretorio;
- use `--save nome_do_arquivo.npz` explicitamente;
- evite interromper o script de forma abrupta antes do fim da execucao.

### Como visualizar rapidamente os resultados do experimento

O `formation_controller.py` ja chama automaticamente o `plot_results.py` ao final da execucao, gerando todos os arquivos de analise na mesma pasta do experimento.

Se quiser rodar a analise novamente de forma manual em algum `.npz`, ainda e possivel usar:

Use o script auxiliar abaixo:

```bash
python3 "plot_results.py" "resultado_formacao_pratico.npz"
```

Sempre que esse script for executado manualmente, ele criara automaticamente uma nova subpasta dentro de `resultados/`, associada ao arquivo `.npz` informado.

Dentro dessa subpasta ele salvara:

- `metrics.csv`
- `01_trajetoria_xy.png`
- `02_altitude_drone.png`
- `03_variaveis_formacao.png`
- `04_erros_formacao.png`
- `05_comandos_limo.png`
- `06_comandos_drone.png`
- `07_distancia_obstaculo.png`

Para tambem abrir as figuras na tela durante a geracao:

```bash
python3 "plot_results.py" "resultado_formacao_pratico.npz" --show
```

O script tambem calcula e imprime metricas automaticamente, incluindo:

- erro medio e maximo da posicao XY da formacao;
- erro medio e maximo de `rho`;
- erro medio e maximo angular de `alpha` e `beta`;
- erro medio e maximo de altitude do drone;
- menor distancia observada ao obstaculo.

Se quiser mudar a pasta raiz dos resultados:

```bash
python3 "plot_results.py" "resultado_formacao_pratico.npz" --results-dir "meus_resultados"
```

## Saida

Por padrao o codigo salva um arquivo `resultado_formacao.npz` com:

- tempo;
- estado atual da formacao;
- referencia da formacao;
- pose do LIMO;
- pose do drone;
- comandos enviados;
- distancia ao obstaculo.

## Observacoes

- Se a pose medida do LIMO no OptiTrack nao for o centro de gravidade, ajuste a funcao `limo_control_point_from_pose`.
- A orientacao de yaw do drone foi mantida em referencia constante `0 rad` por simplicidade, pois o enunciado nao impoe outra orientacao.
- Os ganhos foram escolhidos para fornecer uma base funcional em simulacao; o ajuste fino experimental ainda deve ser feito no laboratorio.
