% diagnostico_ros.m
% Script para diagnosticar problemas de conexao ROS
%
% Este script verifica:
% 1. Conexao com o master ROS
% 2. Topicos disponiveis
% 3. Mensagens nos topicos de pose
% 4. Publishers de comando

clear;
clc;

fprintf('=== DIAGNOSTICO ROS ===\n\n');

%% 1. Configuracao
master_uri = 'http://192.168.0.100:11311';
pose_prefix = '/vrpn_client_node';
limo_ns = 'L1';
drone_ns = 'B1';

fprintf('1. Configuracao:\n');
fprintf('   Master URI: %s\n', master_uri);
fprintf('   LIMO namespace: %s\n', limo_ns);
fprintf('   Drone namespace: %s\n', drone_ns);
fprintf('\n');

%% 2. Conexao com ROS
fprintf('2. Tentando conectar ao master ROS...\n');
try
    rosshutdown;
    pause(0.5);
catch
end

try
    rosinit(master_uri);
    fprintf('   [OK] Conectado ao master ROS\n\n');
catch exc
    fprintf('   [ERRO] Falha ao conectar: %s\n\n', exc.message);
    return;
end

%% 3. Lista de topicos
fprintf('3. Listando topicos disponiveis:\n');
try
    topics = rostopic('list');
    fprintf('   Total de topicos: %d\n', length(topics));
    
    % Procura topicos relevantes
    limo_pose_topic = sprintf('%s/%s/pose', pose_prefix, limo_ns);
    drone_pose_topic = sprintf('%s/%s/pose', pose_prefix, drone_ns);
    limo_cmd_topic = sprintf('/%s/cmd_vel', limo_ns);
    drone_cmd_topic = sprintf('/%s/cmd_vel', drone_ns);
    
    fprintf('\n   Topicos procurados:\n');
    fprintf('   - %s: %s\n', limo_pose_topic, char(any(strcmp(topics, limo_pose_topic)) * "ENCONTRADO" + ~any(strcmp(topics, limo_pose_topic)) * "NAO ENCONTRADO"));
    fprintf('   - %s: %s\n', drone_pose_topic, char(any(strcmp(topics, drone_pose_topic)) * "ENCONTRADO" + ~any(strcmp(topics, drone_pose_topic)) * "NAO ENCONTRADO"));
    fprintf('   - %s: %s\n', limo_cmd_topic, char(any(strcmp(topics, limo_cmd_topic)) * "ENCONTRADO" + ~any(strcmp(topics, limo_cmd_topic)) * "NAO ENCONTRADO"));
    fprintf('   - %s: %s\n', drone_cmd_topic, char(any(strcmp(topics, drone_cmd_topic)) * "ENCONTRADO" + ~any(strcmp(topics, drone_cmd_topic)) * "NAO ENCONTRADO"));
    fprintf('\n');
catch exc
    fprintf('   [ERRO] Falha ao listar topicos: %s\n\n', exc.message);
end

%% 4. Criando subscribers
fprintf('4. Criando subscribers para poses:\n');
try
    limo_pose_sub = rossubscriber(sprintf('%s/%s/pose', pose_prefix, limo_ns), 'geometry_msgs/PoseStamped');
    fprintf('   [OK] Subscriber LIMO criado\n');
catch exc
    fprintf('   [ERRO] Falha ao criar subscriber LIMO: %s\n', exc.message);
    limo_pose_sub = [];
end

try
    drone_pose_sub = rossubscriber(sprintf('%s/%s/pose', pose_prefix, drone_ns), 'geometry_msgs/PoseStamped');
    fprintf('   [OK] Subscriber drone criado\n');
catch exc
    fprintf('   [ERRO] Falha ao criar subscriber drone: %s\n', exc.message);
    drone_pose_sub = [];
end
fprintf('\n');

%% 5. Aguardando mensagens
fprintf('5. Aguardando mensagens (timeout 5s)...\n');

if ~isempty(limo_pose_sub)
    fprintf('   Aguardando mensagem do LIMO...\n');
    pause(2);
    limo_msg = limo_pose_sub.LatestMessage;
    if isempty(limo_msg)
        fprintf('   [AVISO] Nenhuma mensagem recebida do LIMO\n');
    else
        fprintf('   [OK] Mensagem recebida do LIMO\n');
        fprintf('       Posicao: x=%.3f, y=%.3f, z=%.3f\n', ...
                limo_msg.Pose.Position.X, ...
                limo_msg.Pose.Position.Y, ...
                limo_msg.Pose.Position.Z);
        fprintf('       Timestamp: %d.%09d\n', limo_msg.Header.Stamp.Sec, limo_msg.Header.Stamp.Nsec);
    end
end

if ~isempty(drone_pose_sub)
    fprintf('   Aguardando mensagem do drone...\n');
    pause(2);
    drone_msg = drone_pose_sub.LatestMessage;
    if isempty(drone_msg)
        fprintf('   [AVISO] Nenhuma mensagem recebida do drone\n');
    else
        fprintf('   [OK] Mensagem recebida do drone\n');
        fprintf('       Posicao: x=%.3f, y=%.3f, z=%.3f\n', ...
                drone_msg.Pose.Position.X, ...
                drone_msg.Pose.Position.Y, ...
                drone_msg.Pose.Position.Z);
        fprintf('       Timestamp: %d.%09d\n', drone_msg.Header.Stamp.Sec, drone_msg.Header.Stamp.Nsec);
    end
end
fprintf('\n');

%% 6. Testando publishers
fprintf('6. Criando publishers para comandos:\n');
try
    limo_cmd_pub = rospublisher(sprintf('/%s/cmd_vel', limo_ns), 'geometry_msgs/Twist');
    fprintf('   [OK] Publisher LIMO criado\n');
catch exc
    fprintf('   [ERRO] Falha ao criar publisher LIMO: %s\n', exc.message);
end

try
    drone_cmd_pub = rospublisher(sprintf('/%s/cmd_vel', drone_ns), 'geometry_msgs/Twist');
    fprintf('   [OK] Publisher drone criado\n');
catch exc
    fprintf('   [ERRO] Falha ao criar publisher drone: %s\n', exc.message);
end
fprintf('\n');

%% Resumo
fprintf('=== RESUMO ===\n');
fprintf('Se voce viu [OK] em todas as etapas acima, o ROS esta funcionando.\n');
fprintf('Se viu [AVISO] nas mensagens, os topicos existem mas nao estao publicando dados.\n');
fprintf('Verifique:\n');
fprintf('  - OptiTrack esta rodando e publicando poses?\n');
fprintf('  - Os namespaces L1 e B1 estao corretos?\n');
fprintf('  - Os marcadores dos robos estao sendo detectados?\n');
fprintf('\n');
fprintf('Para mais informacoes, rode no terminal:\n');
fprintf('  rostopic list\n');
fprintf('  rostopic echo %s/%s/pose\n', pose_prefix, limo_ns);
fprintf('  rostopic echo %s/%s/pose\n', pose_prefix, drone_ns);
fprintf('\n');
