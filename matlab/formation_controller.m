% formation_controller.m
% Controlador de estrutura virtual LIMO + Bebop 2 em MATLAB.
%
% Este arquivo foi escrito no estilo "script + funcoes locais" para manter
% a execucao simples e proxima do fluxo normalmente usado em laboratorio.
%
% Uso rapido:
%   1) Abra este arquivo no MATLAB.
%   2) Ajuste os campos de "config" logo abaixo, se necessario.
%   3) Rode o script.
%
% Atalhos de emergencia durante a execucao:
%   - pressione Q, Space ou Esc na janela de monitoramento.
%
% Este script e autosuficiente: executa o controlador, salva o log, calcula
% metricas e gera todos os graficos sem depender de outro arquivo MATLAB.
%
% O log salvo em .mat usa os mesmos nomes principais do pipeline Python:
%   time, formation_state, formation_reference, limo_pose, drone_pose,
%   limo_command, drone_command, obstacle_distance.
%
% Teste com apenas um robo (modo ROS):
%   - config.ros.only_limo = true: usa apenas o LIMO real, drone simulado
%   - config.ros.only_bebop = true: usa apenas o Bebop real, LIMO simulado
%   - Se ambos forem false (padrao): ambos os robos devem estar conectados
%   - Se ambos forem true: equivalente a ambos false (requer os dois)

if ~exist('config', 'var') || ~isstruct(config)
    config = default_config();
else
    config = merge_structs(default_config(), config);
end

last_run = run_formation_controller(config); %#ok<NASGU>

function cfg = default_config()
    base_dir = fileparts(mfilename('fullpath'));

    cfg = struct();
    cfg.mode = 'simulation';
    cfg.duration = 40.0;
    cfg.rate_hz = 30.0;
    cfg.real_time = false;
    cfg.takeoff = false;
    cfg.land_on_finish = true;
    cfg.show_monitor = true;
    cfg.show_figures = false;
    cfg.save_logs = true;
    cfg.analyze_logs = true;
    cfg.output_logfile = '';
    cfg.results_dir = fullfile(base_dir, 'resultados');

    cfg.ros = struct();
    cfg.ros.master_uri = 'http://192.168.0.100:11311';
    cfg.ros.pose_prefix = '/natnet_ros';
    cfg.ros.limo_ns = 'L1';
    cfg.ros.drone_ns = 'B1';
    cfg.ros.pose_timeout_s = 0.50;
    cfg.ros.takeoff_delay_s = 3.0;
    cfg.ros.reinitialize = false;
    cfg.ros.only_limo = true;
    cfg.ros.only_bebop = false;

    cfg.initial = struct();
    cfg.initial.limo_pose = [0.4, -0.25, 0.0];
    cfg.initial.drone_pose = [0.4, 0.05, 1.5, 0.0];

    cfg.limo = struct();
    cfg.limo.a = 0.10;
    cfg.limo.theta = [0.1521, 0.0953, 0.0031, 0.9840, -0.0451, 1.6422];
    cfg.limo.kinematic_gains = [1.2, 1.2];
    cfg.limo.kinematic_limits = [0.30, 0.30];
    cfg.limo.dynamic_gains = diag([2.2, 2.0]);
    cfg.limo.command_limits = [1.0, 1.5];
    % Tipo de comando enviado ao LIMO:
    %   'velocity': envia a referencia cinematica (u, omega) direto no cmd_vel.
    %               Correto para o LIMO real, cujo cmd_vel espera VELOCIDADE.
    %   'dynamic' : aplica o controlador dinamico (comportamento antigo).
    %               Use apenas se o driver do robo aceitar torque/aceleracao.
    cfg.limo.command_type = 'velocity';

    cfg.drone = struct();
    cfg.drone.ku = diag([0.8417, 0.8354, 3.9660, 9.8524]);
    cfg.drone.kv = diag([0.18227, 0.17095, 4.0010, 4.7295]);
    cfg.drone.kinematic_gains = [1.0, 1.0, 1.4, 1.0];
    cfg.drone.kinematic_limits = [0.7, 0.7, 0.6, 0.7];
    cfg.drone.dynamic_gains = diag([1.0, 1.0, 1.0, 1.0]);
    cfg.drone.command_limits = [1.0, 1.0, 1.0, 1.0];
    cfg.drone.yaw_reference = 0.0;

    cfg.formation = struct();
    cfg.formation.control_gains = [1.2, 1.2, 1.0, 0.0, 0.0, 0.0];
    cfg.formation.control_limits = [0.25, 0.25, 0.10, 0.00, 0.00, 0.00];
    cfg.formation.obstacle_center = [-0.2, 0.425];
    cfg.formation.obstacle_radius = 0.15;
    cfg.formation.obstacle_influence_radius = 0.60;
    cfg.formation.obstacle_gain = 3.0;
    cfg.formation.obstacle_speed_limit = 0.7;
end

function result = run_formation_controller(cfg)
    cfg = normalize_config(cfg);
    dt = 1.0 / cfg.rate_hz;
    target_samples = max(1, round(cfg.duration * cfg.rate_hz));
    loop_clock = tic;
    logs = initialize_logs(target_samples);
    prev_refs = struct('limo', [], 'drone', []);
    state = initialize_state(cfg);
    ros_io = [];
    stop_monitor = [];
    save_info = struct('output_dir', '', 'log_path', '');
    did_takeoff = false;
    interruption_reason = '';
    caught_exception = [];

    if cfg.show_monitor
        stop_monitor = create_emergency_stop_monitor();
    end

    try
        if strcmp(cfg.mode, 'ros')
            ros_io = setup_ros_io(cfg);
            if cfg.takeoff && ros_io.use_drone
                publish_empty_message(ros_io.takeoff_pub);
                did_takeoff = true;
                pause(cfg.ros.takeoff_delay_s);
            end
        end

        sample_idx = 1;
        while sample_idx <= target_samples
            if emergency_stop_requested(stop_monitor)
                interruption_reason = 'interrupcao manual via teclado';
                break;
            end

            if strcmp(cfg.mode, 'ros')
                [state, ros_io, ready] = update_state_from_ros(state, ros_io, cfg);
                if ~ready
                    pause(min(dt, 0.05));
                    continue;
                end
            end

            t = (sample_idx - 1) * dt;
            [commands, prev_refs] = compute_commands(state, cfg, prev_refs, dt, t);

            if strcmp(cfg.mode, 'simulation')
                logs = append_log_sample(logs, sample_idx, t, state, commands);
                state = step_simulation(state, cfg, commands, dt);
            else
                publish_ros_commands(ros_io, commands);
                logs = append_log_sample(logs, sample_idx, t, state, commands);
            end
            sample_idx = sample_idx + 1;
            wait_for_next_sample(cfg, dt, sample_idx - 1, loop_clock);
        end

        logs = trim_logs(logs, sample_idx - 1);
    catch caught
        caught_exception = caught;
        logs = trim_logs(logs, nnz(~isnan(logs.time)));
    end

    if strcmp(cfg.mode, 'ros')
        publish_zero_commands(ros_io);
        if cfg.land_on_finish && did_takeoff && ros_io.use_drone
            pause(0.25);
            publish_empty_message(ros_io.land_pub);
        end
    end

    if ishghandle(stop_monitor)
        close(stop_monitor);
    end

    if cfg.save_logs
        save_info = resolve_output_paths(cfg);
        save_logs_mat(save_info.log_path, logs, cfg, interruption_reason);
        fprintf('Log salvo em %s\n', save_info.log_path);

        if cfg.analyze_logs
            try
                metrics = compute_metrics_from_logs(logs);
                print_metrics(metrics);
                metrics_path = save_metrics_csv(metrics, save_info.output_dir);
                export_results(logs, save_info.output_dir, cfg.show_figures, cfg.formation);
                fprintf('Metricas salvas em %s\n', metrics_path);
            catch plot_exception
                warning('formation_controller:plotResultsFailed', ...
                        'Falha ao gerar analise MATLAB automaticamente: %s', plot_exception.message);
            end
        end
    end

    result = struct();
    result.config = cfg;
    result.logs = logs;
    result.output_dir = save_info.output_dir;
    result.log_path = save_info.log_path;
    result.interruption_reason = interruption_reason;

    if ~isempty(interruption_reason)
        fprintf('Execucao encerrada por %s.\n', interruption_reason);
    end

    if ~isempty(caught_exception)
        rethrow(caught_exception);
    end
end

function cfg = normalize_config(cfg)
    cfg.mode = lower(string(cfg.mode));
    if ~ismember(cfg.mode, ["simulation", "ros"])
        error('formation_controller:invalidMode', ...
              'config.mode deve ser ''simulation'' ou ''ros''.');
    end
    cfg.mode = char(cfg.mode);
    cfg.duration = double(cfg.duration);
    cfg.rate_hz = double(cfg.rate_hz);
    cfg.real_time = logical(cfg.real_time);
    cfg.takeoff = logical(cfg.takeoff);
    cfg.land_on_finish = logical(cfg.land_on_finish);
    cfg.show_monitor = logical(cfg.show_monitor);
    cfg.show_figures = logical(cfg.show_figures);
    cfg.save_logs = logical(cfg.save_logs);
    cfg.analyze_logs = logical(cfg.analyze_logs);
    cfg.ros.only_limo = logical(cfg.ros.only_limo);
    cfg.ros.only_bebop = logical(cfg.ros.only_bebop);

    cfg.limo.command_type = char(lower(string(cfg.limo.command_type)));
    if ~ismember(cfg.limo.command_type, {'velocity', 'dynamic'})
        error('formation_controller:invalidLimoCommandType', ...
              'config.limo.command_type deve ser ''velocity'' ou ''dynamic''.');
    end
end

function merged = merge_structs(base, overrides)
    merged = base;
    names = fieldnames(overrides);
    for i = 1:numel(names)
        name = names{i};
        if isfield(base, name) && isstruct(base.(name)) && isstruct(overrides.(name))
            merged.(name) = merge_structs(base.(name), overrides.(name));
        else
            merged.(name) = overrides.(name);
        end
    end
end

function state = initialize_state(cfg)
    state = struct();
    state.limo = struct('pose', double(cfg.initial.limo_pose(:).'), 'velocity', zeros(1, 2));
    state.drone = struct('pose', double(cfg.initial.drone_pose(:).'), 'body_velocity', zeros(1, 4));
end

function logs = initialize_logs(n)
    logs = struct();
    logs.time = nan(n, 1);
    logs.formation_state = nan(n, 6);
    logs.formation_reference = nan(n, 6);
    logs.limo_pose = nan(n, 3);
    logs.drone_pose = nan(n, 4);
    logs.limo_command = nan(n, 2);
    logs.drone_command = nan(n, 4);
    logs.obstacle_distance = nan(n, 1);
end

function logs = append_log_sample(logs, idx, t, state, commands)
    logs.time(idx, 1) = t;
    logs.formation_state(idx, :) = commands.q;
    logs.formation_reference(idx, :) = commands.qd;
    logs.limo_pose(idx, :) = state.limo.pose;
    logs.drone_pose(idx, :) = state.drone.pose;
    logs.limo_command(idx, :) = commands.limo_command;
    logs.drone_command(idx, :) = commands.drone_command;
    logs.obstacle_distance(idx, 1) = commands.obstacle_distance;
end

function logs = trim_logs(logs, used)
    used = max(0, min(used, size(logs.time, 1)));
    names = fieldnames(logs);
    for i = 1:numel(names)
        value = logs.(names{i});
        logs.(names{i}) = value(1:used, :);
    end
end

function stop_monitor = create_emergency_stop_monitor()
    stop_monitor = figure( ...
        'Name', 'Emergency Stop Monitor', ...
        'NumberTitle', 'off', ...
        'MenuBar', 'none', ...
        'ToolBar', 'none', ...
        'Color', [0.10, 0.10, 0.10], ...
        'KeyPressFcn', @emergency_stop_keypress, ...
        'CloseRequestFcn', @emergency_stop_close_request);
    setappdata(stop_monitor, 'emergency_stop', false);
    uicontrol( ...
        stop_monitor, ...
        'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.08, 0.20, 0.84, 0.60], ...
        'BackgroundColor', [0.10, 0.10, 0.10], ...
        'ForegroundColor', [1.0, 1.0, 1.0], ...
        'FontSize', 14, ...
        'String', sprintf('Pressione Q, Space ou Esc para parar e pousar o drone.\nMantenha esta janela selecionada durante o ensaio.'));
end

function emergency_stop_keypress(src, event)
    key = lower(string(event.Key));
    if any(strcmp(key, ["q", "space", "escape"]))
        setappdata(src, 'emergency_stop', true);
    end
end

function emergency_stop_close_request(src, ~)
    setappdata(src, 'emergency_stop', true);
    delete(src);
end

function tf = emergency_stop_requested(stop_monitor)
    tf = false;
    if isempty(stop_monitor)
        return;
    end
    drawnow limitrate;
    if ~ishghandle(stop_monitor)
        tf = true;
        return;
    end
    tf = isappdata(stop_monitor, 'emergency_stop') && getappdata(stop_monitor, 'emergency_stop');
end

function ros_io = setup_ros_io(cfg)
    if cfg.ros.reinitialize
        try
            rosshutdown;
            pause(0.5);
        catch
        end
    end

    try
        rosinit(cfg.ros.master_uri);
    catch exc
        if ~contains(exc.message, 'Cannot create', 'IgnoreCase', true) && ...
           ~contains(exc.message, 'already exists', 'IgnoreCase', true)
            rethrow(exc);
        end
    end

    use_limo = ~cfg.ros.only_bebop;
    use_drone = ~cfg.ros.only_limo;

    ros_io = struct();
    ros_io.use_limo = use_limo;
    ros_io.use_drone = use_drone;
    
    if use_limo
        ros_io.pose_limo_sub = rossubscriber(sprintf('%s/%s/pose', cfg.ros.pose_prefix, cfg.ros.limo_ns), 'geometry_msgs/PoseStamped');
        ros_io.cmd_limo_pub = rospublisher(sprintf('/%s/cmd_vel', cfg.ros.limo_ns), 'geometry_msgs/Twist');
    else
        ros_io.pose_limo_sub = [];
        ros_io.cmd_limo_pub = [];
    end
    
    if use_drone
        ros_io.pose_drone_sub = rossubscriber(sprintf('%s/%s/pose', cfg.ros.pose_prefix, cfg.ros.drone_ns), 'geometry_msgs/PoseStamped');
        ros_io.cmd_drone_pub = rospublisher(sprintf('/%s/cmd_vel', cfg.ros.drone_ns), 'geometry_msgs/Twist');
        ros_io.takeoff_pub = rospublisher(sprintf('/%s/takeoff', cfg.ros.drone_ns), 'std_msgs/Empty');
        ros_io.land_pub = rospublisher(sprintf('/%s/land', cfg.ros.drone_ns), 'std_msgs/Empty');
    else
        ros_io.pose_drone_sub = [];
        ros_io.cmd_drone_pub = [];
        ros_io.takeoff_pub = [];
        ros_io.land_pub = [];
    end
    
    ros_io.last_limo_pose = [];
    ros_io.last_drone_pose = [];
    ros_io.last_limo_stamp = [];
    ros_io.last_drone_stamp = [];
end

function publish_empty_message(pub)
    msg = rosmessage(pub);
    send(pub, msg);
end

function [state, ros_io, ready] = update_state_from_ros(state, ros_io, cfg)
    ready = false;
    now_s = posixtime(datetime('now'));
    
    % Obtem mensagens apenas dos robos ativos
    limo_msg = [];
    drone_msg = [];
    
    if ros_io.use_limo
        limo_msg = get_latest_message(ros_io.pose_limo_sub);
        if isempty(limo_msg)
            return;
        end
    end
    
    if ros_io.use_drone
        drone_msg = get_latest_message(ros_io.pose_drone_sub);
        if isempty(drone_msg)
            return;
        end
    end

    % Valida timestamps apenas dos robos ativos
    if ros_io.use_limo
        limo_stamp = get_message_stamp_seconds(limo_msg, now_s);
        if (now_s - limo_stamp) > cfg.ros.pose_timeout_s
            return;
        end
    end
    
    if ros_io.use_drone
        drone_stamp = get_message_stamp_seconds(drone_msg, now_s);
        if (now_s - drone_stamp) > cfg.ros.pose_timeout_s
            return;
        end
    end

    % Atualiza pose e velocidade do LIMO se estiver ativo
    if ros_io.use_limo
        limo_pose = pose_stamped_to_limo_pose(limo_msg);
        
        if ~isempty(ros_io.last_limo_pose) && ~isempty(ros_io.last_limo_stamp)
            dt_pose = max(limo_stamp - ros_io.last_limo_stamp, 1.0e-3);
            cp_now = limo_control_point_from_pose(limo_pose, cfg.limo.a);
            cp_prev = limo_control_point_from_pose(ros_io.last_limo_pose, cfg.limo.a);
            cp_vel = (cp_now - cp_prev) / dt_pose;
            yaw = limo_pose(3);
            u = cp_vel(1) * cos(yaw) + cp_vel(2) * sin(yaw);
            omega = (-cp_vel(1) * sin(yaw) + cp_vel(2) * cos(yaw)) / cfg.limo.a;
            state.limo.velocity = [u, omega];
        end
        
        state.limo.pose = limo_pose;
        ros_io.last_limo_pose = limo_pose;
        ros_io.last_limo_stamp = limo_stamp;
    end

    % Atualiza pose e velocidade do drone se estiver ativo
    if ros_io.use_drone
        drone_pose = pose_stamped_to_drone_pose(drone_msg);
        
        if ~isempty(ros_io.last_drone_pose) && ~isempty(ros_io.last_drone_stamp)
            dt_pose = max(drone_stamp - ros_io.last_drone_stamp, 1.0e-3);
            world_velocity = (drone_pose - ros_io.last_drone_pose) / dt_pose;
            state.drone.body_velocity = (rotation_world_from_body(drone_pose(4)) \ world_velocity.').';
        end
        
        state.drone.pose = drone_pose;
        ros_io.last_drone_pose = drone_pose;
        ros_io.last_drone_stamp = drone_stamp;
    end

    ready = true;
end

function msg = get_latest_message(sub)
    msg = [];
    if isempty(sub)
        return;
    end
    try
        msg = sub.LatestMessage;
        if isempty(msg)
            return;
        end
        % Valida a mensagem tentando acessar o campo esperado do PoseStamped.
        dummy_pose = msg.Pose; %#ok<NASGU>
    catch
        msg = [];
    end
end

function stamp_s = get_message_stamp_seconds(msg, fallback_now)
    stamp_s = fallback_now;
    try
        sec = double(msg.Header.Stamp.Sec);
        nsec = double(msg.Header.Stamp.Nsec);
        candidate = sec + 1.0e-9 * nsec;
        if isfinite(candidate) && candidate > 0.0
            stamp_s = candidate;
        end
    catch
    end
end

function pose = pose_stamped_to_limo_pose(msg)
    yaw = quaternion_to_yaw(msg.Pose.Orientation.W, msg.Pose.Orientation.X, msg.Pose.Orientation.Y, msg.Pose.Orientation.Z);
    pose = [double(msg.Pose.Position.X), double(msg.Pose.Position.Y), yaw];
end

function pose = pose_stamped_to_drone_pose(msg)
    yaw = quaternion_to_yaw(msg.Pose.Orientation.W, msg.Pose.Orientation.X, msg.Pose.Orientation.Y, msg.Pose.Orientation.Z);
    pose = [double(msg.Pose.Position.X), double(msg.Pose.Position.Y), double(msg.Pose.Position.Z), yaw];
end

function yaw = quaternion_to_yaw(w, x, y, z)
    siny_cosp = 2.0 * (w * z + x * y);
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
end

function [commands, prev_refs] = compute_commands(state, cfg, prev_refs, dt, t)
    q = current_formation_state(state, cfg);
    [qd, qd_dot] = formation_reference(t);
    [formation_velocity, obstacle_distance] = formation_outer_loop(qd, qd_dot, q, cfg.formation);

    limo_reference = limo_velocity_reference(formation_velocity(1:2), state.limo.pose(3), cfg.limo.a);
    if strcmp(cfg.limo.command_type, 'dynamic')
        [limo_command, prev_refs.limo] = limo_dynamic_controller(limo_reference, state.limo.velocity, prev_refs.limo, dt, cfg.limo);
    else
        % 'velocity': envia a referencia cinematica direto, apenas saturando
        % nos limites de comando. Correto para o cmd_vel do LIMO real.
        limo_command = clamp_symmetric(limo_reference, cfg.limo.command_limits);
        prev_refs.limo = limo_reference;
    end

    drone_target_position = drone_desired_position(qd);
    drone_position_error = drone_target_position - state.drone.pose(1:3);
    drone_world_velocity = (drone_formation_jacobian(q) * formation_velocity.').';
    yaw_error = wrap_to_pi(cfg.drone.yaw_reference - state.drone.pose(4));

    drone_aux_world = [drone_world_velocity, 0.0];
    drone_aux_world(1:3) = drone_aux_world(1:3) + smooth_term(cfg.drone.kinematic_gains(1:3), cfg.drone.kinematic_limits(1:3), drone_position_error);
    drone_aux_world(4) = smooth_term(cfg.drone.kinematic_gains(4), cfg.drone.kinematic_limits(4), yaw_error);

    drone_velocity_reference = (rotation_world_from_body(state.drone.pose(4)) \ drone_aux_world.').';
    [drone_command, prev_refs.drone] = drone_dynamic_controller(drone_velocity_reference, state.drone.body_velocity, prev_refs.drone, dt, cfg.drone);

    commands = struct();
    commands.q = q;
    commands.qd = qd;
    commands.obstacle_distance = obstacle_distance;
    commands.limo_reference = limo_reference;
    commands.limo_command = limo_command;
    commands.drone_reference = drone_velocity_reference;
    commands.drone_command = drone_command;
end

function q = current_formation_state(state, cfg)
    control_point = limo_control_point_from_pose(state.limo.pose, cfg.limo.a);
    rel = state.drone.pose(1:3) - control_point;
    rho = norm(rel);
    planar = norm(rel(1:2));

    if planar > 1.0e-6
        alpha = atan2(rel(2), rel(1));
    else
        alpha = 0.0;
    end

    if rho > 1.0e-6
        beta = atan2(rel(3), planar);
    else
        beta = pi / 2.0;
    end

    q = [control_point(1), control_point(2), 0.0, rho, alpha, beta];
end

function point = limo_control_point_from_pose(pose, a)
    point = [pose(1) + a * cos(pose(3)), pose(2) + a * sin(pose(3)), 0.0];
end

function [qd, qd_dot] = formation_reference(t)
    omega = 2.0 * pi / 40.0;
    xf = 0.75 * sin(omega * t);
    yf = 0.75 * sin(2.0 * omega * t);
    xfd = 0.75 * omega * cos(omega * t);
    yfd = 0.75 * 2.0 * omega * cos(2.0 * omega * t);
    qd = [xf, yf, 0.0, 1.5, 0.0, pi / 2.0];
    qd_dot = [xfd, yfd, 0.0, 0.0, 0.0, 0.0];
end

function error_q = formation_error(qd, q)
    error_q = qd - q;
    error_q(5) = wrap_to_pi(error_q(5));
    error_q(6) = wrap_to_pi(error_q(6));
end

function [formation_velocity, distance] = formation_outer_loop(qd, qd_dot, q, formation)
    error_q = formation_error(qd, q);
    secondary = qd_dot + smooth_term(formation.control_gains, formation.control_limits, error_q);

    delta = q(1:2) - formation.obstacle_center;
    distance = norm(delta);
    influence = formation.obstacle_influence_radius;
    if distance < 1.0e-6 || distance >= influence
        formation_velocity = secondary;
        return;
    end

    radius = formation.obstacle_radius;
    activation = (influence - distance) / max(influence - radius, 1.0e-6);
    activation = min(max(activation, 0.0), 1.0);
    activation = activation * activation * (3.0 - 2.0 * activation);

    normal = delta / distance;
    j_obs = zeros(1, 6);
    j_obs(1:2) = normal;
    scalar_speed = min(formation.obstacle_speed_limit, formation.obstacle_gain * (influence - distance));
    j_norm_sq = j_obs * j_obs.';
    j_pinv = j_obs.' / j_norm_sq;
    projector = eye(6) - j_pinv * j_obs;
    primary = (j_pinv(:, 1) * scalar_speed).';
    obstacle_task = primary + (projector * secondary.').';
    formation_velocity = (1.0 - activation) * secondary + activation * obstacle_task;
end

function reference = limo_velocity_reference(world_velocity, yaw, a)
    nu_x = world_velocity(1);
    nu_y = world_velocity(2);
    u_d = nu_x * cos(yaw) + nu_y * sin(yaw);
    omega_d = (-nu_x * sin(yaw) + nu_y * cos(yaw)) / a;
    reference = [u_d, omega_d];
end

function [command, prev_reference] = limo_dynamic_controller(reference, current_velocity, prev_reference, dt, limo)
    if isempty(prev_reference)
        derivative = zeros(size(reference));
    else
        derivative = (reference - prev_reference) / dt;
    end
    prev_reference = reference;
    drift = limo_drift(current_velocity, limo.theta);
    input_matrix_inverse = diag([limo.theta(1), limo.theta(2)]);
    command = (input_matrix_inverse * (derivative.' + limo.dynamic_gains * (reference - current_velocity).' - drift.')).';
    command = clamp_symmetric(command, limo.command_limits);
end

function drift = limo_drift(velocity, theta)
    u = velocity(1);
    omega = velocity(2);
    drift = [ ...
        (theta(3) / theta(1)) * (omega ^ 2) - (theta(4) / theta(1)) * u, ...
        -(theta(5) / theta(2)) * u * omega - (theta(6) / theta(2)) * omega];
end

function [command, prev_reference] = drone_dynamic_controller(reference, current_velocity, prev_reference, dt, drone)
    if isempty(prev_reference)
        derivative = zeros(size(reference));
    else
        derivative = (reference - prev_reference) / dt;
    end
    prev_reference = reference;
    command = (drone.ku \ (derivative.' + drone.dynamic_gains * (reference - current_velocity).' + drone.kv * current_velocity.')).';
    command = clamp_symmetric(command, drone.command_limits);
end

function target = drone_desired_position(q)
    xf = q(1);
    yf = q(2);
    zf = q(3);
    rho = q(4);
    alpha = q(5);
    beta = q(6);
    planar = rho * cos(beta);
    target = [xf + planar * cos(alpha), yf + planar * sin(alpha), zf + rho * sin(beta)];
end

function jac = drone_formation_jacobian(q)
    rho = q(4);
    alpha = q(5);
    beta = q(6);
    cos_alpha = cos(alpha);
    sin_alpha = sin(alpha);
    cos_beta = cos(beta);
    sin_beta = sin(beta);

    jac = [ ...
        1.0, 0.0, 0.0, cos_beta * cos_alpha, -rho * cos_beta * sin_alpha, -rho * sin_beta * cos_alpha; ...
        0.0, 1.0, 0.0, cos_beta * sin_alpha, rho * cos_beta * cos_alpha, -rho * sin_beta * sin_alpha; ...
        0.0, 0.0, 1.0, sin_beta, 0.0, rho * cos_beta];
end

function state = step_simulation(state, cfg, commands, dt)
    input_matrix = diag([1.0 / cfg.limo.theta(1), 1.0 / cfg.limo.theta(2)]);
    limo_acc = limo_drift(state.limo.velocity, cfg.limo.theta) + (input_matrix * commands.limo_command.').';
    state.limo.velocity = state.limo.velocity + dt * limo_acc;
    u = state.limo.velocity(1);
    omega = state.limo.velocity(2);
    x = state.limo.pose(1);
    y = state.limo.pose(2);
    psi = state.limo.pose(3);
    state.limo.pose = [ ...
        x + dt * u * cos(psi), ...
        y + dt * u * sin(psi), ...
        wrap_to_pi(psi + dt * omega)];

    body_acc = (cfg.drone.ku * commands.drone_command.' - cfg.drone.kv * state.drone.body_velocity.').';
    state.drone.body_velocity = state.drone.body_velocity + dt * body_acc;
    world_velocity = (rotation_world_from_body(state.drone.pose(4)) * state.drone.body_velocity.').';
    state.drone.pose = [ ...
        state.drone.pose(1) + dt * world_velocity(1), ...
        state.drone.pose(2) + dt * world_velocity(2), ...
        state.drone.pose(3) + dt * world_velocity(3), ...
        wrap_to_pi(state.drone.pose(4) + dt * world_velocity(4))];
end

function publish_ros_commands(ros_io, commands)
    if ros_io.use_limo
        limo_msg = rosmessage(ros_io.cmd_limo_pub);
        limo_msg.Linear.X = commands.limo_command(1);
        limo_msg.Angular.Z = commands.limo_command(2);
        send(ros_io.cmd_limo_pub, limo_msg);
    end

    if ros_io.use_drone
        drone_msg = rosmessage(ros_io.cmd_drone_pub);
        drone_msg.Linear.X = commands.drone_command(1);
        drone_msg.Linear.Y = commands.drone_command(2);
        drone_msg.Linear.Z = commands.drone_command(3);
        drone_msg.Angular.Z = commands.drone_command(4);
        send(ros_io.cmd_drone_pub, drone_msg);
    end
end

function publish_zero_commands(ros_io)
    if isempty(ros_io)
        return;
    end
    
    if ros_io.use_limo
        limo_msg = rosmessage(ros_io.cmd_limo_pub);
        send(ros_io.cmd_limo_pub, limo_msg);
    end
    
    if ros_io.use_drone
        drone_msg = rosmessage(ros_io.cmd_drone_pub);
        send(ros_io.cmd_drone_pub, drone_msg);
    end
end

function wait_for_next_sample(cfg, dt, completed_samples, loop_clock)
    if strcmp(cfg.mode, 'ros') || cfg.real_time
        target_elapsed = completed_samples * dt;
        remaining = target_elapsed - toc(loop_clock);
        if remaining > 0.0
            pause(remaining);
        end
    end
end

function value = smooth_term(gains, limits, error_value)
    value = limits .* tanh(gains .* error_value);
end

function angle = wrap_to_pi(angle)
    angle = atan2(sin(angle), cos(angle));
end

function rot = rotation_world_from_body(yaw)
    rot = [ ...
        cos(yaw), -sin(yaw), 0.0, 0.0; ...
        sin(yaw),  cos(yaw), 0.0, 0.0; ...
        0.0,       0.0,      1.0, 0.0; ...
        0.0,       0.0,      0.0, 1.0];
end

function value = clamp_symmetric(value, limits)
    value = min(max(value, -limits), limits);
end

function save_info = resolve_output_paths(cfg)
    if ~exist(cfg.results_dir, 'dir')
        mkdir(cfg.results_dir);
    end

    if ~isempty(cfg.output_logfile)
        log_path = char(string(cfg.output_logfile));
        output_dir = fileparts(log_path);
        if isempty(output_dir)
            output_dir = pwd;
            log_path = fullfile(output_dir, log_path);
        end
        if ~exist(output_dir, 'dir')
            mkdir(output_dir);
        end
    else
        timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
        output_dir = fullfile(cfg.results_dir, sprintf('resultado_formacao_%s', timestamp));
        mkdir(output_dir);
        log_path = fullfile(output_dir, 'resultado_formacao.mat');
    end

    save_info = struct('output_dir', output_dir, 'log_path', log_path);
end

function save_logs_mat(path, logs, cfg, interruption_reason)
    time = logs.time; %#ok<NASGU>
    formation_state = logs.formation_state; %#ok<NASGU>
    formation_reference = logs.formation_reference; %#ok<NASGU>
    limo_pose = logs.limo_pose; %#ok<NASGU>
    drone_pose = logs.drone_pose; %#ok<NASGU>
    limo_command = logs.limo_command; %#ok<NASGU>
    drone_command = logs.drone_command; %#ok<NASGU>
    obstacle_distance = logs.obstacle_distance; %#ok<NASGU>
    controller_config = cfg; %#ok<NASGU>
    stop_reason = interruption_reason; %#ok<NASGU>
    save(path, 'time', 'formation_state', 'formation_reference', 'limo_pose', 'drone_pose', ...
         'limo_command', 'drone_command', 'obstacle_distance', 'controller_config', 'stop_reason');
end

function errs = compute_error_series(logs)
    formation_error = logs.formation_reference - logs.formation_state;
    formation_error(:, 5) = wrap_to_pi(formation_error(:, 5));
    formation_error(:, 6) = wrap_to_pi(formation_error(:, 6));

    errs = struct();
    errs.xy_error = sqrt(sum(formation_error(:, 1:2) .^ 2, 2));
    errs.rho_error = abs(formation_error(:, 4));
    errs.alpha_error_deg = rad2deg(abs(formation_error(:, 5)));
    errs.beta_error_deg = rad2deg(abs(formation_error(:, 6)));
    errs.altitude_error = abs(logs.drone_pose(:, 3) - logs.formation_reference(:, 4));
    errs.formation_error = formation_error;
end

function metrics = compute_metrics_from_logs(logs)
    errs = compute_error_series(logs);
    t = logs.time(:);
    obstacle_distance = logs.obstacle_distance(:);

    if numel(t) > 1
        duration_s = t(end) - t(1);
    else
        duration_s = 0.0;
    end

    metrics = struct( ...
        'duration_s', duration_s, ...
        'mean_xy_error_m', mean(errs.xy_error), ...
        'max_xy_error_m', max(errs.xy_error), ...
        'mean_rho_error_m', mean(errs.rho_error), ...
        'max_rho_error_m', max(errs.rho_error), ...
        'mean_alpha_error_deg', mean(errs.alpha_error_deg), ...
        'max_alpha_error_deg', max(errs.alpha_error_deg), ...
        'mean_beta_error_deg', mean(errs.beta_error_deg), ...
        'max_beta_error_deg', max(errs.beta_error_deg), ...
        'mean_altitude_error_m', mean(errs.altitude_error), ...
        'max_altitude_error_m', max(errs.altitude_error), ...
        'min_obstacle_distance_m', min(obstacle_distance));
end

function print_metrics(metrics)
    fprintf('Metricas do experimento\n');
    fields = fieldnames(metrics);
    for i = 1:numel(fields)
        fprintf('- %s: %.6f\n', fields{i}, metrics.(fields{i}));
    end
end

function csv_path = save_metrics_csv(metrics, output_dir)
    csv_path = fullfile(output_dir, 'metrics.csv');
    fid = fopen(csv_path, 'w');
    if fid == -1
        error('formation_controller:csvWriteError', 'Nao foi possivel criar %s', csv_path);
    end
    cleanup_fid = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'metric,value\n');
    fields = fieldnames(metrics);
    for i = 1:numel(fields)
        fprintf(fid, '%s,%.10f\n', fields{i}, metrics.(fields{i}));
    end
end

function export_results(logs, output_dir, show, formation)
    plot_trajectory_xy(logs, output_dir, show);
    plot_altitude(logs, output_dir, show);
    plot_formation_variables(logs, output_dir, show);
    plot_formation_errors(logs, output_dir, show);
    plot_limo_commands(logs, output_dir, show);
    plot_drone_commands(logs, output_dir, show);
    plot_obstacle_distance(logs, output_dir, show, formation);
end

function plot_trajectory_xy(logs, output_dir, show)
    fig = figure('Visible', bool_to_onoff(show));
    hold on;
    plot(logs.formation_reference(:, 1), logs.formation_reference(:, 2), 'r--', 'DisplayName', 'referencia');
    plot(logs.formation_state(:, 1), logs.formation_state(:, 2), 'b', 'DisplayName', 'LIMO controle');
    plot(logs.drone_pose(:, 1), logs.drone_pose(:, 2), 'g', 'DisplayName', 'drone');
    hold off;
    title('Plano XY');
    xlabel('x [m]');
    ylabel('y [m]');
    axis equal;
    grid on;
    legend('show', 'Location', 'best');
    save_figure(fig, fullfile(output_dir, '01_trajetoria_xy.png'), show);
end

function plot_altitude(logs, output_dir, show)
    fig = figure('Visible', bool_to_onoff(show));
    hold on;
    plot(logs.time(:), logs.drone_pose(:, 3), 'DisplayName', 'z drone');
    plot(logs.time(:), logs.formation_reference(:, 4), 'r--', 'DisplayName', 'z ref');
    hold off;
    title('Altitude do drone');
    xlabel('tempo [s]');
    ylabel('z [m]');
    grid on;
    legend('show', 'Location', 'best');
    save_figure(fig, fullfile(output_dir, '02_altitude_drone.png'), show);
end

function plot_formation_variables(logs, output_dir, show)
    t = logs.time(:);
    fig = figure('Visible', bool_to_onoff(show));
    hold on;
    plot(t, logs.formation_state(:, 4), 'DisplayName', 'rho');
    plot(t, logs.formation_reference(:, 4), '--', 'DisplayName', 'rho ref');
    plot(t, rad2deg(logs.formation_state(:, 5)), 'DisplayName', 'alpha [deg]');
    plot(t, rad2deg(logs.formation_reference(:, 5)), '--', 'DisplayName', 'alpha ref [deg]');
    plot(t, rad2deg(logs.formation_state(:, 6)), 'DisplayName', 'beta [deg]');
    plot(t, rad2deg(logs.formation_reference(:, 6)), '--', 'DisplayName', 'beta ref [deg]');
    hold off;
    title('Variaveis de formacao');
    xlabel('tempo [s]');
    grid on;
    legend('show', 'Location', 'best');
    save_figure(fig, fullfile(output_dir, '03_variaveis_formacao.png'), show);
end

function plot_formation_errors(logs, output_dir, show)
    t = logs.time(:);
    errs = compute_error_series(logs);
    fe = errs.formation_error;
    fig = figure('Visible', bool_to_onoff(show));
    hold on;
    plot(t, fe(:, 1), 'DisplayName', 'erro x_f [m]');
    plot(t, fe(:, 2), 'DisplayName', 'erro y_f [m]');
    plot(t, errs.rho_error, 'DisplayName', '|erro rho| [m]');
    plot(t, errs.alpha_error_deg, 'DisplayName', '|erro alpha| [deg]');
    plot(t, errs.beta_error_deg, 'DisplayName', '|erro beta| [deg]');
    hold off;
    title('Erros da formacao');
    xlabel('tempo [s]');
    grid on;
    legend('show', 'Location', 'best');
    save_figure(fig, fullfile(output_dir, '04_erros_formacao.png'), show);
end

function plot_limo_commands(logs, output_dir, show)
    t = logs.time(:);
    fig = figure('Visible', bool_to_onoff(show));
    hold on;
    plot(t, logs.limo_command(:, 1), 'DisplayName', 'u_r');
    plot(t, logs.limo_command(:, 2), 'DisplayName', 'omega_r');
    hold off;
    title('Comandos do LIMO');
    xlabel('tempo [s]');
    grid on;
    legend('show', 'Location', 'best');
    save_figure(fig, fullfile(output_dir, '05_comandos_limo.png'), show);
end

function plot_drone_commands(logs, output_dir, show)
    t = logs.time(:);
    fig = figure('Visible', bool_to_onoff(show));
    hold on;
    plot(t, logs.drone_command(:, 1), 'DisplayName', 'vx');
    plot(t, logs.drone_command(:, 2), 'DisplayName', 'vy');
    plot(t, logs.drone_command(:, 3), 'DisplayName', 'vz');
    plot(t, logs.drone_command(:, 4), 'DisplayName', 'r');
    hold off;
    title('Comandos do drone');
    xlabel('tempo [s]');
    grid on;
    legend('show', 'Location', 'best');
    save_figure(fig, fullfile(output_dir, '06_comandos_drone.png'), show);
end

function plot_obstacle_distance(logs, output_dir, show, formation)
    t = logs.time(:);
    fig = figure('Visible', bool_to_onoff(show));
    hold on;
    plot(t, logs.obstacle_distance(:), 'DisplayName', 'distancia ao obstaculo');
    yline(formation.obstacle_influence_radius, '--', 'Color', [0.85 0.45 0], 'DisplayName', 'zona influencia');
    yline(formation.obstacle_radius, ':', 'Color', [0.75 0.0 0.0], 'DisplayName', 'raio obstaculo');
    hold off;
    title('Distancia ao obstaculo');
    xlabel('tempo [s]');
    ylabel('distancia [m]');
    grid on;
    legend('show', 'Location', 'best');
    save_figure(fig, fullfile(output_dir, '07_distancia_obstaculo.png'), show);
end

function save_figure(fig, path, show)
    exportgraphics(fig, path, 'Resolution', 200);
    fprintf('Grafico salvo em %s\n', path);
    if ~show
        close(fig);
    end
end

function s = bool_to_onoff(tf)
    if tf
        s = 'on';
    else
        s = 'off';
    end
end
