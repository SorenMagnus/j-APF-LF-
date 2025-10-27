% main.m
% 仿真主程序：APF + Leader-Follower 编队控制 多车路径规划（动态障碍）
% 使用说明：将本文件与其它 .m 文件放在同一文件夹下，直接在 MATLAB 中运行 main

clc; clear; close all;

% 仿真参数
sim.dt = 0.1;           % 时间步长 (s)
sim.T  = 120;           % 最大仿真时长 (s)
sim.steps = ceil(sim.T/sim.dt);

% 场景参数
scene = scenarioGenerator(); % 生成道路、障碍、车辆初始状态（见 scenarioGenerator.m）

% 算法参数结构体（可在此处调整）
params.apf = struct('k_att',1.2,'k_rep',1.8,'repulsive_range',8,'veh_repulsive_range',3,'k_rot',0.5,'eps',0.1);
params.lf  = struct('k_p',1.0,'k_v',1.2,'desired_spacing',2.0,'max_speed',5.0,'min_speed',0.0);
params.safe_dist = 0.8; % 紧急制动触发安全距离 (m)

% 初始化车辆结构
N = numel(scene.vehicles);
veh = cell(1,N);
for i=1:N
    veh{i} = vehicleDynamics(scene.vehicles{i}); % see vehicleDynamics.m
end

% Data recording
history = struct();
history.pos = zeros(sim.steps, N, 2);
history.theta = zeros(sim.steps, N);
history.v = zeros(sim.steps, N);
history.minDistToObs = inf(sim.steps, N);
history.collisions = zeros(sim.steps, N);

% 主循环
figure('Name','APF+LF Simulation');
for tstep = 1:sim.steps
    t = (tstep-1)*sim.dt;
    % 更新动态障stacles位置（constant velocity model）
    scene = obstacleManager(scene, sim.dt);
    
    % For each vehicle compute control
    states = cellfun(@(v) v.getState(), veh, 'UniformOutput', false);
    statesMat = cell2mat(cellfun(@(s) [s.x; s.y; s.theta; s.v], states, 'UniformOutput', false));
    for i=1:N
        % Leader selection: assume vehicle 1 is leader
        isLeader = (i==1);
        % Compute APF force considering predicted dynamic obstacles and other vehicles
        f_apf = apf_force(veh{i}, scene, params); % see apf_force.m
        % Compute formation/leader-follower control
        u = leaderFollowerController(veh{i}, i, veh, scene, params, isLeader, f_apf); % see leaderFollowerController.m
        % Emergency braking if too close to obstacle
        minDist = collisionChecker(veh{i}, scene, params);
        if minDist < params.safe_dist
            % emergency brake
            u.v_ref = max(0, veh{i}.v - 2.0);
        end
        % Integrate dynamics
        veh{i}.step(u, sim.dt);
        
        % record
        s = veh{i}.getState();
        history.pos(tstep, i, :) = [s.x, s.y];
        history.theta(tstep, i) = s.theta;
        history.v(tstep, i) = s.v;
        history.minDistToObs(tstep,i) = minDist;
        history.collisions(tstep,i) = any(minDist < 0.1); % collision threshold
    end
    
    % Visualization at intervals
    if mod(tstep,2)==1 || tstep==1
        visualizeSimulation(veh, scene, t, params); % see visualizeSimulation.m
        drawnow;
    end
    
    % Termination condition: all vehicles near their goals
    allReached = true;
    for i=1:N
        if norm([veh{i}.x - veh{i}.goal(1), veh{i}.y - veh{i}.goal(2)]) > 0.5
            allReached = false; break;
        end
    end
    if allReached
        fprintf('All vehicles reached goals at t=%.2fs\n', t);
        break;
    end
end

% Post-processing: compute metrics and save
results = evaluateMetrics(history, veh, scene, sim); % see metrics.m
fprintf('Results: Success Rate (no collision) = %.2f\n', results.success_rate);
% Save history
save('simulation_results.mat','history','scene','params','results');
