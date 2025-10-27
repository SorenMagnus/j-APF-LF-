% scenarioGenerator.m
% 生成基础道路、车辆初始状态与动态障碍
% 返回 scene 结构体，包含:
%   road: bbox, edges
%   vehicles: cell array of initial vehicle structs with fields x,y,theta,v,id,goal
%   obstacles: cell array of obstacle structs with pos, vel, r

function scene = scenarioGenerator()
    % Road bbox [xmin xmax ymin ymax]
    road.bbox = [-30, 30, -20, 20];
    % simple straight road edges for repulsion use
    road.edges = [ -30 -5, 30 -5;
                   -30  5, 30  5 ];
    scene.road = road;
    
    % Vehicles: set N=5 formation
    N = 5;
    vehicles = cell(1,N);
    start_x = -20;
    for i=1:N
        vehicles{i} = struct();
        vehicles{i}.x = start_x - (i-1)*2.5;
        vehicles{i}.y = 0 + (i-1)*0; % inline
        vehicles{i}.theta = 0;
        vehicles{i}.v = 0.5 + 0.2*(i-1);
        vehicles{i}.id = i;
        vehicles{i}.goal = [25, 0]; % same goal for simplicity
        vehicles{i}.radius = 0.6;
        vehicles{i}.max_speed = 5.0;
    end
    scene.vehicles = vehicles;
    
    % Obstacles: dynamic moving circles
    obstacles = {};
    % add few moving obstacles crossing the road
    o1.pos = [-5, -15]; o1.vel = [0, 1.5]; o1.r = 0.7;
    o2.pos = [5, 15]; o2.vel = [0, -1.8]; o2.r = 0.8;
    o3.pos = [0, -5]; o3.vel = [0.5, 0.3]; o3.r = 0.6;
    obstacles{1} = o1; obstacles{2} = o2; obstacles{3} = o3;
    scene.obstacles = obstacles;
    scene.spawnNew = false;
end