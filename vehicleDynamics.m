% vehicleDynamics.m
% 车辆动力学（简化 unicycle 模型 + 速度微分方程）
% 使用：
%   v = vehicleDynamics(initStruct);
%   v.step(u, dt);
%   s = v.getState();

classdef vehicleDynamics < handle
    properties
        x
        y
        theta
        v
        id
        max_steer = pi/4
        max_acc = 2.0
        max_speed = 6.0
        goal = [0,0]
        radius = 0.6 % 车辆近似半径 (m)
    end
    methods
        function obj = vehicleDynamics(init)
            obj.x = init.x;
            obj.y = init.y;
            obj.theta = init.theta;
            obj.v = init.v;
            obj.id = init.id;
            obj.goal = init.goal;
            if isfield(init,'radius'), obj.radius = init.radius; end
            if isfield(init,'max_speed'), obj.max_speed = init.max_speed; end
        end
        
        function step(obj, u, dt)
            % u: struct with fields steer (rad), v_ref (m/s)
            % simple first-order speed controller
            dv = u.v_ref - obj.v;
            acc = max(-obj.max_acc, min(obj.max_acc, dv)); % P control ~ 1
            obj.v = max(0, min(obj.max_speed, obj.v + acc*dt));
            steer = max(-obj.max_steer, min(obj.max_steer, u.steer));
            % kinematic unicycle motion
            obj.theta = obj.theta + (obj.v * tan(steer) / 1.0) * dt; % L =1.0
            obj.x = obj.x + obj.v * cos(obj.theta) * dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * dt;
        end
        
        function s = getState(obj)
            s = struct('x',obj.x,'y',obj.y,'theta',obj.theta,'v',obj.v);
        end
    end
end