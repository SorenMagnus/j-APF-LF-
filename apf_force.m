% apf_force.m
% 计算 APF 力（吸引力 + 斥力 + 旋转分量）返回一个“虚拟力”用于引导车辆方向/速度调整
% 输入：
%   vehObj: vehicleDynamics 对象
%   scene: 场景结构（包含 obstacles, road boundaries 等）
%   params: apf 参数结构
% 输出：
%   f: struct with fields fx, fy (force vector)

function f = apf_force(vehObj, scene, params)
    % Get current state
    s = vehObj.getState();
    pos = [s.x, s.y];
    goal = vehObj.goal;
    % Attractive force to goal
    diff = goal - pos;
    dist_goal = norm(diff) + params.apf.eps;
    f_att = params.apf.k_att * diff / dist_goal;
    
    % Repulsive forces from obstacles (dynamic)
    f_rep = [0,0];
    for i=1:numel(scene.obstacles)
        ob = scene.obstacles{i};
        % Predict obstacle future position (short horizon)
        pred_t = 0.5; % seconds
        obpos = ob.pos + pred_t * ob.vel;
        vec = pos - obpos;
        d = norm(vec) + 1e-6;
        if d < params.apf.repulsive_range
            % inverse quadratic repulsion
            f_rep = f_rep + params.apf.k_rep * ((1/d - 1/params.apf.repulsive_range) / (d^2)) * (vec/d);
        end
    end
    
    % Repulse from other vehicles to avoid collision
    f_veh = [0,0];
    % If scene.vehicles_state available, we could use; fallback to scene.vehicles
    if isfield(scene,'vehicles')
        for j=1:numel(scene.vehicles)
            vj = scene.vehicles{j};
            if vj.id == vehObj.id, continue; end
            vec = pos - [vj.x, vj.y];
            d = norm(vec)+1e-6;
            if d < params.apf.veh_repulsive_range
                f_veh = f_veh + params.apf.k_rep * ((1/d - 1/params.apf.veh_repulsive_range) / (d^2)) * (vec/d);
            end
        end
    end
    
    % Road boundary repulsion (if provided)
    f_road = [0,0];
    if isfield(scene,'road')
        % treat road as centerline with two edges; compute distance to edges
        % For simplicity, if outside boundary, push inward
        pos_xy = pos;
        % compute signed distance to each segment in road.edges
        for k=1:size(scene.road.edges,1)
            p1 = scene.road.edges(k,1:2);
            p2 = scene.road.edges(k,3:4);
            % using point-to-segment distance
            [dseg, proj] = point2segment(pos_xy,p1,p2);
            if dseg < params.apf.repulsive_range/2
                vec = pos_xy - proj;
                f_road = f_road + params.apf.k_rep * ((1/dseg - 1/(params.apf.repulsive_range/2)) / (dseg^2)) * (vec/(dseg+1e-6));
            end
        end
    end
    
    % Rotational (tangential) component to escape local minima
    % For each repulsive source, add small perpendicular force
    f_rot = [0,0];
    if norm(f_rep + f_veh) > 1e-6
        base = f_rep + f_veh;
        perp = [ -base(2), base(1) ];
        perp = perp / (norm(perp)+1e-6);
        f_rot = params.apf.k_rot * perp;
    end
    
    f_total = f_att + f_rep + f_veh + f_road + f_rot;
    f.fx = f_total(1); f.fy = f_total(2);
end

function [d, proj] = point2segment(p, a, b)
    % point-to-segment distance and projection point
    v = b - a;
    w = p - a;
    c1 = dot(w,v);
    if c1 <= 0
        proj = a; d = norm(p-a); return;
    end
    c2 = dot(v,v);
    if c2 <= c1
        proj = b; d = norm(p-b); return;
    end
    t = c1/c2;
    proj = a + t*v;
    d = norm(p - proj);
end