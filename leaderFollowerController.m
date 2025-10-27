% leaderFollowerController.m
% Leader-Follower + APF 融合控制器
% 返回控制量 u.steer (rad), u.v_ref (m/s)
% 对 leader：使用 APF 产生的力指示方向并跟踪全局路径；对 follower：基于相对偏置保持队形并受 APF 修正

function u = leaderFollowerController(vehObj, idx, vehAll, scene, params, isLeader, f_apf)
    s = vehObj.getState();
    pos = [s.x, s.y];
    theta = s.theta;
    % default
    u = struct('steer',0,'v_ref',min(vehObj.max_speed, params.lf.max_speed));
    
    % Leader behavior: follow APF direction + goal velocity profile
    if isLeader
        % desired heading from APF force
        fvec = [f_apf.fx, f_apf.fy];
        if norm(fvec) < 1e-6
            desired_theta = theta;
        else
            desired_theta = atan2(fvec(2), fvec(1));
        end
        steer = angdiff(desired_theta, theta);
        % speed reference scaled by projection of force on heading
        speed_ref = params.lf.max_speed * (1 - exp(-norm(fvec)));
        u.steer = steer;
        u.v_ref = min(params.lf.max_speed, max(params.lf.min_speed, speed_ref));
        return;
    end
    
    % Follower behavior: maintain desired offset to leader or predecessor
    % Default: follow vehicle idx-1
    if idx>1
        ref = vehAll{idx-1}.getState();
    else
        ref = vehAll{1}.getState();
    end
    desired_offset = [ -params.lf.desired_spacing, 0 ]; % behind leader in leader frame (x backward)
    % Transform desired_offset from leader body frame to global
    R = [cos(ref.theta), -sin(ref.theta); sin(ref.theta), cos(ref.theta)];
    desired_pos = [ref.x; ref.y] + R * desired_offset';
    err = desired_pos' - pos;
    % formation P controller
    v_ref_from_lf = params.lf.k_p * norm(err);
    desired_theta = atan2(err(2), err(1));
    steer = angdiff(desired_theta, theta);
    
    % Combine with APF: APF acts as modification to desired heading & speed
    fvec = [f_apf.fx, f_apf.fy];
    if norm(fvec) > 1e-6
        apf_theta = atan2(fvec(2), fvec(1));
        % Weighted combine: favor formation when close, APF when obstacle close
        dist_to_apf_goal = norm(vehObj.goal - pos);
        w_form = exp(-0.5*norm(err)); % closer => bigger weight
        w_apf = 1 - w_form;
        desired_theta = angdiff( (w_form*desired_theta + w_apf*apf_theta), 0); % blend angles
    end
    steer = angdiff(desired_theta, theta);
    v_ref = min(params.lf.max_speed, v_ref_from_lf + 0.5*norm(fvec));
    u.steer = steer;
    u.v_ref = max(params.lf.min_speed, v_ref);
end

function d = angdiff(a,b)
    % minimal angle difference a-b
    d = wrapToPi(a - b);
end