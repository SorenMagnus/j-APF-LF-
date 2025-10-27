% evaluateMetrics.m
% 根据仿真历史计算指标
% 输入 history, veh final states, scene, sim
% 输出 results struct: success_rate, avg_goal_error, min_distances, collision_count

function results = evaluateMetrics(history, veh, scene, sim)
    steps = size(history.pos,1);
    N = size(history.pos,2);
    % collision if any recorded collisions
    coll_mask = any(history.collisions>0,1);
    collision_count = sum(coll_mask);
    results.collision_count = collision_count;
    results.success_rate = 1 - (collision_count / N);
    % average goal error at final time
    final_pos = squeeze(history.pos(min(steps, end),:,:));
    goal_errors = zeros(1,N);
    for i=1:N
        goal = veh{i}.goal;
        goal_errors(i) = norm(final_pos(i,:) - goal);
    end
    results.avg_goal_error = mean(goal_errors);
    results.goal_errors = goal_errors;
    % min distances to obstacles across time
    results.min_distances = min(history.minDistToObs,[],1);
    results.min_distances_all = history.minDistToObs;
    % travel times approximate: first time within threshold
    results.travel_time = zeros(1,N);
    for i=1:N
        arrived_idx = find(sqrt( (history.pos(:,i,1)-veh{i}.goal(1)).^2 + (history.pos(:,i,2)-veh{i}.goal(2)).^2 ) < 0.5, 1);
        if isempty(arrived_idx)
            results.travel_time(i) = NaN;
        else
            results.travel_time(i) = (arrived_idx-1) * sim.dt;
        end
    end
end