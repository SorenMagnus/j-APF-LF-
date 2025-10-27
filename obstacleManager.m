% obstacleManager.m
% 更新场景中动态障碍物的位置（简单常速度模型），并支持生成新的突发障碍
% scene 保持 obstacle fields 为 cell array，每个 obstacle: struct('pos',[x,y],'vel',[vx,vy],'r')

function scene = obstacleManager(scene, dt)
    for i=1:numel(scene.obstacles)
        scene.obstacles{i}.pos = scene.obstacles{i}.pos + scene.obstacles{i}.vel * dt;
        % optionally bounce on boundaries
        if isfield(scene,'road')
            % check if outside road bounding box and reflect
            bbox = scene.road.bbox;
            if scene.obstacles{i}.pos(1) < bbox(1) || scene.obstacles{i}.pos(1) > bbox(2)
                scene.obstacles{i}.vel(1) = -scene.obstacles{i}.vel(1);
            end
            if scene.obstacles{i}.pos(2) < bbox(3) || scene.obstacles{i}.pos(2) > bbox(4)
                scene.obstacles{i}.vel(2) = -scene.obstacles{i}.vel(2);
            end
        end
    end
    % Optional: spawn sporadic obstacles (disabled by default)
    if isfield(scene,'spawnNew') && scene.spawnNew
        if rand < 0.01
            newOb.pos = [scene.road.bbox(1) + rand*(scene.road.bbox(2)-scene.road.bbox(1)), scene.road.bbox(3) + rand*(scene.road.bbox(4)-scene.road.bbox(3))];
            ang = rand*2*pi; sp = 1 + rand*3;
            newOb.vel = sp * [cos(ang), sin(ang)];
            newOb.r = 0.5 + rand*0.8;
            scene.obstacles{end+1} = newOb;
        end
    end
end