% collisionChecker.m
% 计算车辆到最近障碍物的距离（欧氏），返回最小距离
% 输入 vehObj, scene, params

function minDist = collisionChecker(vehObj, scene, params)
    s = vehObj.getState();
    pos = [s.x, s.y];
    minDist = inf;
    for i=1:numel(scene.obstacles)
        ob = scene.obstacles{i};
        d = norm(pos - ob.pos) - (ob.r + vehObj.radius);
        if d < minDist, minDist = d; end
    end
    % consider road bounds as obstacles (negative distance if outside)
    bbox = scene.road.bbox;
    % left/right top/bottom distances
    dx1 = pos(1) - bbox(1); dx2 = bbox(2) - pos(1);
    dy1 = pos(2) - bbox(3); dy2 = bbox(4) - pos(2);
    minEdge = min([dx1, dx2, dy1, dy2]);
    % if too close to edge, set minDist small
    minDist = min(minDist, minEdge - vehObj.radius);
end