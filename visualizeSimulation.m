% visualizeSimulation.m
% 绘制场景、车辆与障碍（每次调用重绘）
function visualizeSimulation(vehAll, scene, t, params)
    clf;
    hold on; axis equal;
    % road bbox
    bbox = scene.road.bbox;
    rectangle('Position',[bbox(1), bbox(3), bbox(2)-bbox(1), bbox(4)-bbox(3)],'EdgeColor',[0.8 0.8 0.8],'LineStyle','--');
    % road edges
    for k=1:size(scene.road.edges,1)
        p1 = scene.road.edges(k,1:2);
        p2 = scene.road.edges(k,3:4);
        plot([p1(1),p2(1)],[p1(2),p2(2)],'k-','LineWidth',1);
    end
    % obstacles
    for i=1:numel(scene.obstacles)
        ob = scene.obstacles{i};
        viscircles(ob.pos, ob.r,'Color',[1 0 0],'LineWidth',0.8);
        quiver(ob.pos(1),ob.pos(2),ob.vel(1),ob.vel(2),'r');
    end
    % vehicles
    colors = lines(numel(vehAll));
    for i=1:numel(vehAll)
        v = vehAll{i}.getState();
        plot(v.x, v.y, 's','MarkerSize',8,'MarkerFaceColor',colors(i,:),'MarkerEdgeColor','k');
        % heading arrow
        quiver(v.x, v.y, cos(v.theta), sin(v.theta), 0.8,'Color',colors(i,:),'MaxHeadSize',1);
        % goal
        if ~isempty(vehAll{i}.goal)
            plot(vehAll{i}.goal(1), vehAll{i}.goal(2),'x','Color',colors(i,:));
        end
    end
    title(sprintf('APF+LF Simulation t=%.2fs', t));
    xlim([bbox(1)-2, bbox(2)+2]); ylim([bbox(3)-2, bbox(4)+2]);
    hold off;
end