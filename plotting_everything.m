% UAV Arrow
% rt = size(DA.Paths,2)-1;
% Destination
pltDestin = scatter3(DA.destin(1,1),DA.destin(1,2),DA.destin(1,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5);

hold on, grid on, axis equal

% if animation
dxx = DA.traj{rt}(1,end)-DA.traj{rt}(1,1);
dyy = DA.traj{rt}(2,end)- DA.traj{rt}(2,1);
dzz = DA.traj{rt}(3,end)-DA.traj{rt}(3,1);

dxx = 50 * dxx;
dyy = 50 * dyy;
dzz = 50 * dzz; 
pltArrow = quiver3(DA.traj{rt}(1,1), DA.traj{rt}(2,1), DA.traj{rt}(3,1),...
    dxx, dyy, dzz, 'ok','filled', 'LineWidth', 1.5, 'MaxHeadSize',100,'AutoScaleFactor', 2,...
    'Alignment','tail', 'MarkerSize', 12, 'MarkerFaceColor','w','ShowArrowHead','on');

% end



% IFDS Path, if available
if ~isempty(DA.Paths{rt})
    pltPath = PlotPath(rt, DA.Paths, DA.Param.Xini, DA.Param.Yini, DA.Param.Zini, DA.destin, DA.Param.multiTarget);
end

% Trail of the UAV trajectory
if rt>1
    prevTraj = [DA.traj{1:rt-1}];
    pltTraj = plot3(prevTraj(1,:), prevTraj(2,:), prevTraj(3,:), 'k', 'LineWidth', 1.2); 
end


% if (DA.Param.scene ~= 41 || DA.Param.scene ~= 42) && rt==1
    % Obstacle
    [Gamma, Gamma_star] = PlotObject(DA.Object, DA.Param.Rg, rt, rt, XX, YY, ZZ, Gamma, Gamma_star);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
%     camlight
% end

% Constraint Matrix
% imagesc(0:200, -100:100, weatherMat(:,:,i), 'AlphaData',1)



set(gca, 'LineWidth', 2, 'FontSize', DA.Param.fontSize-8)
% hold off
% colormap gray
clim([0 1])


% Constraint Matrix
if DA.Param.k~=0
    hold on
    set(gca, 'YDir', 'normal')
    colormap turbo
    contourf(1:200,-100:99,DA.weatherMatMod(:,:,rt), 30)
    [C2,h2] = contourf(1:200, -100:99, DA.weatherMat(:,:,rt), [DA.Param.B_U, DA.Param.B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);
%         contourf(1:200,-100:99,DA.weatherMatMod(:,:,15), 30, 'FaceAlpha', 1)
%         [C2,h2] = contourf(1:200, -100:99, DA.weatherMat(:,:,15), [DA.Param.B_U, DA.Param.B_U], 'FaceAlpha',0,'LineColor', 'w', 'LineWidth', 2);

    clabel(C2,h2,'FontSize',15,'Color','w')
%     colorbar
end

%% Functions
function [Gamma, Gamma_star] = PlotObject(Object, Rg, i, N, X, Y, Z, Gamma, Gamma_star)
    for j = 1:size(Object,2)
        x0 = Object(j).origin(i, 1);
        y0 = Object(j).origin(i, 2);
        z0 = Object(j).origin(i, 3);
        a = Object(j).a;
        b = Object(j).b;
        c = Object(j).c;
        p = Object(j).p;
        q = Object(j).q;
        r = Object(j).r;

        Rstar = Object(j).Rstar;
    
        Gamma(X, Y, Z) = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        Gamma_star(X, Y, Z) = Gamma - ( (Rstar + Rg)/Rstar )^2 + 1;

%         if N > 1
%             fimplicit3(Gamma == 1,'EdgeColor','k','FaceAlpha',1,'MeshDensity',20), hold on
%             fimplicit3(Gamma_star == 1, 'EdgeColor','k','FaceAlpha',0,'MeshDensity',20)
%         else
            fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',1,'MeshDensity',80, 'FaceColor', 'w'), hold on
            fimplicit3(Gamma_star == 1, 'EdgeColor','none','FaceAlpha',0.2,'MeshDensity',30, 'FaceColor', 'w')
%         end

        xlim([0 200])
        ylim([-100 100])
        zlim([0 100])
    end

end

function pltPath = PlotPath(i, Paths, Xini, Yini, Zini, destin, multiTarget)
    if multiTarget
        plot3(Paths{1,i}(1,:), Paths{1,i}(2,:), Paths{1,i}(3,:),'b', 'LineWidth', 1.5)
        hold on, grid on, grid minor, axis equal
        plot3(Paths{2,i}(1,:), Paths{2,i}(2,:), Paths{2,i}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{3,i}(1,:), Paths{3,i}(2,:), Paths{3,i}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{4,i}(1,:), Paths{4,i}(2,:), Paths{4,i}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{5,i}(1,:), Paths{5,i}(2,:), Paths{5,i}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{6,i}(1,:), Paths{6,i}(2,:), Paths{6,i}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{7,i}(1,:), Paths{7,i}(2,:), Paths{7,i}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{8,i}(1,:), Paths{8,i}(2,:), Paths{8,i}(3,:),'b', 'LineWidth', 1.5)
        plot3(Paths{9,i}(1,:), Paths{9,i}(2,:), Paths{9,i}(3,:),'b', 'LineWidth', 1.5)
        scatter3(Xini, Yini, Zini, 'filled', 'r')
        scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(2,1),destin(2,2),destin(2,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(3,1),destin(3,2),destin(3,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(4,1),destin(4,2),destin(4,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(5,1),destin(5,2),destin(5,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(6,1),destin(6,2),destin(6,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(7,1),destin(7,2),destin(7,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(8,1),destin(8,2),destin(8,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
        scatter3(destin(9,1),destin(9,2),destin(9,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
    else
        pltPath = plot3(Paths{1,i}(1,:), Paths{1,i}(2,:), Paths{1,i}(3,:),'b--', 'LineWidth', 1.8);
        hold on
%         axis equal, grid on, grid minor
        scatter3(Xini, Yini, Zini, 'filled', 'r', 'xr', 'sizedata', 150)
        scatter3(destin(1,1),destin(1,2),destin(1,3), 'xr', 'xr', 'sizedata', 150, 'LineWidth', 1.5)
    end

    xlim([0 200])
    ylim([-100 100])
    zlim([0 100])
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
%     hold off
end