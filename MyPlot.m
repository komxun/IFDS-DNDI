function MyPlot(swarm, tRange, comsHistory, option)
    zoomIn = option.zoomIn;
    wantLegend = option.legend;

    syms XX YY ZZ Gamma(XX,YY,ZZ) Gamma_star(XX,YY,ZZ)
    colorList = ["#A2142F", "#4DBEEE", "#77AC30", "#7E2F8E","#EDB120"];
    agentPlot = [];
    for rt = tRange
        % ======== Plot leader UAV =======================
        agentPlot(1) = plot3(swarm{1}.trajectory(1,rt), swarm{1}.trajectory(2,rt), swarm{1}.trajectory(3,rt), 'ok-', 'MarkerSize', 10, 'LineWidth', 1.5);
        hold on, grid on
        % ======== Plot leader trajectory ================
        plot3(swarm{1}.trajectory(1,1:rt), swarm{1}.trajectory(2,1:rt), swarm{1}.trajectory(3,1:rt), 'k-','LineWidth', 1.5)
        
        % Leader's path
        plot3(swarm{1}.DA.Paths{rt}(1,:), swarm{1}.DA.Paths{rt}(2,:), swarm{1}.DA.Paths{rt}(3,:), 'b--' ,'LineWidth', 1.8)
        [Gamma, Gamma_star] = PlotObject(swarm{1}.DA.Object, swarm{1}.DA.Param.Rg, rt, rt, XX, YY, ZZ, Gamma, Gamma_star);
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
        camlight
        axis equal
        
        % ======== Plot folower's trajectory ================
        rt2 = ceil(rt/10);
        for j = 2:length(swarm)
            
            % ======== Plot follower UAV =======================
            agentPlot(j) = plot3(swarm{j}.trajectory(1,rt), swarm{j}.trajectory(2,rt), swarm{j}.trajectory(3,rt), '->' ,'LineWidth', 1.5, 'Color', colorList(j-1));
            % ======== Plot follower's trajectory =======================
            plot3(swarm{j}.trajectory(1,1:rt), swarm{j}.trajectory(2,1:rt), swarm{j}.trajectory(3,1:rt), '-','LineWidth', 1.5, 'Color', colorList(j-1))
            destin = swarm{j}.GetItsDestination();
            if (rt<=length(swarm{j}.DA.Paths)) && ~isempty(swarm{j}.DA.Paths{rt})
                plot3(swarm{j}.DA.Paths{rt}(1,:), swarm{j}.DA.Paths{rt}(2,:), swarm{j}.DA.Paths{rt}(3,:), '--' , 'LineWidth', 2, 'Color', colorList(j-1))
                scatter3(destin(1), destin(2), destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
            end
            
            % ++++++++ Plot Coms Link +++++++++++
            if comsHistory(rt,j) == 1 && comsHistory(rt,1) == 1
                % Your two points
                P1 = [swarm{1}.trajectory(:,rt)]';
                P2 = [swarm{j}.trajectory(:,rt)]';
                
                % Their vertial concatenation is what you want
                pts = [P1; P2];
                
                % Because that's what line() wants to see    
                comsLine = line(pts(:,1), pts(:,2), pts(:,3));
                comsLine.Color = "green";
                comsLine.LineWidth = 1.5;
            end
            if zoomIn
                xlim([swarm{1}.trajectory(1,tRange)-35, swarm{1}.trajectory(1,tRange)+35])
                ylim([swarm{1}.trajectory(2,tRange)-30, swarm{1}.trajectory(2,tRange)+30])
            end
            zlim([0 100])
            xlabel("X [m]", 'FontSize', 20)
            ylabel("Y [m]", 'FontSize', 20)
            set(gca, 'FontSize', 20, 'LineWidth', 1)
        end

    end
    if wantLegend
        legend(agentPlot, "Leader", "Agent1", "Agent2", "Agent3", "Agent4", "Agent5", 'Orientation', 'horizontal',...
            'Location', [0.12507287289916 0.0723520069288918 0.448046875 0.0356394129979036])
    end
end

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
            % fimplicit3(Gamma == 1,'EdgeColor','k','FaceAlpha',1,'MeshDensity',20), hold on
%             fimplicit3(Gamma_star == 1, 'EdgeColor','k','FaceAlpha',0,'MeshDensity',20)
%         else
            fimplicit3(Gamma == 1,'EdgeColor','none','FaceAlpha',0.4,'MeshDensity',80, 'FaceColor', 'w'), hold on
%             fimplicit3(Gamma_star == 1, 'EdgeColor','none','FaceAlpha',0.2,'MeshDensity',30, 'FaceColor', 'w')
%         end

%         xlim([0 260])
%         ylim([-100 100])
%         zlim([0 100])
    end

end