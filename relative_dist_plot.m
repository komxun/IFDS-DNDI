if animation
    fg2 = figure(109);
    fg2.WindowState='maximized';
    % 
    % [Gamma, Gamma_star] = PlotObject(leader.DA.Object, leader.DA.Param.Rg, rt, rt, XX, YY, ZZ, Gamma, Gamma_star);
    %         xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
        % camlight
    for rt = 1:2:length(leadTraj)-1
        % subplot(3,1,1:2)
        % subplot(5,5,[1,2,3,6,7,8,11,12,13,16,17,18,21,22,23])
        subplot(5,5,[1,2,3,6,7,8,11,12,13])
        plot3(leadTraj(1,rt), leadTraj(2,rt), leadTraj(3,rt), 'ok-', 'MarkerSize', 10, 'LineWidth', 1.5), hold on
        plot3(a1Traj(1,rt), a1Traj(2,rt), a1Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(1)), hold on
        plot3(a2Traj(1,rt), a2Traj(2,rt), a2Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(2))
        plot3(a3Traj(1,rt), a3Traj(2,rt), a3Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(3))
        plot3(a4Traj(1,rt), a4Traj(2,rt), a4Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(4))
        plot3(a5Traj(1,rt), a5Traj(2,rt), a5Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(5))
    
        plot3(leadTraj(1,1:rt), leadTraj(2,1:rt), leadTraj(3,1:rt), 'k-', 'LineWidth', 1.5),
        plot3(a1Traj(1,1:rt), a1Traj(2,1:rt), a1Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(1))
        plot3(a2Traj(1,1:rt), a2Traj(2,1:rt), a2Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(2))
        plot3(a3Traj(1,1:rt), a3Traj(2,1:rt), a3Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(3))
        plot3(a4Traj(1,1:rt), a4Traj(2,1:rt), a4Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(4))
        plot3(a5Traj(1,1:rt), a5Traj(2,1:rt), a5Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(5))

        legend("Leader", "", "", "", "", "", "", ...
            "Agent 1", "Agent 2", "Agent 3", "Agent 4", "Agent 5", ...
            "AutoUpdate", "off",...
            "Position", [0.54, 0.71,0.05, 0.2], 'FontSize', 14)

        %IFDS Path
        rt2 = ceil(rt/10);
        plot3(leader.DA.Paths{rt}(1,:), leader.DA.Paths{rt}(2,:), leader.DA.Paths{rt}(3,:), 'b--' ,'LineWidth', 1.8)
        if (rt<=length(agent1.DA.Paths)) && ~isempty(agent1.DA.Paths{rt})
            plot3(agent1.DA.Paths{rt}(1,:), agent1.DA.Paths{rt}(2,:), agent1.DA.Paths{rt}(3,:), '--' , 'LineWidth', 1.2, 'Color', colorList(1))
            scatter3(agent1.DA.destin(1), agent1.DA.destin(2), agent1.DA.destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end
        if (rt<=length(agent2.DA.Paths)) && ~isempty(agent2.DA.Paths{rt})
            plot3(agent2.DA.Paths{rt}(1,:), agent2.DA.Paths{rt}(2,:), agent2.DA.Paths{rt}(3,:), '--' ,'LineWidth', 1.2, 'Color', colorList(2))
            scatter3(agent2.DA.destin(1), agent2.DA.destin(2), agent2.DA.destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end
        if (rt<=length(agent3.DA.Paths)) && ~isempty(agent3.DA.Paths{rt})
            plot3(agent3.DA.Paths{rt}(1,:), agent3.DA.Paths{rt}(2,:), agent3.DA.Paths{rt}(3,:), '--' ,'LineWidth', 1.2, 'Color', colorList(3))
            scatter3(agent3.DA.destin(1), agent3.DA.destin(2), agent3.DA.destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end
        if (rt<=length(agent4.DA.Paths)) && ~isempty(agent4.DA.Paths{rt})
            plot3(agent4.DA.Paths{rt}(1,:), agent4.DA.Paths{rt}(2,:), agent4.DA.Paths{rt}(3,:), '--' ,'LineWidth', 1.2, 'Color', colorList(4))
            scatter3(agent4.DA.destin(1), agent4.DA.destin(2), agent4.DA.destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end
        if (rt<=length(agent5.DA.Paths)) && ~isempty(agent5.DA.Paths{rt})
            plot3(agent5.DA.Paths{rt}(1,:), agent5.DA.Paths{rt}(2,:), agent5.DA.Paths{rt}(3,:), '--' ,'LineWidth', 1.2, 'Color', colorList(5))
            scatter3(agent5.DA.destin(1), agent5.DA.destin(2), agent5.DA.destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end

        axis equal
        grid on, grid minor
        xlim([0 280])
        ylim([-100 100])
        zlim([0 100])
        title(num2str(rt/100,'time = %4.2f s'), 'FontSize', 16)
    
        xlabel("X [m]", 'FontSize', 16)
        ylabel("Y [m]", 'FontSize', 16)
        zlabel("Z [m]", 'FontSize', 16)
    
        [Gamma, Gamma_star] = PlotObject(leader.DA.Object, leader.DA.Param.Rg, rt, rt, XX, YY, ZZ, Gamma, Gamma_star);
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
    
        [Gamma, Gamma_star] = PlotObject(leader.DA.Object, leader.DA.Param.Rg, length(leadTraj)-1, rt, XX, YY, ZZ, Gamma, Gamma_star);
        xlabel('X [m]', 'FontSize', 16); ylabel('Y [m]', 'FontSize', 16); zlabel('Z [m]', 'FontSize', 16); 
        camlight
        hold off
        
        drawnow

        
        subplot(5,5,[16,17,18,21,22,23])
        plot3(leadTraj(1,rt), leadTraj(2,rt), leadTraj(3,rt), 'ok-', 'MarkerSize', 10, 'LineWidth', 1.5), hold on
        plot3(a1Traj(1,rt), a1Traj(2,rt), a1Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(1)), hold on
        plot3(a2Traj(1,rt), a2Traj(2,rt), a2Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(2))
        plot3(a3Traj(1,rt), a3Traj(2,rt), a3Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(3))
        plot3(a4Traj(1,rt), a4Traj(2,rt), a4Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(4))
        plot3(a5Traj(1,rt), a5Traj(2,rt), a5Traj(3,rt), '>' ,'LineWidth', 1.5, 'Color', colorList(5))
    
        plot3(leadTraj(1,1:rt), leadTraj(2,1:rt), leadTraj(3,1:rt), 'k-', 'LineWidth', 1.5),
        plot3(a1Traj(1,1:rt), a1Traj(2,1:rt), a1Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(1))
        plot3(a2Traj(1,1:rt), a2Traj(2,1:rt), a2Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(2))
        plot3(a3Traj(1,1:rt), a3Traj(2,1:rt), a3Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(3))
        plot3(a4Traj(1,1:rt), a4Traj(2,1:rt), a4Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(4))
        plot3(a5Traj(1,1:rt), a5Traj(2,1:rt), a5Traj(3,1:rt), '-' ,'LineWidth', 1.5, 'Color', colorList(5))
        
        xlabel("X [m]", 'FontSize', 16)
        ylabel("Y [m]", 'FontSize', 16)
        zlabel("Z [m]", 'FontSize', 16)
        grid on, grid minor
        hold off
        
        drawnow

        subplot(5,5,4:5)
        plot(r12(1:rt),'LineWidth', 1.2, 'Color', colorList(2)), hold on
        plot(r13(1:rt),'LineWidth', 1.2, 'Color', colorList(3))
        plot(r14(1:rt), 'LineWidth', 1.2, 'Color', colorList(4))
        plot(r15(1:rt), 'LineWidth', 1.2, 'Color', colorList(5))
        yline(gcs.rIPN, 'r--', 'LineWidth', 1.5)
        title("Agent 1 Relative distance", 'Color', colorList(1), 'FontSize', 16)
        grid on, grid minor
        ylabel("[m]", 'FontSize', 16)
        hold off
        

        subplot(5,5,9:10)
        plot(r12(1:rt),'LineWidth', 1.2, 'Color', colorList(1)), hold on
        plot(r23(1:rt),'LineWidth', 1.2, 'Color', colorList(3))
        plot(r24(1:rt),'LineWidth', 1.2, 'Color', colorList(4))
        plot(r25(1:rt),'LineWidth', 1.2, 'Color', colorList(5))
        yline(gcs.rIPN, 'r--', 'LineWidth', 1.5)
        title("Agent 2 Relative distance", 'Color', colorList(2), 'FontSize', 16)
        grid on, grid minor
        ylabel("[m]", 'FontSize', 16)
        hold off
        

        subplot(5,5,14:15)
        plot(r13(1:rt),'LineWidth', 1.2, 'Color', colorList(1)), hold on
        plot(r23(1:rt),'LineWidth', 1.2, 'Color', colorList(2))
        plot(r34(1:rt),'LineWidth', 1.2, 'Color', colorList(4))
        plot(r35(1:rt),'LineWidth', 1.2, 'Color', colorList(5))
        yline(gcs.rIPN, 'r--', 'LineWidth', 1.5)
        title("Agent 3 Relative distance", 'Color', colorList(3), 'FontSize', 16)
        grid on, grid minor
        ylabel("[m]", 'FontSize', 16)
        hold off
        

        subplot(5,5,19:20)
        plot(r14(1:rt), 'LineWidth', 1.2, 'Color', colorList(1)), hold on
        plot(r24(1:rt),'LineWidth', 1.2, 'Color', colorList(2))
        plot(r34(1:rt),'LineWidth', 1.2, 'Color', colorList(3))
        plot(r45(1:rt),'LineWidth', 1.2, 'Color', colorList(5))
        yline(gcs.rIPN, 'r--', 'LineWidth', 1.5)
        grid on, grid minor
        title("Agent 4 Relative distance", 'Color', colorList(4), 'FontSize', 16)
        ylabel("[m]", 'FontSize', 16)
        hold off
        drawnow
        

        subplot(5,5,24:25)
        plot(r15(1:rt), 'LineWidth', 1.2, 'Color', colorList(1)), hold on
        plot(r25(1:rt),'LineWidth', 1.2, 'Color', colorList(2))
        plot(r35(1:rt),'LineWidth', 1.2, 'Color', colorList(3))
        plot(r45(1:rt),'LineWidth', 1.2, 'Color', colorList(4))
        yline(gcs.rIPN, 'r--', 'LineWidth', 1.5)
        grid on, grid minor
        title("Agent 5 Relative distance", 'Color', colorList(5), 'FontSize', 16)
        ylabel("[m]", 'FontSize', 16)
        xlabel("Time [ms]", 'FontSize', 16)
        hold off
        drawnow
        sgtitle("Relative distance between UAVs VS Time, Minimum allowed distance R_{min}= " + num2str(gcs.rIPN) + " m", 'FontSize', 20)
        
        
        if saveVid
            frm(rt) = getframe(gcf) ;
        end
        
    end
    
    if saveVid
    
            video_name = "IPN_dist_test.avi";
            disp("Video saved: " + video_name);
            % create the video writer with 1 fps
            writerObj = VideoWriter(video_name);
        %     writerObj.FrameRate = 30;
            writerObj.FrameRate = 60;
            % set the seconds per image
            % open the video writer
            open(writerObj);
            % write the frames to the video
            for j=1:2:length(frm)
                % convert the image to a frame
                j
                frame = frm(j) ;    
                writeVideo(writerObj, frame);
            end
            % close the writer object
            close(writerObj);
    end
end