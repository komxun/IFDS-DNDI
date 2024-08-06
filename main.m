% .-------------------------------------------------------------.
% | Dynamic Autorouting Program for Multi-Agent Systems         |
% | created by Komsun Tamanakijprasart and Dr.Sabyasachi Mondal |
% '-------------------------------------------------------------'

clc, clear, close all

animation = 1;
saveVid = 0;
% UAV's Initial State
psi_i = 0;          % [rad] Initial Yaw angle
gamma_i = 0;        % [rad] Initial Pitch angle

formation = 7;  % 1) Diagonal line 2) Star 3) Triangle 
                % 4) Twisting star 5) Large twisting star
                % 6) Horizontal line 7) V-shape

% Initialize GCS
gcs = GCS;
no_uav = 5; % fixed for this version

% Pre-allocation
Xnei_states = zeros(3,no_uav);
pos2Fol = zeros(3,no_uav);

% Create UAVs
leader = BasicUAV;
leader.SetName("Leader");

XX = leader.GetCurrentPos();

% Setting Initial states for followers
for ii = 1:1:no_uav 
    Xnei_states(:,ii) = [XX(1)-(-1)^ii*ii*1 ; 
                         XX(2)+(-1)^(ii+1)*ii*0.5; 
                         XX(3)+(-1)^(ii+1)*ii*1];
end

% Initialize followers
agent1 = FollowerUAV(Xnei_states(1,1), Xnei_states(2,1), Xnei_states(3,1), 0, 0);
agent2 = FollowerUAV(Xnei_states(1,2), Xnei_states(2,2), Xnei_states(3,2), 0, 0);
agent3 = FollowerUAV(Xnei_states(1,3), Xnei_states(2,3), Xnei_states(3,3), 0, 0);
agent4 = FollowerUAV(Xnei_states(1,4), Xnei_states(2,4), Xnei_states(3,4), 0, 0);
agent5 = FollowerUAV(Xnei_states(1,5), Xnei_states(2,5), Xnei_states(3,5), 0, 0);
agent1.SetName("Agent1");
agent2.SetName("Agent2");
agent3.SetName("Agent3");
agent4.SetName("Agent4");
agent5.SetName("Agent5");

gcs.numUAV = 5;

% Load the configuration
conf = Config();

% Initialization
leader.Initialize(conf)
agent1.Initialize(conf)
agent2.Initialize(conf)
agent3.Initialize(conf)
agent4.Initialize(conf)
agent5.Initialize(conf)


% Set different destination
followerDestin = gcs.CreateFormation(leader.GetItsDestination, no_uav, leader.GetCurrentAngle,formation);
ds1 = followerDestin(:,1)';
ds2 = followerDestin(:,2)';
ds3 = followerDestin(:,3)';
ds4 = followerDestin(:,4)';
ds5 = followerDestin(:,5)';

ds1 = leader.GetItsDestination + [0, 0, 5];
ds2 = leader.GetItsDestination + [0, 0, 10];
ds3 = leader.GetItsDestination + [0, 0, 15];
ds4 = leader.GetItsDestination + [0, 0, 20];
ds5 = leader.GetItsDestination + [0, 0, 25];

agent1.SetItsDestination(ds1(1), ds1(2), ds1(3))
agent2.SetItsDestination(ds2(1), ds2(2), ds2(3))
agent3.SetItsDestination(ds3(1), ds3(2), ds3(3))
agent4.SetItsDestination(ds4(1), ds4(2), ds4(3))
agent5.SetItsDestination(ds5(1), ds5(2), ds5(3))

X_nei = [agent1.GetCurrentState();
         agent2.GetCurrentState();
         agent3.GetCurrentState();
         agent4.GetCurrentState();
         agent5.GetCurrentState()];

swarm = {leader, agent1, agent2, agent3, agent4, agent5};

refTrajs = cell(1,no_uav);

%% Main Loop
leader.SetFlagComs(1)
rt = 0;
while true
    rt = rt+1;
    % Synchronize time step
    gcs.SetTimeStep(rt)

    % Propagate Leader with DA
    leader.UpdateBasicUAV()

    % Check if leader has reached the destination or not
    if leader.GetFlagDestin == 1 
        disp("* Leader has reached the destination!")
        break
    end

    % Get Leader's current state
    XL_states = leader.GetCurrentState();
    XL = leader.GetCurrentPos();

    % Create formation from leader's current position
    pos2Fol = gcs.CreateFormation(XL, no_uav, leader.GetCurrentAngle, formation);
    
    for k = 1:no_uav
        refTrajs{k} = [refTrajs{k}; pos2Fol(:,k)'];
    end

    % Compute Consensus-Formation-Following for followers
    X_nei = consensus_func(X_nei, pos2Fol, XL_states, no_uav, swarm);

    % Check for IPN for collision avoidance among followerUAVs
    [ipnIdx, closestIdx] = gcs.CheckIPN([XL; X_nei(:,1:3)]);
    selectedUav = swarm(ipnIdx);  
    closestUav = closestIdx(ipnIdx);

    % Manually feed the UAVs information to the IPN-activated UAVs
    if ~isempty(selectedUav)
        for j = 1:length(selectedUav)
            cuav = swarm{closestUav(j)};
            selectedUav{j}.flagIPN = 1;
            selectedUav{j}.gcsData = [cuav.GetCurrentPos,...
                cuav.GetCurrentAngle, cuav.cruisingSpeed];   
        end
    end

    % ----------------------- Manual Logic --------------------------------
    % if i >= 1300
    %     leader.SetFlagDanger(0)
    % elseif i >= 600
    %     leader.SetFlagDanger(1)
    % end
    % 
    % if i >= 2100
    %     leader.SetFlagDanger(0)
    % elseif i >= 1800
    %     leader.SetFlagDanger(1)
    % end

    %==================================
    if rt >= 1300
        leader.SetFlagComs(1)
    elseif rt >= 600
        leader.SetFlagComs(0)
    end
    
    if rt == 500
        agent1.SetFlagComs(0)
        agent3.SetFlagComs(0)
    end
    
    if rt == 1500
        agent1.SetFlagComs(1)
    end



    % Update FollowerUAV states
    agent1.UpdateFollower(X_nei(1,:), leader.DA.Object, leader.GetFlagDanger, leader.GetFlagComs);
    agent2.UpdateFollower(X_nei(2,:), leader.DA.Object, leader.GetFlagDanger, leader.GetFlagComs);
    agent3.UpdateFollower(X_nei(3,:), leader.DA.Object, leader.GetFlagDanger, leader.GetFlagComs);
    agent4.UpdateFollower(X_nei(4,:), leader.DA.Object, leader.GetFlagDanger, leader.GetFlagComs);
    agent5.UpdateFollower(X_nei(5,:), leader.DA.Object, leader.GetFlagDanger, leader.GetFlagComs);

    X_nei = [agent1.GetCurrentState();
             agent2.GetCurrentState();
             agent3.GetCurrentState();
             agent4.GetCurrentState();
             agent5.GetCurrentState()];
end

%% Plotting
colorList = ["#A2142F", "#4DBEEE", "#77AC30", "#7E2F8E","#EDB120"];
leadTraj = leader.trajectory;
a1Traj = agent1.trajectory;
a2Traj = agent2.trajectory;
a3Traj = agent3.trajectory;
a4Traj = agent4.trajectory;
a5Traj = agent5.trajectory;
a1destin = agent1.GetItsDestination();
a2destin = agent2.GetItsDestination();
a3destin = agent3.GetItsDestination();
a4destin = agent4.GetItsDestination();
a5destin = agent5.GetItsDestination();


rt = length(leadTraj)-1;
syms XX YY ZZ Gamma(XX,YY,ZZ) Gamma_star(XX,YY,ZZ) 
figure
plot3(leadTraj(1,:), leadTraj(2,:), leadTraj(3,:), 'k-', 'LineWidth', 1.5), hold on
plot3(a1Traj(1,:), a1Traj(2,:), a1Traj(3,:), 'LineWidth', 1.5, 'Color', colorList(1))
plot3(a2Traj(1,:), a2Traj(2,:), a2Traj(3,:), 'LineWidth', 1.5, 'Color', colorList(2))
plot3(a3Traj(1,:), a3Traj(2,:), a3Traj(3,:), 'LineWidth', 1.5, 'Color', colorList(3))
plot3(a4Traj(1,:), a4Traj(2,:), a4Traj(3,:), 'LineWidth', 1.5, 'Color', colorList(4))
plot3(a5Traj(1,:), a5Traj(2,:), a5Traj(3,:), 'LineWidth', 1.5, 'Color', colorList(5))
[Gamma, Gamma_star] = PlotObject(leader.DA.Object, leader.DA.Param.Rg, rt, rt, XX, YY, ZZ, Gamma, Gamma_star);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 

camlight

axis equal
grid on, grid minor
xlim([0 280])
ylim([-100 100])
zlim([0 100])

%% % Animation plot
if animation
    fg = figure(101);
    fg.WindowState='maximized';
    
    % [Gamma, Gamma_star] = PlotObject(leader.DA.Object, leader.DA.Param.Rg, rt, rt, XX, YY, ZZ, Gamma, Gamma_star);
    %         xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
    %     camlight
    for rt = 1:10:length(leadTraj)-1
        
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

        %IFDS Path
        rt2 = ceil(rt/10);
        plot3(leader.DA.Paths{rt}(1,:), leader.DA.Paths{rt}(2,:), leader.DA.Paths{rt}(3,:), 'b--' ,'LineWidth', 1.8)
        if (rt<=length(agent1.DA.Paths)) && ~isempty(agent1.DA.Paths{rt})
            plot3(agent1.DA.Paths{rt}(1,:), agent1.DA.Paths{rt}(2,:), agent1.DA.Paths{rt}(3,:), '--' , 'LineWidth', 1.2, 'Color', colorList(1))
            scatter3(a1destin(1), a1destin(2), a1destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end
        if (rt<=length(agent2.DA.Paths)) && ~isempty(agent2.DA.Paths{rt})
            plot3(agent2.DA.Paths{rt}(1,:), agent2.DA.Paths{rt}(2,:), agent2.DA.Paths{rt}(3,:), '--' ,'LineWidth', 1.2, 'Color', colorList(2))
            scatter3(a2destin(1), a2destin(2), a2destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end
        if (rt<=length(agent3.DA.Paths)) && ~isempty(agent3.DA.Paths{rt})
            plot3(agent3.DA.Paths{rt}(1,:), agent3.DA.Paths{rt}(2,:), agent3.DA.Paths{rt}(3,:), '--' ,'LineWidth', 1.2, 'Color', colorList(3))
            scatter3(a3destin(1), a3destin(2), a3destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end
        if (rt<=length(agent4.DA.Paths)) && ~isempty(agent4.DA.Paths{rt})
            plot3(agent4.DA.Paths{rt}(1,:), agent4.DA.Paths{rt}(2,:), agent4.DA.Paths{rt}(3,:), '--' ,'LineWidth', 1.2, 'Color', colorList(4))
            scatter3(a4destin(1), a4destin(2), a4destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end
        if (rt<=length(agent5.DA.Paths)) && ~isempty(agent5.DA.Paths{rt})
            plot3(agent5.DA.Paths{rt}(1,:), agent5.DA.Paths{rt}(2,:), agent5.DA.Paths{rt}(3,:), '--' ,'LineWidth', 1.2, 'Color', colorList(5))
            scatter3(a5destin(1), a5destin(2), a5destin(3),'xr', 'sizedata', 150, 'LineWidth', 1.5)
        end

        axis equal
        grid on, grid minor
%         xlim([0 280])
%         ylim([-100 100])
%         zlim([0 100])
%         title(num2str(rt/100,'time = %4.2f s'), 'FontSize', 24)
    
        xlabel("X [m]",'FontSize', 24)
        ylabel("Y [m]",'FontSize', 24)
        zlabel("Z [m]", 'FontSize', 24)
    
        % [Gamma, Gamma_star] = PlotObject(leader.DA.Object, leader.DA.Param.Rg, rt, rt, XX, YY, ZZ, Gamma, Gamma_star);
        %     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); 
    
        % [Gamma, Gamma_star] = PlotObject(leader.DA.Object, leader.DA.Param.Rg, length(leadTraj)-1, rt, XX, YY, ZZ, Gamma, Gamma_star);
        % camlight
        % 
        % legend("Leader", "", "", "", "", "", "", ...
        %     "Agent 1", "Agent 2", "Agent 3", "Agent 4", "Agent 5", ...
        %     "AutoUpdate", "off",...
        %     "Position", [0.92, 0.71,0.05, 0.2], 'FontSize', 14)
        set(gca,'FontSize', 24)
    
        hold off
        drawnow
    
        if saveVid
            frm(rt) = getframe(gcf) ;
        end
        
    end
    
    if saveVid
        video_name = "case2-1_2.avi";
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
            frame = frm(j) ;    
            writeVideo(writerObj, frame);
        end
        % close the writer object
        close(writerObj);
    end
end

%% Extra Plot

r12 = zeros(1,length(a1Traj));
r13 = zeros(1,length(a1Traj));
r14 = zeros(1,length(a1Traj));
r15 = zeros(1,length(a1Traj));

r23 = zeros(1,length(a1Traj));
r24 = zeros(1,length(a1Traj));
r25 = zeros(1,length(a1Traj));

r34 = zeros(1,length(a1Traj));
r35 = zeros(1,length(a1Traj));

r45 = zeros(1,length(a1Traj));

for j = 1:min([length(a1Traj), length(a2Traj), length(a3Traj), length(a4Traj), length(a5Traj)])
    r12(j) = norm(a1Traj(:,j) - a2Traj(:,j));
    r13(j) = norm(a1Traj(:,j) - a3Traj(:,j));
    r14(j) = norm(a1Traj(:,j) - a4Traj(:,j));
    r15(j) = norm(a1Traj(:,j) - a5Traj(:,j));

    r23(j) = norm(a2Traj(:,j) - a3Traj(:,j));
    r24(j) = norm(a2Traj(:,j) - a4Traj(:,j));
    r25(j) = norm(a2Traj(:,j) - a5Traj(:,j));

    r34(j) = norm(a3Traj(:,j) - a4Traj(:,j));
    r35(j) = norm(a3Traj(:,j) - a5Traj(:,j));         

    r45(j) = norm(a4Traj(:,j) - a5Traj(:,j));

end

yo = linspace(0,30,length(r12));
figure(99)
plot(yo, r12, 'LineWidth', 1.2), hold on
plot(yo, r13, 'LineWidth', 1.2)
plot(yo, r14, 'LineWidth', 1.2)
plot(yo, r15, 'LineWidth', 1.2)
plot(yo, r23,'LineWidth', 1.2)
plot(yo, r24, '-', 'LineWidth', 1.2)
plot(yo, r25, '-', 'LineWidth', 1.2)
plot(yo, r34, '-', 'LineWidth', 1.2)
plot(yo, r35, '-', 'LineWidth', 1.2)
plot(yo, r45, '-', 'LineWidth', 1.2)
yline(gcs.rIPN, 'r--', 'LineWidth', 1.5)

grid on, grid minor
legend("r12", "r13", "r14", "r15", "r23", "r24", "r25", ...
    "r34", "r35", "r45", "R_{min}")
set(gca, "FontSize", 22, 'LineWidth', 1.2)
xlabel("Time (s)")
ylabel("Relative distance (m)")
title("Relative distance between UAVs VS Time")
subtitle("Minimum allowed distance R_{min}= " + num2str(gcs.rIPN) + " m")

%% Plot Actual trajectories 
figure()
subplot(3,5,[1 2 3; 6 7 8; 11 12 13])
plot3(swarm{1}.trajectory(1,:), swarm{1}.trajectory(2,:), swarm{1}.trajectory(3,:), 'k','LineWidth', 1.5)
hold on, grid on, grid minor
axis equal
set(gca, 'FontSize', 20)
xlabel("X [m]", 'FontSize', 20)
ylabel("Y [m]", 'FontSize', 20)
zlabel("Z [m]", 'FontSize', 20)

for j = 1:length(swarm)
    

    subplot(3,5,[1 2 3; 6 7 8; 11 12 13])
    % plot3(refTrajs{j2}(:,1)', refTrajs{j2}(:,2)',  refTrajs{j2}(:,3)','-','LineWidth', 1.5)
    if j>1
        plot3(swarm{j}.trajectory(1,:), swarm{j}.trajectory(2,:), swarm{j}.trajectory(3,:), '-','LineWidth', 1.5, 'Color', colorList(j-1))
    end

    subplot(3,5,[4 5])
    plot(swarm{1}.trajectory(1,:), swarm{1}.trajectory(2,:), 'k','LineWidth', 1.5)
    hold on
    if j>1
        plot(swarm{j}.trajectory(1,:), swarm{j}.trajectory(2,:), '-','LineWidth', 1.5, 'Color', colorList(j-1))
    end
    % plot(refTrajs{j2}(:,1)', refTrajs{j2}(:,2)', '-','LineWidth', 1.5)
    xlabel("X [m]", 'FontSize', 20)
    ylabel("Y [m]", 'FontSize', 20)
    set(gca, 'FontSize', 20)

    subplot(3,5,[9 10])
    plot(swarm{1}.trajectory(2,:), swarm{1}.trajectory(3,:), 'k','LineWidth', 1.5)
    hold on
    if j>1
        plot(swarm{j}.trajectory(2,:), swarm{j}.trajectory(3,:), '-','LineWidth', 1.5, 'Color', colorList(j-1))
    end
    % plot(refTrajs{j2}(:,2)', refTrajs{j2}(:,3)', '-','LineWidth', 1.5)
    xlabel("Y [m]", 'FontSize', 20)
    ylabel("Z [m]", 'FontSize', 20)
    set(gca, 'FontSize', 20)

    subplot(3,5,[14 15])
    plot(swarm{1}.trajectory(1,:), swarm{1}.trajectory(3,:), 'k','LineWidth', 1.5)
    hold on
    if j>1
        plot(swarm{j}.trajectory(1,:), swarm{j}.trajectory(3,:), '-','LineWidth', 1.5, 'Color', colorList(j-1))
    end
    % plot(refTrajs{j2}(:,1)', refTrajs{j2}(:,3)', '-','LineWidth', 1.5)
    xlabel("X [m]", 'FontSize', 20)
    ylabel("Z [m]", 'FontSize', 20)
    set(gca, 'FontSize', 20)
end
sgtitle("Actual Formation Trajectories", "FontSize", 30)

% Reference plot
figure()
subplot(3,5,[1 2 3; 6 7 8; 11 12 13])
plot3(swarm{1}.trajectory(1,:), swarm{1}.trajectory(2,:), swarm{1}.trajectory(3,:), 'k','LineWidth', 1.5)
hold on, grid on, grid minor
axis equal
set(gca, 'FontSize', 20)
xlabel("X [m]", 'FontSize', 20)
ylabel("Y [m]", 'FontSize', 20)
zlabel("Z [m]", 'FontSize', 20)

for j = 1:length(swarm)

    subplot(3,5,[1 2 3; 6 7 8; 11 12 13])
    if j<=no_uav
        plot3(refTrajs{j}(:,1)', refTrajs{j}(:,2)',  refTrajs{j}(:,3)','-','LineWidth', 1.5, 'Color', colorList(j))
    end

    subplot(3,5,[4 5])
    plot(swarm{1}.trajectory(1,:), swarm{1}.trajectory(2,:), 'k','LineWidth', 1.5)
    hold on
    if j<=no_uav
        plot(refTrajs{j}(:,1)', refTrajs{j}(:,2)', '-','LineWidth', 1.5, 'Color', colorList(j))
    end
    xlabel("X [m]", 'FontSize', 20)
    ylabel("Y [m]", 'FontSize', 20)
    set(gca, 'FontSize', 20)

    subplot(3,5,[9 10])
    plot(swarm{1}.trajectory(2,:), swarm{1}.trajectory(3,:), 'k','LineWidth', 1.5)
    hold on
    if j<=no_uav
        plot(refTrajs{j}(:,2)', refTrajs{j}(:,3)', '-','LineWidth', 1.5, 'Color', colorList(j))
    end
    xlabel("Y [m]", 'FontSize', 20)
    ylabel("Z [m]", 'FontSize', 20)
    set(gca, 'FontSize', 20)

    subplot(3,5,[14 15])
    plot(swarm{1}.trajectory(1,:), swarm{1}.trajectory(3,:), 'k','LineWidth', 1.5)
    hold on
    if j<=no_uav
        plot(refTrajs{j}(:,1)', refTrajs{j}(:,3)', '-','LineWidth', 1.5, 'Color', colorList(j))
    end
    xlabel("X [m]", 'FontSize', 20)
    ylabel("Z [m]", 'FontSize', 20)
    set(gca, 'FontSize', 20)
end
sgtitle("Referenced Formation Trajectories", "FontSize", 30)

%% Plot Agent trajectories with lost coms ( 1 and 4 )

figure()
subplot(3,5,[1 2 3; 6 7 8; 11 12 13])
plot3(swarm{1}.trajectory(1,:), swarm{1}.trajectory(2,:), swarm{1}.trajectory(3,:), 'k','LineWidth', 1.5)
hold on, grid on, grid minor
axis equal
set(gca, 'FontSize', 20)
xlabel("X [m]", 'FontSize', 20)
ylabel("Y [m]", 'FontSize', 20)
zlabel("Z [m]", 'FontSize', 20)

pltAgent = [1,2,3,4,5];
for j = 1:length(pltAgent)

    
    if j<no_uav
        j2 = j+1;
    else
        j2 = no_uav;
    end
    subplot(3,5,[1 2 3; 6 7 8; 11 12 13])
    % plot3(refTrajs{j2}(:,1)', refTrajs{j2}(:,2)',  refTrajs{j2}(:,3)','-','LineWidth', 1.5)
    plot3(swarm{pltAgent(j)}.trajectory(1,:), swarm{pltAgent(j)}.trajectory(2,:), swarm{pltAgent(j)}.trajectory(3,:), '--','LineWidth', 1.5, 'Color', colorList(pltAgent(j)))

    subplot(3,5,[4 5])
    plot(swarm{1}.trajectory(1,:), swarm{1}.trajectory(2,:), 'k-','LineWidth', 1.5)
    hold on
    plot(swarm{pltAgent(j)}.trajectory(1,:), swarm{pltAgent(j)}.trajectory(2,:), '--','LineWidth', 1.5, 'Color', colorList(pltAgent(j)))
    % plot(refTrajs{j2}(:,1)', refTrajs{j2}(:,2)', '-','LineWidth', 1.5)
    xlabel("X [m]", 'FontSize', 20)
    ylabel("Y [m]", 'FontSize', 20)
    set(gca, 'FontSize', 20)

    subplot(3,5,[9 10])
    plot(swarm{1}.trajectory(2,:), swarm{1}.trajectory(3,:), 'k-','LineWidth', 1.5)
    hold on
    plot(swarm{pltAgent(j)}.trajectory(2,:), swarm{pltAgent(j)}.trajectory(3,:), '--','LineWidth', 1.5, 'Color', colorList(pltAgent(j)))
    % plot(refTrajs{j2}(:,2)', refTrajs{j2}(:,3)', '-','LineWidth', 1.5)
    xlabel("Y [m]", 'FontSize', 20)
    ylabel("Z [m]", 'FontSize', 20)
    set(gca, 'FontSize', 20)

    subplot(3,5,[14 15])
    plot(swarm{1}.trajectory(1,:), swarm{1}.trajectory(3,:), 'k-','LineWidth', 1.5)
    hold on
    plot(swarm{pltAgent(j)}.trajectory(1,:), swarm{pltAgent(j)}.trajectory(3,:), '--','LineWidth', 1.5, 'Color', colorList(pltAgent(j)))
    % plot(refTrajs{j2}(:,1)', refTrajs{j2}(:,3)', '-','LineWidth', 1.5)
    xlabel("X [m]", 'FontSize', 20)
    ylabel("Z [m]", 'FontSize', 20)
    set(gca, 'FontSize', 20)
end
sgtitle("Agent 1 and 4 Trajectories", "FontSize", 30)



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



