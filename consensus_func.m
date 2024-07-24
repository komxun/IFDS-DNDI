function X = consensus_func(X_nei, XL_pos_to_fol, XL_states, no_uav, swarm)

%%%%============================================%%%%
% This function runs on the ground station in a    %
% centralized architecture                         %
%%%%============================================%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Position tracking and velocity consensus             %               
%  X_nei (no_uav x 6):            includes the ith agent's states    %
%  XL_pos_to_fol (3 x no_uav):    contains positon for all followers %
%  XL_states (1 x 6) :        States of leader [x, y, z, Vx, Vy, Vz]    %
%  comms:            1-communication exists or not (0)  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

showPlot = 0;

global h U
h =0.01;
t0 =0;
tf =h;
dt =h;
N = 1;
% Communication topology
% A=[ 0	1	1	0	1;
%     1	0	0	0	1;
%     1	0	0	1	1;
%     0	0	1	0	0;
%     1	1	1	0	0];

A=[0	0	1	0	0;
   0	0	0	0	1;
   1	0	0	1	1;
   1	0	0	0	0;
   0	1	1	0	0];

% A=[0	1	1	1	1;
%    1	0	1	1	1;
%    1	1	0	1	1;
%    1	1	1	0	1;
%    1	1	1	1	0];
% A = eye(5);


% Leader comms
% lead_fol_con = [3 5 2 4 2];
lead_fol_con = [0 1  0 0 0]*1e6;
% lead_fol_con = [0 1 1 0 1]*1e6;

% lead_fol_con = double([swarm{2}.GetFlagComs, ...
%                 swarm{3}.GetFlagComs, ...
%                 swarm{4}.GetFlagComs, ...
%                 swarm{5}.GetFlagComs, ...
%                 swarm{6}.GetFlagComs]) * double(swarm{1}.GetFlagComs) * 1e3;


p  = length(X_nei(1,1:3));


%% Extracting leader info

XL_dot = XL_states(4:6);

%% Loading variables

Y_nei = X_nei(1,1:3)';                              %%% (15 x 1) position
for i = 2:1:no_uav
    Y_nei = [Y_nei;X_nei(i,1:3)'];
end

% Y_nei = XL_pos_to_fol(:,1);                              %%% (15 x 1) position
% for i = 2:1:no_uav
%     Y_nei = [Y_nei;XL_pos_to_fol(:,i)];
% end


Y_dot_nei = X_nei(1,4:6)';                          %%%% (15 x 1) velocity
for i = 2:1:no_uav
    Y_dot_nei = [Y_dot_nei;X_nei(i,4:6)'];
end

Y_dot = [XL_dot'; Y_dot_nei];   %%%%%%%%%%%      (18 x 1)

%% Gains

Ke1 = 5*eye(p);
Ke2 = 2*eye(p);

% Ke1 = 100*eye(p);
% Ke2 = 100*eye(p);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Main loop starts %%%
%%%%%%%%%%%%%%%%%%%%%%%%%

X = zeros(no_uav, 6);
er = zeros(no_uav, 3);
er_dot = zeros(no_uav, 3);
gcs = GCS;
for i=1:1:N    %% N=1 function will run at every time instant

    for j = 1:1:no_uav

        x = X_nei(j,:);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        y = x(1:3)';                                %%%% output(position)
        y_dot = x(4:6)';                            %%%% velocity
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        d_I = kron(sum(A(j,:)),eye(p));
        g_I = kron(lead_fol_con(j),eye(p));
        a_I = [ g_I kron(A(j,:),eye(p))];

        %%%%%%%%% error calc %%%%%%%%%%%%
        Y = [XL_pos_to_fol(:,j) ; Y_nei];          %%% (18 x 1) put leader position alongwith the followers

        er(j,:)     = (d_I+g_I)*y-a_I*Y;            %%%% cons error
        er_dot(j,:) = (d_I+g_I)*y_dot - a_I*Y_dot;  %%%% error dot

        %%%%%%%%%% DNDI  %%%%%%%%%%%%%

        U = -pinv(d_I+g_I)*(Ke1*er_dot(j,:)'+Ke2*er(j,:)');
     

%% Dynamics : integrate using RK4

        Out = rk4_m('Dyn_imple',t0,tf,dt,x');
           
        X(j,:)  = double(Out');
        Uu(j,:) = U';
        
    end
    
     
        %%%%% Store for trajectory  %%%%%

        traj(:,:,i)   = X;
        err(:,:,i)    = er;
        Control(:,:,i)= Uu;
    

%% Update Y and Ydot

        Y = X(1,1:3)';
    for k = 2:1:no_uav
        Y = [Y;X(k,1:3)'];
    end
    Y_dot = X(1,4:6)'; 
    for k = 2:1:no_uav
        Y_dot = [Y_dot;X(k,4:6)'];
    end 
%     Y_dot = [XL_dot'; Y_dot];

end

%%  Plotting variables
%%%%%%%%%%%%%%%%%%%%%%
for i=1:1:N
    States=traj(:,:,i);
    Ua=Control(:,:,i);
    err_st=err(:,:,i);
    S1(:,i)=States(:,1);
    S2(:,i)=States(:,2);
    S3(:,i)=States(:,3);
    S4(:,i)=States(:,4);
    S5(:,i)=States(:,5);
    S6(:,i)=States(:,6);
    U1(:,i)=Ua(:,1);
    U2(:,i)=Ua(:,2);
    U3(:,i)=Ua(:,3);
    err_x1(:,i)=err_st(:,1);
    err_x2(:,i)=err_st(:,2);
    time(i)=i*h;
    
    limit_states1(i)=10+S1(1,i);
 end   

%%  Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%
col_code=[1 0.5 0;1 0 1;0 1 1;1 0.3 0;0 1 0;0 0 1;1 .1 0;0 0 0;0.5 0 0.3;0 0.5 0.6];

if showPlot
    figure(1)
    hold on
    for i=1:1:no_uav
        plot(time,S1(i,:),'Color',col_code(i,:),'LineWidth',1.5)
        xlabel('time(s)')
        ylabel('X_1')
        box on
        hold on
        legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
            'Agent8','Agent9','Agent10')
    end
    
    figure(2)
    hold on
    for i=1:1:no_uav
        plot(time,S2(i,:),'LineWidth',1.5)
        xlabel('time(s)')
        ylabel('X_2')
        box on
        hold on
        legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
            'Agent8','Agent9','Agent10')
    end
    
    figure(3)
    hold on
    for i=1:1:no_uav
        plot(time,S3(i,:),'LineWidth',1.5)
        xlabel('time(s)')
        ylabel('X_3')
        box on
        hold on
        legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
            'Agent8','Agent9','Agent10')
    end
    figure(4)
    hold on
    for i=1:1:no_uav
        plot(time,S4(i,:),'LineWidth',1.5)
        xlabel('time(s)')
        ylabel('X_4')
        box on
        hold on
        legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
            'Agent8','Agent9','Agent10')
    end
    
    figure(5)
    hold on
    for i=1:1:no_uav
        plot(time,S5(i,:),'LineWidth',1.5)
        xlabel('time(s)')
        ylabel('X_5')
        box on
        hold on
        legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
            'Agent8','Agent9','Agent10')
    end
    
    figure(6)
    hold on
    for i=1:1:no_uav
        plot(time,S6(i,:),'LineWidth',1.5)
        xlabel('time(s)')
        ylabel('X_6')
        box on
        hold on
        legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
            'Agent8','Agent9','Agent10')
    end
    
    figure(7)
    hold on
    for i=1:1:no_uav
        plot(time,U1(i,:),'LineWidth',1.5)
        xlabel('time(s)')
        ylabel('U_1')
        box on
        hold on
        legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
            'Agent8','Agent9','Agent10','k--')
    end
    
    figure(8)
    hold on
    for i=1:1:no_uav
        plot(time,U2(i,:),'LineWidth',1.5)
        xlabel('time(s)')
        ylabel('U_2')
        box on
        hold on
        legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
            'Agent8','Agent9','Agent10')
    end
    figure(9)
    hold on
    for i=1:1:no_uav
        plot(time,U3(i,:),'LineWidth',1.5)
        xlabel('time(s)')
        ylabel('U_3')
        box on
        hold on
        legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
            'Agent8','Agent9','Agent10')
    end
    
    figure(10)
    hold on
    for i=1:1:no_uav
        plot3(S1(i,:),S2(i,:),S3(i,:),'LineWidth',1.5)
        hold on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        grid on
        box on
        hold on
    end
end

% figure(9)
% hold on
% for i=1:1:no_uav
%     plot(time,err_x1(i,:),'LineWidth',1.5)
%     xlabel('time(s)')
%     ylabel('e_i in X_1')
%     box on
%     hold on
%     legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
%         'Agent8','Agent9','Agent10')
% end
% plot(time,Ke_u1,'r','LineWidth',1.5)
% plot(time,Ke_l1,'r','LineWidth',1.5)

% figure(10)
% hold on
% for i=1:1:no_uav
%     plot(time,err_x2(i,:),'LineWidth',1.5)
%     xlabel('time(s)')
%     ylabel('e_i in X_2')
%     box on
%     hold on
%     legend('Agent1','Agent2','Agent3','Agent4','Agent5','Agent6','Agent7',...
%         'Agent8','Agent9','Agent10')
% end
% plot(time,Ke_u2,'r','LineWidth',1.5)
% plot(time,Ke_l2,'r','LineWidth',1.5)

% figure(11)
% plot(time,SIG1(1,:))
% xlabel('time (s)')
% ylabel('noise intensity')

% figure(12)
% plot(time,sw_ind)
% xlabel('time (s)')
% ylabel('topology')
% 
% figure(13)
% plot(time,noise_comm1)
% hold on
% plot(time,noise_comm2)
% xlabel('time (s)')
% ylabel('External bounded disturbance')
% 
% 
% 
% 
% 
% 
% 
out=[U1; U2];