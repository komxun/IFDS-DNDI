% .-------------------------------------.
% | Dynamic Autorouting Program         |
% | created by Komsun Tamanakijprasart  |
% '-------------------------------------'

function DynamicAutorouting(obj)
    DA = obj.DA;
    rt = obj.rt;
    Param = DA.Param;

    x_i = obj.currentPos(1);
    y_i = obj.currentPos(2);
    z_i = obj.currentPos(3);
    psi_i = obj.currentAngle(1);
    gamma_i = obj.currentAngle(2);
    Xini = obj.pathOrigin(1);
    Yini = obj.pathOrigin(2);
    Zini = obj.pathOrigin(3);
    C = obj.cruisingSpeed;
    destin = obj.destination;

    Wp = DA.Wp;
    traj = DA.traj;
    Paths = DA.Paths;
    Object = DA.Object;
    
    % Constants
    rho0 = DA.Param.rho0_initial;
    sigma0 = DA.Param.sigma0_initial;
    
    targetThresh = DA.Param.targetThresh;
    scene = DA.Param.scene;
    % destin = DA.destin;
    useOptimizer = DA.Param.useOptimizer;
    tuning = DA.tuning;

    numLine = size(destin,1);
    rt2 = ceil(rt/10);
tic

    if norm([x_i y_i z_i] - destin) < targetThresh  % [m]
        disp(obj.name + ": Destination reached at t = " + num2str(rt/100) + " s")
%         traj = traj(~cellfun('isempty',traj));
%         obj.DA.traj = traj;
        obj.SetFlagDestin(1)
        
    elseif obj.flagIPN ~= 0 && class(obj) == "FollowerUAV"
        [uh, uv] = obj.IPN();

        % Dynamic Model of UAV
        dx = C * cos(gamma_i) * cos(psi_i) ;
        dy = C * cos(gamma_i) * sin(psi_i) ;
        dz = C * sin(gamma_i);
        dpsi = uh / (C*cos(gamma_i)) ;
        dgam = uv / C;

        % UAV State Update
        dt = 0.01;
        x_i = obj.currentPos(1) + dx * dt; 
        y_i = obj.currentPos(2) + dy * dt;
        z_i = obj.currentPos(3) + dz * dt;
        psi_i = psi_i + dpsi * dt ;
        gamma_i = gamma_i + dgam * dt;

        % Save the updated states
        obj.currentPos = [x_i, y_i, z_i];
        obj.currentAngle(1) = psi_i;
        obj.currentAngle(2) = gamma_i;
        obj.currentState = [obj.currentPos, obj.GetCurrentVel()];

    else
%         obj.flagDestinReached = 0;
        if scene == 41 || scene == 42  || scene == 43 || class(obj) == "FollowerUAV"
            % Wp(:,1) = [x_i; y_i; z_i];      % UAV's current position
            Wp(:,1) = obj.currentPos;
        else
            Wp(:,1) = [Xini; Yini; Zini];  % Starting Location
        end
    
        %_____________________IFDS Path Calculation_______________________

        %------------Global Path Optimization-------------
        if useOptimizer == 1
           [rho0, sigma0] = path_optimizing(obj, destin, rt, Wp, Paths, Param, Object);
        end
        %------------------------------------------------
        for t = 1:obj.DA.Param.tsim
            xx = Wp(1,t);
            yy = Wp(2,t);
            zz = Wp(3,t);

            DA.Object = obj.create_scene(scene, Object, xx, yy, zz);
        end

        if rt > 1 && ~isempty(obj.globalPath) && class(obj) ~= "FollowerUAV"
            path2Follow{rt} = obj.globalPath;
            foundPath = 1;
        else
            % Compute the IFDS Algorithm
            [path2Follow, Object, ~, foundPath] = obj.IFDS(rho0, sigma0, destin, rt, Wp, Paths, Param, Object);
            if rt == 1 && foundPath>=1 && class(obj) ~= "FollowerUAV"
                disp(obj.name + ": setting global path . . .")
                obj.globalPath = path2Follow{rt};
            end
        end
        %_______________________CCA3D Calculation__________________________
        if foundPath ~= 1 || isempty(path2Follow) % || size(Paths{rt},2)==1 
            disp(obj.name + ": CAUTION - Path not found at t = " +num2str(rt/100) + " s")
            disp("*" + obj.name + " is standing by*")
        else
            % Compute Path Following Algorithm
            trajectory = zeros(3, length(path2Follow{rt}));
            trajectory(:,1) = [x_i; y_i; z_i];

            count = 1;
            dtcum = 0;
            
            for j = 1:length(path2Follow{rt})-1
                if dtcum >= 0.01
                    break
                end 
                Wi = path2Follow{rt}(:,j);
                Wf = path2Follow{rt}(:,j+1);
                
                path_vect = Wf - Wi;
                a = path_vect(1);
                b = path_vect(2);
                c = path_vect(3);
                      
                % Check if the waypoint is ahead of current position
                if a*(x_i - Wf(1)) + b*(y_i - Wf(2)) + c*(z_i - Wf(3)) < 0
                    [x, y, z, psi, gamma, timeSpent] = obj.CCA3D_straight(Wi, Wf, x_i, y_i, z_i, psi_i, gamma_i, C, tuning);
                    x_i = x(end);
                    y_i = y(end);
                    z_i = z(end);
                    psi_i = psi(end);
                    gamma_i = gamma(end);
                    dtcum = dtcum + timeSpent;
                  
                    trajectory(:,count+1) = [x(end); y(end); z(end)];
                    count = count+1;
                else
                    % skip waypoint
                end   
            end

        
%             trajectory = trajectory(:,1:count);   % remove extra element
%             DA.traj{rt} = trajectory;
            timer(rt) = toc;
            if DA.Param.showDisp == 1
                disp(obj.name + ": Simulation time = " + num2str(rt/100) + ", Computed time = " + num2str((timer(rt))) + " s")
            end

            % Save new states
            obj.currentPos = [x_i, y_i, z_i];
            obj.currentAngle = [psi_i, gamma_i];

            DA.Wp = Wp;
            DA.Paths{rt} = path2Follow{rt};
%             DA.Object = Object;
            % Save into obj
            obj.DA = DA;
        end
  
    end

end

%% Supporting functions

function [rho0, sigma0] = path_optimizing(obj, loc_final, rt, Wp, Paths, Param, Object)

    xg = [Param.rho0_initial; Param.sigma0_initial];

    lower_bound_rho = 0.05;  % <0.05 issue started to occur
    lower_bound_sigma = 0;
    
    upper_bound_rho = 2.5;
    upper_bound_sigma = 1;
    
    % Set up the optimization problem
    problem.objective = @(x) PathDistObjective(x(1), x(2));
    problem.x0 = xg;
    problem.Aineq = [];
    problem.bineq = [];
    problem.Aeq = [];
    problem.beq = [];
    problem.lb = [lower_bound_rho; lower_bound_sigma];
    problem.ub = [upper_bound_rho; upper_bound_sigma];
    problem.nonlcon = [];
    problem.solver = 'fmincon';  % specify the solver
    problem.options = optimoptions('fmincon', ...
        'Algorithm', 'interior-point', ...   % option2: sqp
        'Display', 'off');
    
    % Call fmincon
    [xOpt, fval, exitflag, output] = fmincon(problem);
    disp(output)
    rho0 = xOpt(1);
    sigma0 = xOpt(2);

    function totalLength = PathDistObjective(rho0, sigma0)
        Param.showDisp = 0;
        Param.useOptimizer = 0;
        [~, ~, totalLength] = ...
            obj.IFDS(rho0, sigma0, loc_final, rt, Wp, Paths, Param, Object);
   
    end
    
end