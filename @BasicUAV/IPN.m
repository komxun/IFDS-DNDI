function [uh, uv] = IPN(obj)

    limitAngle = 1;
    
    if obj.flagIPN == 1
        obstacle = obj.gcsData;
        disp(obj.name + ": -----------------------IPN Activated!")
    elseif obj.flagIPN == 2
        obstacle = obj.obsData;
        disp(obj.name + ": ----------IPN for obstacle Activated!")
    end
    % Read uavs data
    X     = obj.itsCurrentPos;
    v     = obj.itsCruisingSpeed;
    psi   = obj.itsCurrentAngle(1);
    gamma = obj.itsCurrentAngle(2);

    X_obs     = obstacle(1:3);
    psi_obs   = obstacle(4);
    gamma_obs = obstacle(5);
    v_obs     = obstacle(6);

    R = norm(X_obs - X);    % Relative distance
    N = 3;                  % Proportional navigation gain (3 is optimal) (1000 works)

    % LOS angles
    smh = atan( (X_obs(2) - X(2))/ (X_obs(1) - X(1)) );

    smv = atan( (X_obs(3) - X(3))...
        / (sqrt( (X_obs(1) - X(1))^2 + (X_obs(2) - X(2))^2 )));


    psi_d = psi - smh;
    gamma_d = gamma - smv;

    LOS2V = rot2(psi_d) * rot3(gamma_d);

    % Wrapping up psid
    
    if limitAngle == 1

        temp = [0; gamma_d; psi_d];
        temp2 = LOS2V * temp;
        psi_d = temp2(3);
        gamma_d = temp2(2);

        psi_d = rem(psi_d, 2*pi);
        gamma_d = rem(gamma_d, 2*pi);

        % if psi_d < -pi
        %     psi_d = psi_d + 2*pi;
        % elseif psi_d > pi
        %     psi_d = psi_d - 2* pi;
        % end
        % % 
        % if gamma_d < -pi
        %     gamma_d = gamma_d + 2*pi;
        % elseif gamma_d > pi
        %     gamma_d = gamma_d-2*pi;
        % end
    
        % Limit turning angle
        if psi_d > pi/2
            psi_d = pi/2;
        elseif psi_d < -pi/2
            psi_d = -pi/2;
        end
        % 
        % % Limit pitching angle
        % if gamma_d > pi/2
        %     gamma_d = pi/2;
        % elseif gamma_d < -pi/2
        %     gamma_d = -pi/2;
        % end

        %-----For Debugging-----------
        % if obj.name == "Agent1"
        %     figure(888)
        %     scatter(obj.GetCurrentTime, psi_d*180/pi, 'filled', 'b'), hold on
        % end

        temp3 = (LOS2V)\[0; gamma_d; psi_d];
        psi_d = temp3(3);
        gamma_d = temp3(2);
    end

    % Relative distance rate
    % Rdot = v_obs * cos(psi_obs - smh) * cos(gamma_obs - smv) ...
    %          - v * cos(psi - smh) * cos(gamma - smv);
    

    Rdot = v_obs * cos(psi_obs - smh) * cos(gamma_obs - smv) ...
             - v * cos(psi_d) * cos(gamma_d);

    % LOS rates
    smh_dot = 1/R * (v_obs * cos(gamma_obs - smv) * sin(psi_obs - smh) ...
                    - v * cos(gamma_d) * sin(psi_d));
    smv_dot = 1/R * (v_obs * cos(psi_obs - smh) * sin(gamma_obs - smv) ...
                    - v * cos(psi_d) * sin(gamma_d));

    uh = N * Rdot * smh_dot;
    uv = N * Rdot * smv_dot;

    % Coordinate Conversion
    % LOS2V = rot3(gamma - smv) * rot2(psi - smh);
    % LOS2V = rot2(psi_d) * rot3(gamma_d); % works
    % LOS2V = rot1(-pi/2) * rot3(gamma - smv) * rot2(psi - smh);
    u_new = LOS2V * [0; uv; uh];   % [roll; pitch; yaw]
    uPitch = u_new(2);
    uYaw = u_new(3);

    uh = uYaw;
    uv = uPitch;

    umax = 10;
    % Limit u1
    if uh > umax
        uh= umax;
    elseif uh < -umax
        uh = - umax;
    end

    % Limit u2
    if uv > umax
        uv= umax;
    elseif uv < -umax
        uv = - umax;
    end

    psi_dot = uh/(v*cos(gamma));
    gamma_dot = uv/v;
end


function out = rot3(x)
    out = [cos(x) sin(x) 0; -sin(x) cos(x) 0; 0 0 1;];
end

function out = rot2(x)
    out = [cos(x) 0 -sin(x); 0 1 0; sin(x) 0 cos(x)];
end

function out = rot1(x)
    out = [1 0 0; 0 cos(x) sin(x); 0 -sin(x) cos(x)];
end
