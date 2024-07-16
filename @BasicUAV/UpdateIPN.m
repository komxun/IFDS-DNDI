function UpdateIPN(obj)
    dt = 0.01;
    [psi_dot, gamma_dot] = obj.IPN;
    
    X = obj.GetCurrentPos;
    V = obj.DA.Param.C; 
    psi0 = obj.DA.Param.psi_i;
    gamma0 = obj.DA.Param.gamma_i;
    
    %.. UAV Dynamics
    % Dynamic Model of UAV
    dx                  =               V * cos( gamma0 ) * cos( psi0 ) ;
    dy                  =               V * cos( gamma0 ) * sin( psi0 ) ;
    dz                  =               V * sin( gamma0);
    % dpsi                =               u1(i) / (V*cos(gamma(i))) ;
    % dgam                =               u2(i) / V;
    
    % UAV State Update
    x              =               X(1) + dx * dt; 
    y              =               X(2) + dy * dt;
    z              =               X(3) + dz * dt;
    psi            =               psi0 + psi_dot * dt ;
    gamma          =               gamma0 + gamma_dot * dt;
    
    
    % Saving states
    obj.itsCurrentPos = [x, y, z];
    obj.itsCurrentAngle = [psi, gamma];
    obj.itsCurrentState = [obj.GetCurrentPos, obj.GetCurrentVel];
    obj.itsTrajectory = [obj.itsTrajectory, obj.itsCurrentPos'];
end