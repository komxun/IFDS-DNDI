% .-------------------------------------------------------------.
% | Dynamic Autorouting Program for Multi-Agent Systems v.3.4   |
% | created by Komsun Tamanakijprasart and Dr.Sabyasachi Mondal |
% '-------------------------------------------------------------'

function tempStruct = Config()
% ___________________Simulation Set-up Parameters__________________________
fontSize = 20;
saveVid = 0;
animation = 0;              % Figure(69)m 1: see the simulation
showDisp = 1;               % Show console display 1: on 0: off
tsim = uint16(400);          % [s] simulation time for the path 
N = 50*100;                   % [s] (50) time for the whole scenario 
dt = 1;                    % [s] simulation time step
simMode = uint8(2);          % 1: by time, 2: by target distance
targetThresh = 2.5;          % [m] allowed error for final target distance 
multiTarget = uint8(0);      % 1: multi-target 0: single-target

% scenelist = {'scene 1: 1 object', 'scene 2: 2 objects', 'scene 3: 3 objects',...
%     'scene 4: 3 complex objects'};
% scene = listdlg('PromptString',{'Select your scenario.', ...
%     '(Only one option can be', 'selected at a time)'},'ListString',scenelist);

% scene = 2;
scene = 8;
% scene = 41;

% ___________________Features Control Parameters___________________________
useOptimizer = 0; % 0:Off  1:Global optimized  2: Local optimized
delta_g = 5;            % [m] Safeguarding distance (minimum allowed gap)

% ______________________IFDS Tuning Parameters_____________________________
sf    = uint8(1);   % Shape-following demand (1=on, 0=off)
rho0  = 4.5;          % Repulsive parameter (rho >= 0)
sigma0 = 0.2;      % Tangential parameter 

% Good: rho0 = 2, simga0 = 0.01
% The algorihtm still doesnt work for overlapped objects

% _______________________CCA Tuning Parameters_____________________________
ccaTuning = 5;
switch ccaTuning
    case 1
        kappa =  10 ;              % Gain
        delta =  2;                % Carrot Distance
        kd = 0;
    case 2
        kappa = 20;
        delta = 5;
        kd = 0;
    case 3
        kappa = 10;
        delta = 10;
        kd = 0.1;
    case 4
        kappa = 100;
        delta = 1;
        kd = 0;
    case 5
        kappa = 50;
        delta = 20;
        kd = 0;
end

tuning = [kappa, delta, kd];

% _______________________ UAV Parameters _________________________________
C  = 10;             % [m/s] UAV cruising speed (30)

% Save to table Param
Param = table;
Param.showDisp = showDisp;
Param.tsim = tsim;
Param.rtsim = N;
Param.dt = dt;
Param.targetThresh = targetThresh;
Param.simMode = simMode;
Param.multiTarget = multiTarget;
Param.scene = scene;
Param.sf = sf;
Param.Rg = delta_g;
Param.rho0_initial = rho0;
Param.sigma0_initial = sigma0;
Param.useOptimizer = useOptimizer;
Param.fontSize = fontSize;
Param.saveVid = saveVid;
Param.animation = animation;

switch scene
    case 0, numObj = 1;
    case 1, numObj = 1;
    case 2, numObj = 2;
    case 3, numObj = 3;
    case 4, numObj = 3;
    case 5, numObj = 3;
    case 7, numObj = 7;
    case 8, numObj = 8;
    case 12, numObj = 12;
    case 41, numObj = 3;
    case 42, numObj = 4;
    case 43, numObj = 4;
end
Param.numObj = numObj;
Object(1:numObj) = struct('origin',zeros(N,3),'Gamma',0,'n',[],'t',[],...
'a',0,'b',0,'c',0,'p',0,'q',0,'r',0,'Rstar',0);

tempStruct = struct;
tempStruct.Param = Param;
tempStruct.Object = Object;
tempStruct.tuning = tuning;

end