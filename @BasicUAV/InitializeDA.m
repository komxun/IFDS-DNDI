function InitializeDA(obj, configStruct)

tempStruct = configStruct;
disp("___" + obj.name  + "'s DA configurations : __________")


disp(['Number of object: ' num2str(size(tempStruct.Object,2))])
if tempStruct.Param.sf == 0, disp("Shape-following: Off") 
else, disp("Shape-following: On")
end

switch tempStruct.Param.useOptimizer
    case 0,  disp("Path optimization: Off")
    case 1,  disp("Path optimization: Global");
    case 2,  disp("Path optimization: Local")
end
timer = zeros(1, tempStruct.Param.rtsim);
% Pre-allocate waypoints and path
Wp = zeros(3, tempStruct.Param.tsim+1);
Paths = cell(1, tempStruct.Param.rtsim);
traj = cell(1,2*2000);
% traj{1} = [x_i, y_i, z_i];
traj{1} = obj.currentPos;

Obstacle(tempStruct.Param.numObj) = struct('origin',zeros(tempStruct.Param.rtsim, 3),'Gamma',0,'Rstar',0);

tempStruct.Obstacle = Obstacle;
tempStruct.Wp = Wp;
tempStruct.Paths = Paths;
tempStruct.timer = timer;
tempStruct.traj = traj;
% tempStruct.destin = [Xfinal Yfinal Zfinal];
obj.DA = tempStruct;
end