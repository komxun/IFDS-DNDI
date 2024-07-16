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

scene = tempStruct.Param.scene;
% k = tempStruct.Param.k;
if scene == 41 || scene == 42 || scene == 44  || class(obj) == "FollowerUAV" %|| k ~= 0 
    disp("** Dynamic environment **")
    obj.env = "dynamic";
    Paths = cell(1, tempStruct.Param.rtsim);
else
    disp("** Static environment **")
    obj.env = "static";
    Paths = cell(1, 1);
end

% Pre-allocate waypoints and path
Wp = zeros(3, tempStruct.Param.tsim+1);

traj = cell(1,2*2000);
% traj{1} = [x_i, y_i, z_i];
traj{1} = obj.itsCurrentPos;
            
Obstacle(tempStruct.Param.numObj-1) = struct('origin',zeros(1,3),...
'a',0,'b',0,'c',0,'p',0,'q',0,'r',0,'Rstar',0);
Obstacle = create_static_obs(tempStruct.Param.scene, Obstacle);
elm = struct('Gamma',0);
tempStruct.Obstacle = Obstacle;
tempStruct.Wp = Wp;
tempStruct.Paths = Paths;
tempStruct.timer = timer;
tempStruct.traj = traj;
tempStruct.elm = elm;
obj.DA = tempStruct;
end



function Obj = create_static_obs(num, Obj, rt)
    switch num
        case 1  % Single object
%             Obj(1) = create_cone(100, 5, 0, 50, 80, Obj(1));

            % Obj(1) = create_sphere(100, 5, 0, 50, Obj(1));
            % Obj(1) = create_sphere(100, 100, 0, 50, Obj(1));
            % Obj(1) = create_sphere(-4535.96, 29077.2, 180, 2000, Obj(1));  % don't forget swap x-y
            Obj(1) = create_sphere(-2963.129, 33966.46, 151.904, 2000, Obj(1));

            % Obj(1) = create_sphere(33966.46, -2963.129, 151.904, 3000, Obj(1));
    
        case 2 % 2 objects
            Obj(1) = create_sphere(621, 204.91, 67.8, 200, Obj(1));
            Obj(2) = create_cylinder(913, 41.7, 96, 210, 300, Obj(2));
    
        case 3 % 3 objects
            Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_sphere(120, -10, 0, 50, Obj(2));
            Obj(3) = create_cone(168, 0, 0, 25, 80, Obj(3));

        case 4 % single(complex) object
            Obj(1) = create_cylinder(100, 5, 0, 25, 200, Obj(1));
            Obj(2) = create_pipe(60, 20, 60, 80, 5, Obj(2));
            Obj(3) = create_pipe(130, -30, 30, 100, 50, Obj(3));
        case 5
            Obj(1) = create_cylinder(50, -20, 0, 30, 50, Obj(1));
            Obj(2) = create_cone(100, -20, 0, 30, 50, Obj(2));
            Obj(3) = create_pipe(150, -20, 0, 30, 50, Obj(3));
    
        case 12 % 12 objects
            Obj(1) = create_cylinder(100, 5, 0, 30, 50, Obj(1));
            Obj(2) = create_pipe(140, 20, 0, 40,10, Obj(2));
            Obj(3) = create_pipe(20, 20, 0, 24, 40, Obj(3));
            Obj(4) = create_pipe(55, -20, 0, 28, 50, Obj(4));
            Obj(5) = create_sphere(53, -60, 0, 50, Obj(5));
            Obj(6) = create_pipe(150, -80, 0, 40, 50, Obj(6));
            Obj(7) = create_cone(100, -35, 0, 50,45, Obj(7));
            Obj(8) = create_cone(170, 2, 0, 20,50, Obj(8));
            Obj(9) = create_cone(60, 35, 0, 50,30, Obj(9));
            Obj(10) = create_cylinder(110, 70, 0, 60, 50, Obj(10));
            Obj(11) = create_pipe(170, 60, 0, 40, 27, Obj(11));
            Obj(12) = create_cone(150, -30, 0, 32, 45, Obj(12));
        case 7 % 7 objects
            Obj(1) = create_cone(60,8, 0, 70, 50, Obj(1));
            Obj(2) = create_cone(100,-24, 0, 89, 100, Obj(2));
            Obj(3) = create_cone(160,40, -4, 100, 30, Obj(3));
            Obj(4) = create_cone(100,100, -10, 150, 100, Obj(4));
            Obj(5) = create_cone(180,-70, -10, 150, 20, Obj(5));
            Obj(6) = create_cone(75,-75, -10, 150, 40, Obj(6));
            Obj(7) = create_cylinder(170, -6, 0, 34, 100, Obj(7));  
        case 69 
            Obj(1) = create_cylinder(100, 5, 0, 30, 80, Obj(1));
            Obj(2) = create_sphere(100, 30, 0, 40, Obj(2));
            Obj(3) = create_sphere(100, -20, 0, 40, Obj(3));
            Obj(4) = create_sphere(100, 5, 80, 30, Obj(4));
        case 6969
            Obj(1) = create_sphere(100 + 30*sin(rt/8), 0 + 30*cos(rt/8), 0, 40, Obj(1));
            Obj(2) = create_cylinder(100, 0, 0, 40, 80, Obj(2));
            Obj(3) = create_sphere(100 - 30*sin(rt/8), 0 - 30*cos(rt/8), 0, 40, Obj(3));
    end

    function Obj = create_sphere(x0, y0, z0, D, Obj)
        
        a = D/2;   b = D/2;   c = D/2;      % Object's axis length
        p = 1;     q = 1;     r = 1;        % Index parameters
       
        % Save to Field
        Obj.origin = [x0, y0, z0];
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        Obj.Rstar = min([a,b,c]);
    end

    function Obj = create_cylinder(x0, y0, z0, D, h, Obj)
    
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 4;  % Index parameters
     
        % Save to Field
        Obj.origin = [x0, y0, z0]; 
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        Obj.Rstar = min([a,b,c]); 
    end
    
    function Obj = create_cone(x0, y0, z0, D, h, Obj)
             
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 0.5;  % Index parameters
    
        % Save to Field
        Obj.origin = [x0, y0, z0];
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        Obj.Rstar = min([a,b,c]);
    end
     
    function Obj = create_pipe(x0, y0, z0, D, h, Obj)
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 2;     q = 2;     r = 2;  % Index parameters
        
        % Save to Field
        Obj.origin = [x0, y0, z0];
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r; 
        Obj.Rstar = min([a,b,c]);
    end
    
end