function Object = create_scene(obj, num, Object, X, Y, Z)
    rt = obj.rt;
    switch num
        case 0
            Object(1) = create_ceiling(100, 0, 50, 200, 10, Object(1));

        case 1  % Single object
            Object(1) = create_sphere(100, 5, 0, 50, Object(1));
            % Obj(1) = create_cone(100, 5, 0, 50, 80, Obj(1));
            % Obj(1) = create_sphere(100, 180, 0, 50, Obj(1));

    
        case 2 % 2 objects
            Object(1) = create_cone(90, 5, 0, 50, 80, Object(1));
            Object(2) = create_cylinder(160, -20, 0, 40, 70, Object(2));

            % Obj(1) = create_cylinder(60, 5, 0, 30, 50, Obj(1));
            % Obj(2) = create_sphere(120, -10, 0, 50, Obj(2));
    
        case 3 % 3 objects
            Object(1) = create_cone(90, 5, 0, 50, 80, Object(1));
            Object(2) = create_cylinder(140, -40, 0, 40, 70, Object(2));
            Object(3) = create_cylinder(152.5, 34, 0, 30, 90, Object(3));

        case 4 % single(complex) object
            Object(1) = create_cylinder(100, 5, 0, 25, 200, Object(1));
            Object(2) = create_pipe(60, 20, 60, 80, 5, Object(2));
            Object(3) = create_pipe(130, -30, 30, 100, 5, Object(3));
        case 5
            Object(1) = create_cylinder(50, -20, 0, 30, 50, Object(1));
            Object(2) = create_cone(100, -20, 0, 30, 50, Object(2));
            Object(3) = create_pipe(150, -20, 0, 30, 50, Object(3));
    
        case 12 % 12 objects
            Object(1) = create_cylinder(100, 5, 0, 30, 50, Object(1));
            Object(2) = create_pipe(140, 20, 0, 40,10, Object(2));
            Object(3) = create_pipe(20, 20, 0, 24, 40, Object(3));
            Object(4) = create_pipe(55, -20, 0, 28, 50, Object(4));
            Object(5) = create_sphere(53, -60, 0, 50, Object(5));
            Object(6) = create_pipe(150, -80, 0, 40, 50, Object(6));
            Object(7) = create_cone(100, -35, 0, 50,45, Object(7));
            Object(8) = create_cone(170, 2, 0, 20,50, Object(8));
            Object(9) = create_cone(60, 35, 0, 50,30, Object(9));
            Object(10) = create_cylinder(110, 70, 0, 60, 50, Object(10));
            Object(11) = create_pipe(170, 60, 0, 40, 27, Object(11));
            Object(12) = create_cone(150, -30, 0, 32, 45, Object(12));
        case 7 % 7 objects
            Object(1) = create_cone(60,8, 0, 70, 50, Object(1));
            Object(2) = create_cone(100,-24, 0, 89, 100, Object(2));
            Object(3) = create_cone(160,40, -4, 100, 30, Object(3));
            Object(4) = create_cone(100,100, -10, 150, 100, Object(4));
            Object(5) = create_cone(180,-70, -10, 150, 20, Object(5));
            Object(6) = create_cone(75,-75, -10, 150, 40, Object(6));
            Object(7) = create_cylinder(170, -6, 0, 34, 100, Object(7));
        case 8
            Object(1) = create_cone(90, 5, 0, 50/1.5, 80, Object(1));
            Object(2) = create_cylinder(150, -10, 0, 40/1.5, 70, Object(2));
            Object(3) = create_cylinder(152.5, 50, 0, 30/1.5, 90, Object(3));
            Object(4) = create_pipe(210, 30, 0, 40/1.5, 90, Object(4));
            Object(5) = create_pipe(220, -40, 0, 40/1.5, 40, Object(5));
            Object(6) = create_cone(100,70,0,30/1.5,70,Object(6));
            Object(7) = create_sphere(67, -63, 0, 50, Object(7));
            Object(8) = create_sphere(50, 80, 0, 50, Object(8));
        case 41 % Dynamic case 1
            Object(1) = create_cylinder(100 + 50*sin(rt/8/50), 0 + 50*cos(rt/8/50), 0, 20, 80, Object(1));
            Object(2) = create_sphere(100, 0, 0, 30, Object(2));
            Object(3) = create_cylinder(100 - 50*sin(rt/8/50), 0 - 50*cos(rt/8/50), 0, 20, 50, Object(3));

        case 42 % Dynamic case 2
            Oy1 = -5 + 60*cos(0.4*single(rt));
            Oy2 = -20 - 20*sin(0.8*single(rt));
            Oz2 =  60 + 20*cos(0.8*single(rt));
            Object(1) = create_cylinder(60, 5, 0, 30, 50, Object(1));
            Object(2) = create_cylinder(110, -10, 0, 25, 80,Object(2));
            Object(3) = create_cylinder(80, Oy1, 0, 20, 60, Object(3));
            Object(4) = create_sphere(160, Oy2, Oz2, 30, Object(4));
    
        case 43 % Dynamic case 3
%             Obj(1) = create_cylinder(80 + 140*sin(rt/8/50), 0 + 140*cos(rt/8/50), 0, 20, 80, Obj(1));
%             Obj(2) = create_sphere(80, 0, 0, 30, Obj(2));
%             Obj(3) = create_cylinder(80 - 140*sin(rt/8/50), 0 - 140*cos(rt/8/50), 0, 20, 50, Obj(3));
%             Obj(4) = create_cone(158, 30, 0, 45, 80, Obj(4));

            Object(1) = create_cylinder(150 + 70*sin(rt/8/70+pi/2), 0 + 70*cos(rt/8/70+pi/2), 0, 30, 80, Object(1));
            Object(2) = create_cylinder(150 - 70*sin(rt/8/70+pi/2), 0 - 70*cos(rt/8/70+pi/2), 0, 30, 50, Object(2));
            Object(3) = create_cylinder(150 + 50*sin(rt/8/70), 0 + 50*cos(rt/8/70), 0, 30, 80, Object(3));
            Object(4) = create_cylinder(150 - 50*sin(rt/8/70), 0 - 50*cos(rt/8/70), 0, 30, 50, Object(4));
            
    end
    

    function Obj = create_sphere(x0, y0, z0, D, Obj)
        
        a = D/2;   b = D/2;   c = D/2;      % Object's axis length
        p = 1;     q = 1;     r = 1;        % Index parameters
       
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        n = [dGdx; dGdy; dGdz];
        t = [dGdy; -dGdx; 0];
%-----------------------------------------------------------------------
        
        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        Obj.Rstar = min([a,b,c]);
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end
    end

    function Obj = create_cylinder(x0, y0, z0, D, h, Obj)
    
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 4;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();

        n = [dGdx; dGdy; dGdz];
        t = [dGdy; -dGdx; 0];


        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0]; 
        Obj.Gamma = Gamma;
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        Obj.Rstar = min([a,b,c]);
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end    
    end
    
    function Obj = create_cone(x0, y0, z0, D, h, Obj)
             
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 1;     q = 1;     r = 0.5;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();
        
        % n and t
        n = [dGdx; dGdy; dGdz];
        t = [dGdy; -dGdx; 0];

        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r;
        Obj.Rstar = min([a,b,c]);
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end    
    end
     
    function Obj = create_pipe(x0, y0, z0, D, h, Obj)
             
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 2;     q = 2;     r = 2;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();

        % n and t
        n = [dGdx; dGdy; dGdz];
        t = [dGdy; -dGdx; 0];
        
        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r; 
        Obj.Rstar = min([a,b,c]);
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end
    end

    function Obj = create_ceiling(x0, y0, z0, D, h, Obj)
             
        z0 = z0 + h;
        a = D/2;   b = D/2;   c = h;    % Object's axis length
        p = 20;     q = 20;     r = 20;  % Index parameters
     
        % Object Shape Equation
        Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
        
        % Differential
        [dGdx, dGdy, dGdz] = calc_dG();

        % n and t
        n = [dGdx; dGdy; dGdz];
        t = [dGdy; -dGdx; 0];
        
        % Save to Field
        Obj.origin(rt,:) = [x0, y0, z0];
        Obj.Gamma = Gamma;
        Obj.n = n;
        Obj.t = t;
        Obj.a = a;
        Obj.b = b;
        Obj.c = c;
        Obj.p = p;
        Obj.q = q;
        Obj.r = r; 
        Obj.Rstar = min([a,b,c]);
        function [dGdx, dGdy, dGdz] = calc_dG()
            dGdx = (2*p*((X - x0)/a).^(2*p - 1))/a;
            dGdy = (2*q*((Y - y0)/b).^(2*q - 1))/b;
            dGdz = (2*r*((Z - z0)/c).^(2*r - 1))/c;
        end
    end

    
end