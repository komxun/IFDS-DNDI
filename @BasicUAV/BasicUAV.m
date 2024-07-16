% .-------------------------------------------------------------.
% | Dynamic Autorouting Program for Multi-Agent Systems v.3.4   |
% | created by Komsun Tamanakijprasart and Dr.Sabyasachi Mondal |
% '-------------------------------------------------------------'

classdef BasicUAV < handle   % < handle : pass the value by reference 
    properties (Access = public)
        itsTrajectory (3,:) double
        itsAngle(2,:) double
        itsCurrentAngle (1,2) single
        itsPath cell
        DA struct
        
        flagIPN (1,1) uint8 = 0
        gcsData (1,6) double
        obsData (1,6) double
        itsCruisingSpeed (1,1) single 
    end

    properties (Access = protected)
        itsPathOrigin (1,3) double
        itsDestination (1,3) double
        rt (1,1) single {mustBeNonnegative} = 1
        itsCurrentPos (1,3) single
        itsCurrentState (1,6) single
        
        gmThresh (1,1) single {mustBePositive} = 2

        % UAV specs
        name string
        

        % Flag variables
        flagDestinReached (1,1) uint8 = 0
        flagComsAvailable (1,1) uint8 = 1
        flagDanger (1,1) uint8 = 0
    end

    methods (Access = public)
        % Constructor
        function obj = BasicUAV(x_i, y_i, z_i, psi_i, gamma_i)
            if nargin == 0  % Default
                disp(obj.name + ": Using Default UAV Initial States...")
                % Default UAV's Initial State
                x_i = 20;
                y_i = 20;
                z_i = 30;
                psi_i = 0;          % [rad] Initial Yaw angle
                gamma_i = 0;        % [rad] Initial Pitch angle
            end
            % Default UAV's name
            obj.name = "[Unnamed BasicUAV]";
            disp("* " + obj.name + " is created")

            % Default Path's Starting location
            Xini = 20;
            Yini = 0;
            Zini = 30;

            % Default Target Destination
            Xfinal = 250;  % 270
            Yfinal = 0;
            Zfinal = 50;

            % Default UAV's Cruising Speed [m/s]
            obj.itsCruisingSpeed = 10;
            obj.itsCurrentPos = [x_i, y_i, z_i];
            obj.itsCurrentAngle = [psi_i, gamma_i];
            obj.itsAngle = [psi_i; gamma_i];
            obj.itsCurrentState = [obj.itsCurrentPos, obj.GetCurrentVel];
            obj.itsDestination = [Xfinal, Yfinal, Zfinal];
            obj.itsPathOrigin = [Xini, Yini, Zini];

        end

        function Initialize(obj, configStruct)
            disp("* Initializing " + obj.name + " . . .")
            obj.InitializeDA(configStruct)
            disp("________________________________________")
        end
        
        function pos = GetCurrentPos(obj)
            pos = obj.itsCurrentPos;
        end

        function angle = GetCurrentAngle(obj)
            angle = obj.itsCurrentAngle;
        end

        function vel = GetCurrentVel(obj)
            psi_i = obj.itsCurrentAngle(1);
            gamma_i = obj.itsCurrentAngle(2);
            vel = obj.itsCruisingSpeed * [cos(gamma_i) * cos(psi_i),...
                                cos(gamma_i) * sin(psi_i),...
                                sin(gamma_i)];
        end
        
        function state = GetCurrentState(obj)
            state = obj.itsCurrentState;
        end

        function rt = GetCurrentTime(obj)
            rt = obj.rt;
        end

        function destin = GetItsDestination(obj)
            destin = obj.itsDestination;
        end

        function pathOg = GetItsPathOrigin(obj)
            pathOg = obj.itsPathOrigin;
        end

        function name = GetName(obj)
            name  = obj.name;
        end

        % ---------------Get Flags values--------------------
        function val = GetFlagComs(obj)
            val = obj.flagComsAvailable;
        end

        function val = GetFlagDestin(obj)
            val = obj.flagDestinReached;
        end

        function val = GetFlagDanger(obj)
            val = obj.flagDanger;
        end

        function SetCurrentPos(obj, x, y, z)
           obj.itsCurrentPos = [x, y, z];
        end

        function SetCurrentAngle(obj, psi, gamma)
            obj.itsCurrentAngle = [psi, gamma];
        end

        function SetCurrentState(obj, x, y, z, vx, vy, vz)
            obj.itsCurrentState = [x, y, z, vx, vy, vz];
        end

        function SetItsDestination(obj, x, y, z)
            destin = [x, y, z];
            oldDt = "(" + num2str(obj.itsDestination(1)) + "," + num2str(obj.itsDestination(2)) + ","...
                + num2str(obj.itsDestination(3)) + ")";
            newDt= "(" + num2str(x) + "," + num2str(y) + ","+ num2str(z) + ")";
            disp("* " + obj.name + "'s destination has been changed from " + oldDt + " to " + newDt)
            obj.itsDestination = destin;
        end

        function SetItsPathOrigin(obj, x ,y ,z)
            pathOg = [x, y, z];
            oldOg = "(" + num2str(obj.itsPathOrigin(1)) + "," + num2str(obj.itsPathOrigin(2)) + ","...
                + num2str(obj.itsPathOrigin(3)) + ")";
            newOg = "(" + num2str(x) + "," + num2str(y) + ","+ num2str(z) + ")";
            disp("* " + obj.name + "'s path origin has been changed from " + oldOg + " to " + newOg)
            obj.itsPathOrigin = pathOg;
        end

        function SetName(obj, name)
            disp("* " + obj.name + "'s name has been changed to " + name)
            obj.name = name;
        end

        % ---------------Set Flags values----------------------
        function SetFlagComs(obj, val)
            arguments, obj, val (:,:) {mustBeInteger, mustBeInRange(val, 0, 1)} 
            end
            obj.flagComsAvailable = val;
            disp(obj.name + ": Setting flagComsAvailable to " + num2str(val) + " ...");
        end

        function SetFlagDestin(obj, val)
            arguments, obj, val (:,:) {mustBeInteger, mustBeInRange(val, 0, 1)} 
            end
            obj.flagDestinReached = val;
            disp(obj.name + ": Setting flagDestinAvailable to " + num2str(val) + " ...");
        end

        function SetFlagDanger(obj, val)
            arguments, obj, val (:,:) {mustBeInteger, mustBeInRange(val, 0, 1)} 
            end
            obj.flagDanger = val;
            disp(obj.name + ": Setting flagDanger to " + num2str(val) + " ...");
        end

        function SetTimeStep(obj, val)
            obj.rt = val;
        end
        %----------------------------------------------------------

        function IncrementTimeStep(obj)
            obj.rt = obj.rt + 1;
        end

        function CheckDanger(obj, Object)
  
            S = obj.itsCurrentPos;
            X = S(1);
            Y = S(2);
            Z = S(3);

            for j = 1:size(Object,2)
                
                t = obj.rt;

                x0 = Object(j).origin(t, 1);
                y0 = Object(j).origin(t, 2);
                z0 = Object(j).origin(t, 3);
                a = Object(j).a;
                b = Object(j).b;
                c = Object(j).c;
                p = Object(j).p;
                q = Object(j).q;
                r = Object(j).r;
                Rstar = Object(j).Rstar;
                Rg = obj.DA.Param.Rg;
            
                Gamma = ((X - x0) / a).^(2*p) + ((Y - y0) / b).^(2*q) + ((Z - z0) / c).^(2*r);
                gm = Gamma - ( (Rstar + Rg)/Rstar )^2 + 1;
    
                if (gm <= obj.gmThresh) 
                    disp(obj.name + ": Danger detected!")
                    if (obj.flagDanger == 0)
%                         obj.SetFlagDanger(1)
                        obj.flagIPN = 0;
                        obj.flagDanger = 1;
                        obj.obsData = [x0, y0, z0, 0, 0, 0]; % static obstacle for now
                    end
                    break
                else
                    obj.flagDanger = 0;
                end
            end
        end
 
        function UpdateDA(obj) 
            % Propagate states
            obj.DynamicAutorouting()
            % Saving states
            obj.itsCurrentState = [obj.GetCurrentPos, obj.GetCurrentVel];
            obj.itsTrajectory = [obj.itsTrajectory, obj.itsCurrentPos'];
        end

        function UpdateBasicUAV(obj)
            obj.UpdateDA
            obj.CheckDanger(obj.DA.Object)
            obj.IncrementTimeStep
        end
        
    end

    methods (Access = protected)
        % Private Function Prototype
        [uh, uv] = IPN(obj)
        InitializeDA(obj, configStruct)
        DynamicAutorouting(obj)
        [Paths, Object, totalLength, foundPath] = IFDS(obj, rho0, sigma0, loc_final, rt, Wp, Paths, Param, Object)
        [x, y, z, psi, gamma, timeSpent] = CCA3D_straight(obj, Wi, Wf, x0, y0, z0, psi0, gamma0, V, tuning)
    end

end


