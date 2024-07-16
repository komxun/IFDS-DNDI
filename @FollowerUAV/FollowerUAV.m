% .-------------------------------------------------------------.
% | Dynamic Autorouting Program for Multi-Agent Systems v.4.1   |
% | created by Komsun Tamanakijprasart and Dr.Sabyasachi Mondal |
% '-------------------------------------------------------------'

classdef FollowerUAV < BasicUAV 
    properties (Access = public)
        % Public Function Prototype
    end

    methods (Access = public)
        function Follow(obj, state)
            % obj.IncrementTimeStep
            obj.itsCurrentPos = state(1:3);
            % obj.itsCurrentState = state;
            % obj.itsAngle = [obj.itsAngle, obj.itsAngle(:,end)];
            vx = state(4);
            vy = state(5);
            vz = state(6);
     
            psi = atan2(vy, vx);
            gamma = atan2(vz, sqrt(vx^2 + vy^2));
            obj.SetCurrentAngle(psi, gamma)
            obj.SetItsCruisingSpeed(norm(state(4:end)))
            % obj.UpdateTrajectory()
        end
        
        function UpdateFollower(obj, state2Follow, Object, leaderFlagDanger)
            obj.CheckDanger(obj.DA.Obstacle) 

            % obj.CheckTerrain()

            if (obj.flagDanger == 0) && (leaderFlagDanger == 0)
                obj.Follow(state2Follow)
            else
                obj.UpdateDA()
            end
            obj.IncrementTimeStep
            obj.flagIPN = 0;
            
        end
    end

end