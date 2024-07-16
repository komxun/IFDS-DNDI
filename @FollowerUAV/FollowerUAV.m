% .-------------------------------------------------------------.
% | Dynamic Autorouting Program for Multi-Agent Systems v.3.4   |
% | created by Komsun Tamanakijprasart and Dr.Sabyasachi Mondal |
% '-------------------------------------------------------------'

classdef FollowerUAV < BasicUAV 
    properties (Access = public)
        % Public Function Prototype
    end

    methods (Access = public)
        function Follow(obj, state)
            % obj.IncrementTimeStep
            obj.itsTrajectory = [obj.itsTrajectory, state(1:3)'];
            obj.itsCurrentPos = state(1:3);
            obj.itsCurrentState = state;
            obj.itsAngle = [obj.itsAngle, obj.itsAngle(:,end)];
        end
        
        function UpdateFollower(obj, state, Object, leaderFlagDanger)
            obj.CheckDanger(Object) 

            if (obj.flagDanger == 0) && (leaderFlagDanger == 0)
                obj.Follow(state)
            else
                obj.UpdateDA()
            end
            obj.IncrementTimeStep
            obj.flagIPN = 0;
            
        end
    end

end