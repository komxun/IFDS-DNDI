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
            obj.trajectory = [obj.trajectory, state(1:3)'];
            obj.currentPos = state(1:3);
            obj.currentState = state;
            obj.angle = [obj.angle, obj.angle(:,end)];
        end
        
        function UpdateFollower(obj, state, Object, leaderFlagDanger, leaderFlagComs)
%             obj.CheckDanger(Object) 

%             if (obj.flagDanger == 0) && (leaderFlagDanger == 0) %&& (leaderFlagComs == 1) && (obj.flagComsAvailable == 1)
%                 obj.Follow(state)
%             else
%                 obj.UpdateDA()
%             end

            if (leaderFlagComs == 1) && (obj.flagComsAvailable == 1)
                obj.Follow(state)
            else
                obj.UpdateDA()
            end
            obj.IncrementTimeStep
            obj.flagIPN = 0;
            
        end
    end

end