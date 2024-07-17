% .-------------------------------------------------------------.
% | Dynamic Autorouting Program for Multi-Agent Systems v.3.4   |
% | created by Komsun Tamanakijprasart and Dr.Sabyasachi Mondal |
% '-------------------------------------------------------------'

classdef GCS < handle
    properties (Access = public)
        numUAV (1,1) uint8
        rIPN (1,1) single = 6  % m
    end

    properties (Access = private)
        rt (1,1) single {mustBeNonnegative}
        flagDestinReached (1,1) uint8
        flagComsAvailable (1,1) uint8 
        tempUavStates (3,:) single
    end

    methods (Access = public)

        X = consensus_func(obj, X_nei, XL_pos_to_fol, XL_states, comms)
        XL_pos_to_fol = CreateFormation(obj, XL, no_uav, leaderAngle, option)
        function SetTimeStep(obj, x)
            obj.rt = x;
        end

        function val = GetFlagComs(obj)
            val = obj.flagComsAvailable;
        end

        function val = GetFlagDestin(obj)
            val = obj.flagDestinReached;
        end

        function SetFlagComs(obj, val)
            if val == 0 || val ==1
                obj.flagComsAvailable = val;
                disp("GCS: Setting flagComsAvailable to " + num2str(val) + " ...");
            else
                error("The flag value must be 0 or 1")
            end
        end

        function SetFlagDestin(obj, val)
            if val == 0 || val ==1
                obj.flagDestinReached= val;
                disp("GCS: Setting flagDestinAvailable to " + num2str(val) + " ...");
            else
                error("The flag value must be 0 or 1")
            end
        end

        function StoreTempUAVState(obj, uavID, state)
            if all(uavID >=1 & uavID <= obj.numUAV)
                state = reshape(state, [3,1]);
                obj.tempUavStates(:,uavID) = state;
            else
                error("The first argument (uavID) must be in the range of 1 to " + num2str(obj.numUAV))
            end
        end

        function [ipnIdx, closestIdx] = CheckIPN(obj, Xnei)
            % Xnei (nx3 matrix) 
            % index #1 is always the leader's position
            Xnei = reshape(Xnei, [], 3);
            n = size(Xnei, 1);

            % closest uav to the ith uav
            closestIdx = -1 * ones(1, n);
            rMin = inf * ones(1, n);

            for i = 1:n
                for j = 1:n
                    if i ~= j
                        dist = norm(Xnei(i,:) - Xnei(j,:));
                        if dist < rMin(i)
                            rMin(i) = dist;
                            closestIdx(i) = j;
                        end
                    end
                end
            end
            
            % Check which follower need to perform IPN
            ipnIdx = rMin <= obj.rIPN;

        end

    end

    methods (Access = private)
        
    end


end