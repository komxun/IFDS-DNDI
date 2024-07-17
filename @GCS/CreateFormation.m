% .-------------------------------------------------------------.
% | Dynamic Autorouting Program for Multi-Agent Systems v.3.4   |
% | created by Komsun Tamanakijprasart and Dr.Sabyasachi Mondal |
% '-------------------------------------------------------------'

function pos2Follow = CreateFormation(obj, XL, no_uav, leaderAngle, option)
    psi = leaderAngle(1);
    gamma = leaderAngle(2);
    B2E = rot3(-psi) * rot2(-gamma);  % Body-to-Earth
    E2B = inv(B2E);
    % XL = XL.';
    XL = E2B * XL';

    pos2Follow = zeros(3,5);
    switch option
        case 1 % Diagonal Line
            for i = 1:1:no_uav 
%                 pos2Follow(:,i) = [XL(1) - 2*(-1)^i*i*1 ; 
%                                    XL(2) + 4*(-1)^(i+1)*i*1; 
%                                    XL(3) + 3*(-1)^(i+1)*i*1];
                pos2Follow(:,i) = [XL(1) - 6*(-1)^i*i*1 ; 
                               XL(2) + 12*(-1)^(i+1)*i*1; 
                               XL(3) + 9*(-1)^(i+1)*i*1];
            end

        case 2  % Star
            % only for no_uav = 5
            d = 15;  % 15
            if no_uav == 5
                pos2Follow(:,1) = XL + [0; -d*cosd(18); +d*sind(18)];
                pos2Follow(:,2) = XL + [0; 0; +d];
                pos2Follow(:,3) = XL + [0; +d*cosd(18); +d*sind(18)];
                pos2Follow(:,4) = XL + [0; +d*cosd(54); -d*sind(54)];
                pos2Follow(:,5) = XL + [0; -d*cosd(54); -d*sind(54)];
                
            else
                error("Number of UAVs must be 5 for this formation")
            end
        case 3 % Triangle
            d = 30;
            if no_uav == 5
                pos2Follow(:,1) = XL + [0; 0; +d];
                pos2Follow(:,2) = XL + [0; -d/2; +d/2];
                pos2Follow(:,3) = XL + [0; +d/2; +d/2];
                pos2Follow(:,4) = XL + [0; -d; 0];
                pos2Follow(:,5) = XL + [0; +d; 0];

            else
                error("Number of UAVs must be 5 for this formation")
            end

        case 4 % Clock-wise Twisting star
            % only for no_uav = 5
            d = 20;
            if no_uav == 5
                pos2Follow(:,1) = XL' + [0; +d*sind(0 + obj.rt); +d*cosd(0 + obj.rt)];
                pos2Follow(:,2) = XL' + [0; +d*cosd(18 - obj.rt); +d*sind(18 - obj.rt)];
                pos2Follow(:,3) = XL' + [0; +d*cosd(54 + obj.rt); -d*sind(54 + obj.rt)];
                pos2Follow(:,4) = XL' + [0; -d*cosd(54 - obj.rt); -d*sind(54 - obj.rt)];
                pos2Follow(:,5) = XL' + [0; -d*cosd(18 + obj.rt); +d*sind(18 + obj.rt)];
            else
                error("Number of UAVs must be 5 for this formation")
            end
        case 5 % Clockwise-Twisting star (large radius)
            d = 60;
            if no_uav == 5
                pos2Follow(:,1) = XL' + [0; +d*sind(0 + obj.rt); +d*cosd(0 + obj.rt)];
                pos2Follow(:,2) = XL' + [0; +d*cosd(18 - obj.rt); +d*sind(18 - obj.rt)];
                pos2Follow(:,3) = XL' + [0; +d*cosd(54 + obj.rt); -d*sind(54 + obj.rt)];
                pos2Follow(:,4) = XL' + [0; -d*cosd(54 - obj.rt); -d*sind(54 - obj.rt)];
                pos2Follow(:,5) = XL' + [0; -d*cosd(18 + obj.rt); +d*sind(18 + obj.rt)];
            else
                error("Number of UAVs must be 5 for this formation")
            end
        case 6 % Horizontal Line with equal y-distance
            % d = 7;
%             d = 50;
%             d = 15;
            % d = 1000;
            d = 200;
            for i = 1:1:no_uav 
                pos2Follow(:,i) = [XL(1); 
                               XL(2) + d*(-1)^(i+1)*i*1; 
                               XL(3)];
            end
        case 7 % Planar V-shape
            d = 5;
            for i = 1:1:no_uav
                pos2Follow(:,i) = [XL(1) - d*i*sind(60);
                                   XL(2) + d*i*cosd(60)*(-1)^i;
                                   XL(3)];
            end
    end

    pos2Follow(:,1) = B2E*pos2Follow(:,1);
                pos2Follow(:,2) = B2E*pos2Follow(:,2);
                pos2Follow(:,3) = B2E*pos2Follow(:,3);
                pos2Follow(:,4) = B2E*pos2Follow(:,4);
                pos2Follow(:,5) = B2E*pos2Follow(:,5);

end


function out = rot3(x)
    out = [cos(x) sin(x) 0; -sin(x) cos(x) 0; 0 0 1;];
end

function out = rot2(x)
    out = [cos(x) 0 -sin(x); 0 1 0; sin(x) 0 cos(x)];
end

function out = rot1(x)
    out = [1 0 0; 0 cos(x) sin(x); 0 -sin(x) cos(x)];
end