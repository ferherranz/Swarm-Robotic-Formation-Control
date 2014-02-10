function [du, w] = Obstacle(p, u, o, rad_Rob, u_Max, sensor_Range)
%% --------------------------------------------------%
% Obstacle - obstacle avoidance
% --------------------------------------------------- input
% p     - 1 x [x, y]        - position of itself
% u     - 1 x [vx, vy]      - velocity of itself
% o     - n x [x, y, siz]   - obs detected
%       - within sensing range & bearing(only those in front)
% rad_Rob
% u_Max
% sensor_Range
% --------------------------------------------------- output
% du    - [dvx, dvy] 
%       - velo turbulence under this rule
% w     - between [0, 1]
%       - 1 if < thres
% ---------------------------------------------------

if isempty(o)
    du = [0, 0];
    w  = 0;
    return;
else
    numO = size(o, 1);
    du = zeros(numO, 2);
    w  = zeros(numO, 1);
    
    for i = 1 : size(o, 1)
        theta = atan2(u(2), u(1));                      % bearing of the robot
        distRO = pdist2(p, o(i, 1 : 2));                % dist between center of r & o
        dist = DistPt2Ln_thetaP(o(i, 1 : 2), theta, p); % dist from o center to proceeding line of r
        r = o(i, 3) + rad_Rob + 10;                     % expand o with rad of r, and a redun 10
        if abs(dist) < r                                % possible to collide
            % ----------------------------------------- % get len
            len = sqrt(distRO^2 - dist^2) - sqrt(r^2 - dist^2);
            % ----------------------------------------- % get beta
            beta = asin(abs(dist)/r);
            if dist > 0                                 % obs is on the left of the robot
                beta = theta - (pi/2 - beta); 
            else                                        % obs is on the right of the robot
                beta = theta + (pi/2 - beta);
            end
%             hold on;
%             DrawBearing(p(1), p(2), beta, 100, 'r', '-');
            % ----------------------------------------- % get du(i, :) & w(i)
%             lenMax  = sqrt(distRO^2 - r^2);
%             lenMin  = 0;
            lenSafe = norm(u);
            du(i, :) = [cos(beta), sin(beta)];
            if len < lenSafe
                du(i, :) = [cos(beta), sin(beta)];
                w(i) = 1;
            else
                du(i, :) = [cos(beta), sin(beta)] - ...
                           u / norm(u);
                du(i, :) = du(i, :) / norm(du(i, :));
                w(i) = (r+sensor_Range-distRO) / (r+sensor_Range);
%                        (len-lenMax)^2 / (lenSafe-lenMax)^2 * (0.618) + ...
%                        (r - abs(dist)) / r * (1 - 0.618);
            end
        else
            w(i) = -1;
        end
    end

    du = du(w~=-1, :);
    w  =  w(w~=-1);
    
    if isempty(w)
        du = [0, 0];
        w  = 0;
    else
        if sum(w==1)
            du = mean(du(w==1, :), 1);
            w  = 1;
        else
            du = mean(du .* repmat(w ./ sum(w), 1, 2), 1);
            w  = mean(w);
        end
    end
end

end