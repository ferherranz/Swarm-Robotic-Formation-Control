function [data, form] = Control(t, data, config, sensor, form)
%% --------------------------------------------------%
% --------------------------------------------------- input
% t - current time step
% data - data.p - T x (nR x 3)  mat  - [x, y]       - up to (t-1)
%      - data.u - T x (nR x 2)  mat  - [vx, vy]     - up to (t-1)
%      - data.z - T x (nR)      cell - n x [id]     - up to (t-1)
%      - data.zO- T x (nR)      cell - n x [id]     - up to (t-1)
% config - num_Robots/dim_X
% sensor - range
% form - id
% --------------------------------------------------- output
% data - data.p - T x (nR x 3)  mat  - [x, y]       - up to (t-1)
%      - data.u - T x (nR x 2)  mat  - [vx, vy]     - up to (t)
%      - data.z - T x (nR)      cell - n x [id]     - up to (t-1)
%      - data.zO- T x (nR)      cell - n x [id]     - up tp (t-1)
% ---------------------------------------------------

if t == 2
    data.u(t, :) = data.u(t-1, :);                  % no move, haven't sensed
    return;
end

for i = 1 : config.num_Robots
    % --------------------------------------------- % at t, use z(t-1) to get u(t) then p(t)
    % --------------------------------------------- % obs detected
    o     = data.zO{t-1,i};                         % n x [x, y, siz]
    % --------------------------------------------- % of itself
    p     = data.p(t-1, (config.dim_X * (i-1) + 1) : ...
                        (config.dim_X * (i-1) + config.dim_X));
    u     = data.u(t-1, (2            * (i-1) + 1) : ...
                        (2            * (i-1) + 2));
    % --------------------------------------------- % of its neighbour
    z     = data.z{t-1, i};                       
    pN    = [data.p(t-1, config.dim_X * z - 1)' ...
             data.p(t-1, config.dim_X * z    )'];
    uN    = [data.u(t-1, 2            * z - 1)' ...
             data.u(t-1, 2            * z    )'];
    % --------------------------------------------- % of all except itself
    pA    = [data.p(t-1, 1 : config.dim_X : (end-1))' ...
             data.p(t-1, 2 : config.dim_X : (end  ))'];
    uA    = [data.u(t-1, 1 : 2 : (end-1))' ...
             data.u(t-1, 2 : 2 : (end  ))'];
    % --------------------------------------------- % for formation
    % --------------------------------------------- % rules
    numRules = 5;
    du    = zeros(numRules, 2);
    
    % Dynamic factors that determined by specific distribution in the
    % sensor range
    w     = zeros(numRules, 1);
    
    % Manually set scale factors.
    % It's a constant that makes du to a resonable scale.
    scale =  ones(numRules, 1);
    scale(1) = 5;
    scale(2) = 0;
    scale(3) = 1;
    scale(4) = 0.1;
    scale(5) = 5;
    
    [du(1, :), w(1)] = Obstacle(p, u, o, config.rad_Rob, sensor.range);
    [du(2, :), w(2)] = Separation(p, pN, u, uN, config.rad_Rob);
    [du(3, :), w(3)] = Alignment(u, uN);
    [du(4, :), w(4)] = Cohesion(p, pN);
    if form.id == 1
        [du(5, :)]       = I_Form(200, p, u, pN, uN);
        w(5) = 1;
    end
    if form.id == 2 && t == 3                       % leader for V formation
        pM   = [sum(pA(:, 1)), sum(pA(:, 2))] / size(pA, 1);
        dist = pdist2(pM, pA);
        [~, form.leader.id] = min(dist);
    end
    if form.id == 2
        form.leader.p = pA(form.leader.id, :);
        form.leader.theta = atan2(uA(form.leader.id, 2), uA(form.leader.id, 1));
    end
    if form.id == 2
        du(5, :) =   V_Form(20, p, uN, deg2rad(45), form.leader);
        w(5) = 1;
    end
    % --------------------------------------------- % update
    
    % Obstacle avoidance has the highest priority.
    % If the robot is too close to the obstacle, it won't care about
    % other rules and will only avoides the obstacle,
    % which could 'push' others away due to the seperation rule.
    if w(1) == 1
        u = du(1, :) * norm(u);
    else
        % Seperation rule has the second highest priority. If the robot 
        % is not too close to an obstacle but too close to
        % another robot, it won't care about the rest rules and will only
        % keep away from that robot before it can continue aligning and
        % cohering with others.
        if w(2) == 1
            u = du(2, :) * norm(u);
        else
            % 
            if w(2) * scale(2) < (1-0.618)
                w(2) = 1 - 0.618;
                scale(2) = 1;
            else
                %
                if w(2) * scale(2) > 1
                    w(2) = 1;
                    scale(2) = 1; 
                end
            end
            % univector of dynamically and constantly weighed average du after all rules
            du = mean(du .* repmat(w, 1, 2) .* repmat(scale, 1, 2), 1);
            
            % Only provide a small portion of current speed to change.
            % let new speed always has the same value as the old, only
            % change the direction
            norm_u = norm(u);
            u = 0.618* u + (1) * norm(u) * du;
            u = u / norm(u) * norm_u;
        end
    end
    
    % limit the speed within [u_Min u_Max]
    normU = norm(u);
    if normU > config.u_Max
        u = config.u_Max * u ./ normU;
    end
    if normU < config.u_Min && normU >0;
        u = config.u_Min * u ./ normU;
    end
    
    % update the speed
    data.u(t, (2 * (i-1) + 1) : (2 * (i))) = u;
end

end

function [du, w] = Obstacle(p, u, o, rad_Rob, sensor_Range)
%% --------------------------------------------------%
% Obstacle - obstacle avoidance
% --------------------------------------------------- input
% p     - 1 x [x, y]        - position of itself
% u     - 1 x [vx, vy]      - velocity of itself
% o     - n x [x, y, siz]   - obs detected
%       - within sensing range & bearing(only those in front)
% rad_Rob
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
            % ----------------------------------------- % get du(i, :) & w(i)
            lenSafe = norm(u);
            du(i, :) = [cos(beta), sin(beta)];
            if len < lenSafe
                % too close to the ith obstacle, weight to avoid this obstacle = 1
                du(i, :) = [cos(beta), sin(beta)];
                w(i) = 1;
            else
                % Not too close to the ith obstacle. So give a du to push 
                % current speed to the tagent speed which can lead the robot 
                % avoid the obstacle.
                du(i, :) = [cos(beta), sin(beta)] - u / norm(u);
                du(i, :) = du(i, :) / norm(du(i, :));
                
                % distRO will not exceed obstacle radius plus sensor range.
                % closer to the obstacle has higher weight (0~1).
                w(i) = (o(i,3)+sensor_Range-distRO) / (o(i,3)+sensor_Range);
            end
        else
            % will not collide, set the weight to -1 temporarily
            w(i) = -1;
        end
    end
    % filter out the conditions that will not collide
    du = du(w~=-1, :);
    w  =  w(w~=-1);
    
    % there is no obstacle that will collide in range.
    if isempty(w)
        du = [0, 0];
        w  = 0;
    else
        % if there exists at least one obstacle have to avoid urgently
        if sum(w==1)
            du = mean(du(w==1, :), 1);
            w  = 1;
        % there is no obstacle that has to avoid urgently
        else
            % weighted average of all du to avoid obstacles
            du = mean(du .* repmat(w ./ sum(w), 1, 2), 1);
            w  = mean(w);
        end
        % get the univector of du.
        du = du / norm(du);
    end
end

end

function [du, w] = Separation(p, pN, u, uN, rad_Rob)
%% --------------------------------------------------%
% Separation - avoid crowding local flockmates
% --------------------------------------------------- input
% p     - [x, y]
% pN    - n x [x, y]
% u     - [vx, vy]
% uN    - n x [vx, vy]
% rad_Rob
% --------------------------------------------------- output
% du    - [dvx, dvy] 
%       - velo turbulence under this rule
% w
% ---------------------------------------------------

if isempty(pN)
    du = [0, 0];
    w  = 0;
else
    numN = size(pN, 1);
    du = zeros(numN, 2);
    w  = zeros(numN, 1);
    
    tmpP = repmat(p, numN, 1);
    tmp = tmpP - pN;
    
    normT = zeros(numN, 1);
    for i = 1 : numN
        normT(i) = norm(tmp(i, :));
        du(i, :) = tmp(i, :) ./ (normT(i)^3);
        w(i) = 1 / (normT(i)^2);
    end
    du = sum(du) / norm(sum(du));
    if min(normT) < 2 * rad_Rob
        w = sum(w); %1;
    else
        w = sum(w);
    end
end

end

function [du, w] = Alignment(u, uN)
%% --------------------------------------------------%
% Alignment - steer towards the average heading of local flockmates
% --------------------------------------------------- input
% u     - [vx, vy]
% uN    - n x [vx, vy]
% --------------------------------------------------- output
% du    - [dvx, dvy] 
%       - velo turbulence under this rule
% w
% ---------------------------------------------------

du = [0, 0];
w = 0;
if ~isempty(uN)
    du = ([sum(uN(:, 1)) sum(uN(:, 2))] / size(uN, 1) - u);
    w  = (1-0.618);%norm(du);
    du = du ./ norm(du);
end
end

function [du, w] = Cohesion(p, pN)
%% --------------------------------------------------%
% Cohesion - steer to move toward the average position of local flockmates
% --------------------------------------------------- input
% p     - [x, y]
% pN    - n x [x, y]
% --------------------------------------------------- output
% du    - [dvx, dvy] 
%       - velo turbulence under this rule
% w
% ---------------------------------------------------

du = [0, 0];
w = 0;
if ~isempty(pN)
    du = ([sum(pN(:, 1)) sum(pN(:, 2))] / size(pN, 1) - p);
    w  = 1;%0.618;
    du = du ./ norm(du);
end
end

function du = I_Form(scale, p, u, pA, uA)
%% --------------------------------------------------%
% I_Form - proceed in line formation
% --------------------------------------------------- input
% scale - control the convergence speed
% p     - [x, y]
% u     - [vx, vy]
% pA    - n x [x, y]
% uA    - n x [vx, vy]
% --------------------------------------------------- output
% du    - [dvx, dvy] 
%       - velo turbulence under this rule
% ---------------------------------------------------

du = [0, 0];
if ~isempty(uA)
    uC = ([sum(uA(:, 1)) sum(uA(:, 2))] + u) / (size(uA, 1) + 1);   % motion of center
    pC = ([sum(pA(:, 1)) sum(pA(:, 2))] + p) / (size(pA, 1) + 1);   % position of center
    thetaC = atan2(uC(2), uC(1));
    hold on;
%     DrawBearing(pC(1), pC(2), thetaC, 30, 'r', '-');
    dist = DistPt2Ln_thetaP(p, thetaC, pC);         % >0 p on the left side of the line
    if dist < 0     % on the right
%         DrawBearing(p(1), p(2), thetaC+pi/2, -dist, 'b', '--');
        du = abs(dist) * [-sin(thetaC),  cos(thetaC)];
    else            % on the left
%         DrawBearing(p(1), p(2), thetaC-pi/2,  dist, 'b', '--');
        du = abs(dist) * [ sin(thetaC), -cos(thetaC)];
    end
    du = du / scale;
end

end

function du = V_Form(scale, p, uA, theta, leader)
%% --------------------------------------------------%
% V_Form - proceed in inverse V formation
% --------------------------------------------------- input
% scale - control the convergence speed
% p     - [x, y]
% uA    - n x [vx, vy]
% leader- leader.id
%       - leader.p
%       - leader.theta
% --------------------------------------------------- output
% du    - [dvx, dvy] 
%       - velo turbulence under this rule
% ---------------------------------------------------

du = [0, 0];
if ~isempty(uA)
    thetaC = leader.theta;                          % motion of center
    pC = leader.p;                                  % position of center
%     hold on;
%     DrawBearing(pC(1), pC(2), thetaC+pi, 30, 'r', '-');
%     DrawBearing(pC(1), pC(2), thetaC-theta+pi, 200, 'r', '-');
%     DrawBearing(pC(1), pC(2), thetaC+theta+pi, 200, 'r', '-');
    
    thetaL = thetaC - theta;
    thetaR = thetaC + theta;
    distL = DistPt2Ln_thetaP(p, thetaL, pC);        % >0 p on the left side of the line
    distR = DistPt2Ln_thetaP(p, thetaR, pC);        % >0 p on the left side of the line
    
    dist = DistPt2Ln_thetaP(p, thetaC, pC);
    if abs(dist) >= 0.01                            % not the leader
        if dist < 0                                 % should converge to the right line
            if distR < 0
%                 DrawBearing(p(1), p(2), thetaR+pi/2, -distR, 'b', '--');
                du = abs(distR) * [-sin(thetaR),  cos(thetaR)];
            else
%                 DrawBearing(p(1), p(2), thetaR-pi/2,  distR, 'b', '--');
                du = abs(distR) * [ sin(thetaR), -cos(thetaR)];
            end
        else
            if distL < 0
%                 DrawBearing(p(1), p(2), thetaL+pi/2, -distL, 'b', '--');
                du = abs(distL) * [-sin(thetaL),  cos(thetaL)];
            else
%                 DrawBearing(p(1), p(2), thetaL-pi/2,  distL, 'b', '--');
                du = abs(distL) * [ sin(thetaL), - cos(thetaL)];
            end
        end
    end  
    du = du / scale;
end

end