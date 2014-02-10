function [data, obs] = Initialize(t, config)
%% --------------------------------------------------%
% initialize robots - no overlap
% --------------------------------------------------- input
% t      - t=1: initial time step
% config - num_Robots/siz_Ini/rad_Rob
% --------------------------------------------------- output
% data - data.p - T x (nR x 3)  mat  - [x, y]       - up to (t)
%      - data.u - T x (nR x 2)  mat  - [vx, vy]     - 
%      - data.z - T x (nR)      cell - n x [id]     - 
% ---------------------------------------------------

obs     = zeros(config.num_Obs, 3);                 % [x, y, siz]
obs(:, 3) = config.siz_Obs;

xMin = 0;
xMax = config.siz_Fig;
yMin = 0;
yMax = config.siz_Fig;

for i = 1 : config.num_Obs
    overlap = 1;
    while overlap
        x = rand() * (xMax - xMin - 2 * obs(i, 3)) + xMin + obs(i, 3);  % x
        y = rand() * (yMax - yMin - 2 * obs(i, 3)) + yMin + obs(i, 3);  % y
        if i == 1
            overlap = 0;
        else
            dist = pdist2([x, y], ...
                          obs(1 : (i-1), 1 : 2));
            if sum(dist' < obs(i, 3) + obs(1 : (i-1), 3) + 100)          % at least overlap with one
                overlap = 1;
            else
                overlap = 0;
            end
        end
    end
    obs(i, 1 : 2) = [x, y];
end


data.p  = zeros(config.T, config.num_Robots * config.dim_X);% T x (nR x [x, y])
data.u  = zeros(config.T, config.num_Robots * 2);           % T x (nR x [vx, vy])
data.z  =  cell(config.T, config.num_Robots * 1);           % T x (nR), entry - n x [id]
data.zO =  cell(config.T, config.num_Robots * 1);           % T x (nR), entry - n x [x, y, siz]

% [LC_Revise][Nov.26th][Use rand matrix instead]
% for i = 1 : config.num_Robots
%     data.u(t, (2 * (i-1) + 1) : (2 * (i))) = [rand() * 10, rand() * 10];
% end

rng('shuffle');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Let the initial velocity in [-10 10]
% PROBLEM: The whole swarm will not move. The swarm velocity will
% neutrulize to 0.
%data.u(t, :) = -10 + 20 * rand(1, config.num_Robots * 2);

% Let the initial velocity in [0 10]
% PROBLEM: The whole swarm will not move. The swarm velocity will
% neutrulize to 0.
data.u(t, :) = config.u_Ini*2 * rand(1, config.num_Robots * 2)-10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% region to initialize robots within
xMin = config.siz_Ini(1, 1);
xMax = config.siz_Ini(1, 2);
yMin = config.siz_Ini(2, 1);
yMax = config.siz_Ini(2, 2);

for i = 1 : config.num_Robots
    overlap = 1;
    while overlap
        x = rand() * (xMax - xMin) + xMin;          % x
        y = rand() * (yMax - yMin) + yMin;          % y
        dist = pdist2([x, y], obs(:, 1 : 2));
        if sum(dist' < obs(:, 3) + config.rad_Rob)  % at least overlap with one obs
            overlap = 1;
        else
            overlap = 0;
        end
    end
    data.p(t, (config.dim_X * (i - 1) + 1) : (config.dim_X * i)) = [x, y];
end

% rng('shuffle');
% x = rand(config.num_Robots, 1) .* (xMax - xMin) + xMin;
% y = rand(config.num_Robots, 1) .* (yMax - yMin) + yMin;
% initial_rnd_position = [x y];
% data.p(t, :) = reshape(initial_rnd_position, 1, numel(initial_rnd_position));

end