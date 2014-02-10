function data = Move(t, data, config)
%% --------------------------------------------------%
% --------------------------------------------------- input
% t - current time step
% data - data.p - T x (nR x 3)  mat  - [x, y]       - up to (t-1)
%      - data.u - T x (nR x 2)  mat  - [vx, vy]     - up to (t)
%      - data.z - T x (nR)      cell - n x [id]     - up to (t-1)
% config - dim_X/alpha
% --------------------------------------------------- output
% data - data.p - T x (nR x 3)  mat  - [x, y]       - up to (t)
%      - data.u - T x (nR x 2)  mat  - [vx, vy]     - up to (t)
%      - data.z - T x (nR)      cell - n x [id]     - up to (t-1)
% ---------------------------------------------------

for i = 1 : config.num_Robots
    x     = data.p(t-1, config.dim_X * (i-1) + 1);
    y     = data.p(t-1, config.dim_X * (i-1) + 2);
    vx    = data.u(t,              2 * (i-1) + 1);
    vy    = data.u(t,              2 * (i-1) + 2);
    
    % keep within range
    x = mod(x + vx * config.dt, config.siz_Fig);
    y = mod(y + vy * config.dt, config.siz_Fig);
    
    data.p(t, (config.dim_X * (i-1) + 1) : (config.dim_X * (i))) = [x, y];
end

end