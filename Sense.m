function data = Sense(t, data, obs, config, sensor)
%% --------------------------------------------------%
% --------------------------------------------------- input
% t - current time step
% data - data.p - T x (nR x 3)  mat  - [x, y]       - up to (t)
%      - data.u - T x (nR x 2)  mat  - [vx, vy]     - up to (t)
%      - data.z - T x (nR)      cell - n x [id]     - up to (t-1)
%      - data.zO- T x (nR)      cell - n x [id]     - up to (t-1)
% obs  - n x [x, y, siz]
% config - num_Robots/
% sensor - 
% --------------------------------------------------- output
% data - data.p - T x (nR x 3)  mat  - [x, y]       - up to (t)
%      - data.u - T x (nR x 2)  mat  - [vx, vy]     - up to (t)
%      - data.z - T x (nR)      cell - n x [id]     - up to (t)
%      - data.zO- T x (nR)      cell - n x [id]     - up to (t)
% ---------------------------------------------------

data.p = real(data.p);
data.u = real(data.u);

X = data.p(t, 1 : config.dim_X : end-(config.dim_X-1))';
Y = data.p(t, 2 : config.dim_X : end-(config.dim_X-2))';
range = pdist2([X, Y], [X, Y]);

for i = 1 : config.num_Robots
    cx     = data.p(t, config.dim_X * (i-1) + 1);
    cy     = data.p(t, config.dim_X * (i-1) + 2);
    ctheta = atan2(data.u(t, 2 * (i-1) + 2), data.u(t, 2 * (i-1) + 1));

    % --------------------------------------------- % other robots
    rangeInd   = range(:, i) < sensor.range;        % ind with valid range
    
    if abs(sensor.bearing - 2*pi) >= 0.01
        bearing    = atan2(Y - cy, X - cx) - ctheta;
        bearing    = StandardizeAngle(bearing);
        bearingInd = abs(bearing) < sensor.bearing; % ind with valid bearing 

        ind = rangeInd .* bearingInd;
    else                                            % if can sense circly, just chech range
        ind = rangeInd;
    end
    
    ind(i) = 0;                                     % itself should not be included
    z = 1 : config.num_Robots;
    data.z{t, i} = z(ind == 1)';
    
    % --------------------------------------------- % obs
    dist = pdist2([cx, cy], obs(:, 1:2))';
    rangeInd = dist < sensor.range + obs(:, 3);     % ind with valid range
    
    bearing = atan2(obs(:,2)-cy, obs(:,1)-cx) - ctheta;
    bearing = StandardizeAngle(bearing);
    bearingInd = abs(bearing) < pi/2;%sensor.bearing;     % ind with valid bearing
    ind = rangeInd .* bearingInd;
    data.zO{t, i} = obs(ind==1, :);
end