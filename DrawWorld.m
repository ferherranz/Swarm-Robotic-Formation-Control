function DrawWorld(p, u, o, config, sensor)
%% --------------------------------------------------%
% --------------------------------------------------- input
% p - 1 x (config.num_Robots x config.dim_X) vec
% u - 1 x (config.num_Robots x [vx, vy]) vec
% 
% config - num_Robots/rad_Rob/
%          color_Map/style_Rob/
%          color_Bearing/style_Bearing
% sensor - 
% --------------------------------------------------- output
% ---------------------------------------------------

hold on;
DrawObs(o, config);
DrawRobots(p, u, config, sensor);

end

function DrawObs(o, config)

hold on;
if ~isempty(o)
    DrawCircle(o(:, 1), o(:, 2), o(:, 3), config.color_Obs, config.style_Obs);
end

end

function DrawRobots(p, u, config, sensor)
%% --------------------------------------------------%
% --------------------------------------------------- input
% p - 1 x (config.num_Robots x config.dim_X) vec
% u - 1 x (config.num_Robots x [vx, vy]) vec
% config - num_Robots/rad_Rob/
%          color_Map/style_Rob/
%          color_Bearing/style_Bearing
% sensor - 
% --------------------------------------------------- output
% ---------------------------------------------------

%% [LCrevise][Nov.26th][Use matrix calculation instead.]
P     = reshape(p, config.dim_X, config.num_Robots)';
U     = reshape(u, config.dim_X, config.num_Robots)';
X     = P(:, 1);
Y     = P(:, 2);
UX    = U(:, 1);
UY    = U(:, 2);

% This is much faster than n-times calculation.
Theta = atan2(UY, UX);

for i = 1 : config.num_Robots
%     x     = p(config.dim_X * (i-1) + 1);
%     y     = p(config.dim_X * (i-1) + 2);
%     theta = atan2(u(2 * (i-1) + 2), u(2 * (i-1) + 1));
    x = X(i);
    y = Y(i);
    theta = Theta(i);
    hold on;
    % [plot in a loop is too slow..]
    % [Can this be done by matrix operation?]
    plot(x, y, ...
         'Color', config.color_Map(i, :), ...
         'LineStyle', config.style_Rob);
%     DrawCircle(x, y, ...
%                config.rad_Rob, ...
%                config.color_Map(i, :), ...
%                config.style_Rob);
    DrawBearing(x, y, theta, ...
                config.len_Bearing, ...%sensor.range, ...%config.len_Bearing, ...
                config.color_Map(i, :), ...%config.color_Bearing, ...
                config.style_Bearing);
% %     if i==1%
%     if abs(sensor.bearing - pi) < 0.01
%         DrawCircle(x, y, ...
%                    sensor.range, ...
%                    config.color_Map(i, :), ...
%                    sensor.style);
%     else
%         DrawHalfCircle(x, y, theta, ...
%                        sensor.range, ...
%                        sensor.bearing, ...
%                        config.color_Map(i, :), ... %sensor.color, 
%                        sensor.style);
%     end
% %     end%
end
% axis([0, config.siz_Fig, 0, config.siz_Fig]);
% %%
% x     = p(1 : config.dim_X : end-1);
% y     = p(2 : config.dim_X : end);
% theta = atan2(x, y);
% plot(x, y, ...
%     'Color', config.color_Map(randi(config.num_Robots), :), ...
%     'LineStyle', config.style_Rob);
% % DrawBearing(x, y, theta, ...
% %             config.len_Bearing, ...
% %             config.color_Map(randi(config.num_Robots), :), ...
% %             config.style_Bearing);
% 

%%
% --------------------------------------------------- range of fig
% xMin = min(p(1 : config.dim_X : end));
% xMax = max(p(1 : config.dim_X : end));
% yMin = min(p(2 : config.dim_X : end));
% yMax = max(p(2 : config.dim_X : end));
% if xMax-xMin < config.siz_Fig - 2 * config.rad_Rob
%     x1 = (xMin+xMax)/2 - config.siz_Fig/2;
%     x2 = (xMin+xMax)/2 + config.siz_Fig/2;
% else
%     x1 = xMin - config.rad_Rob;
%     x2 = xMax + config.rad_Rob;
% end
% if yMax-yMin < config.siz_Fig - 2 * config.rad_Rob
%     y1 = (yMin+yMax)/2 - config.siz_Fig/2;
%     y2 = (yMin+yMax)/2 + config.siz_Fig/2;
% else
%     y1 = yMin - config.rad_Rob;
%     y2 = yMax + config.rad_Rob;
% end
axis([0, config.siz_Fig, 0, config.siz_Fig]);

end

function DrawHalfCircle(cx, cy, theta, r, angle, color_Sensor, style_Sensor)
%% --------------------------------------------------%
% --------------------------------------------------- input
% [cx, cy] - center of the circle
% theta - circle pointing at theta
% r - radius
% angle - half of the opening angle of the circle
% color_Sensor - color to draw
% style_Sensor - style to draw
% --------------------------------------------------- output
% ---------------------------------------------------

th = linspace(theta + angle, theta - angle, 100);
x = r * cos(th) + cx;
y = r * sin(th) + cy;
plot(x, y, 'Color', color_Sensor, 'LineStyle', style_Sensor);
plot([x(1), cx], [y(1), cy], 'Color', color_Sensor, 'LineStyle', style_Sensor);
plot([x(end), cx], [y(end), cy], 'Color', color_Sensor, 'LineStyle', style_Sensor);

end