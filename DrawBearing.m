function DrawBearing(x, y, theta, len_Bearing, colorMap, style_Bearing)
%% --------------------------------------------------%
% --------------------------------------------------- input
% [x, y] - bearing starting from this pt
% theta - orientation of bearing
% len_Bearing - length of bearing to draw
% colorMap - color to draw with
% styleBearing - line style of bearing
% --------------------------------------------------- output
% ---------------------------------------------------

plot([x + len_Bearing * cos(theta), x], ...
     [y + len_Bearing * sin(theta), y], ...
     'Color', colorMap, ...
     'LineStyle', style_Bearing);

end