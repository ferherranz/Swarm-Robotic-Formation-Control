function DrawCircle(cx, cy, r, color_Circle, style_Circle)
%% --------------------------------------------------
% --------------------------------------------------- input
% cx - n x 1 - center of circle
% cy - n x 1 - center of circle
% r - n x 1 - radius
% color_Circle
% style_Circle
% --------------------------------------------------- output
% ---------------------------------------------------

for i = 1 : size(cx, 1)
    th = linspace(0, 2 * pi, r(i) * 200);
    x = r(i) * cos(th) + cx(i);
    y = r(i) * sin(th) + cy(i);
    plot(x, y, ...
        'Color', color_Circle, ...
        'LineStyle', style_Circle);
    hold on;
end

end