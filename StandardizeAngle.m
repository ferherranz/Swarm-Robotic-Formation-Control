function theta = StandardizeAngle(theta)
%% --------------------------------------------------%
% bring any angle to [-180 180] deg
% --------------------------------------------------- input
% --------------------------------------------------- output
% ---------------------------------------------------

theta = rem(theta, 2 * pi);
theta(theta >   pi) = theta(theta >   pi) - 2 * pi;
theta(theta < - pi) = theta(theta < - pi) + 2 * pi;

end