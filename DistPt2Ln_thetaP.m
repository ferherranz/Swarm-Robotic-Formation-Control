function dist = DistPt2Ln_thetaP(Pt, theta, Pt1)
%% --------------------------------------------------
% --------------------------------------------------- input
% Pt - [x, y]
% theta - 
% Pt1 - [x, y]
% --------------------------------------------------- output
% dist - distance between Pt to Line defined by 
%        slope k=tan(theta) and Pt1
%      - >0 if Pt is on the left side of the line
%      - <0 if on the right
% ---------------------------------------------------

theta1 = atan2(Pt(2) - Pt1(2), Pt(1) - Pt1(1));
relativeTheta = theta1 - theta;
dist = pdist2([Pt(1), Pt(2)], [Pt1(1), Pt1(2)]);
dist = dist * sin(relativeTheta);

end