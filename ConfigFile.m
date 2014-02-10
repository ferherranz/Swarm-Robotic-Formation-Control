function [config, sensor, form, obs] = ConfigFile()
%% --------------------------------------------------%
% --------------------------------------------------- input
% --------------------------------------------------- output
% config - parameters for robots
% sensor - parameters for sensors
% form   - parameters for formation control
% obs    - parameters for obs
% ---------------------------------------------------

%% --------------------------------------------------
%   General Parameters
% ---------------------------------------------------
scale                   = 1;

config.movie            = 0;                        % movie - 1 then make movie, 0 not
config.pause            = 0;

%% --------------------------------------------------
%   config - robots
% ---------------------------------------------------
config.T                = 10^3;                     % total time steps
config.num_Robots       = 50;                       % total number of robots

config.dim_X            = 2;                        % [x, y, theta]
config.dim_L            = 2;                        % [range, bearing]

% draw
config.siz_Fig          = 2000  * scale;            % size of the figure to draw
config.color_Map        = MakeColorMap([1 0 0], ... % red   [1 0 0]
                                       [0 1 0], ... % green [0 1 0]
                                       [0 0 1], ... % blue  [0 0 1]
                                       config.num_Robots);
config.style_Rob        = 'o';
config.color_Bearing    = 'r';
config.style_Bearing    = '-';
config.len_Bearing      = 20    * scale;

% region for initialization
config.siz_Ini          = [0, config.siz_Fig;%/2;     % initialize robots to be
                           0, config.siz_Fig];%/2];    % within this region
% size
config.rad_Rob          = 10    * scale;            %
% motion
config.u_Ini            = 12;
config.u_Min            = 10;
config.u_Max            = 15;
config.dt               = 1;

config.straight_Thres   = deg2rad(1);               % w - straight motion or not

% for obstacle
config.num_Obs          = 0;
config.siz_Obs          = 200*ones(config.num_Obs,1);%[20, 50, 80]';            % range of size of obs
config.color_Obs        = 'k';
config.style_Obs        = '--';
%% --------------------------------------------------
%   formation
% ---------------------------------------------------
form.id                 = 2;                        % what formation to use for the fleet: formation(formationID, :) 

%% --------------------------------------------------
%   sensor
% ---------------------------------------------------
sensor.n                = 3;                        % 1 - laser; 2 - radar; 3 - vision
sensor.range            = 200 * scale;            
sensor.bearing          = deg2rad(180); 
sensor.color            = 'r';                      % now useing robot-specific color/draw the sensing area
sensor.style            = '--';        

end

