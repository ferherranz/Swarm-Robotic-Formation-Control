function Simulator()
%% --------------------------------------------------%
% main simulation
% --------------------------------------------------- input
% --------------------------------------------------- output
% ---------------------------------------------------
                                                    
%% -------------------------------------------------- set parameters
global config sensor form;
[config, sensor, form] = ConfigFile();

fig = figure(1);                                    % for the main simulator
set(fig, 'position', [675 105 700 700]);
hold on;

% --------------------------------------------------% *movie*
if config.movie == 1                                %
    winsize = get(fig, 'Position');                 %
    winsize(1 : 2) = [0 0];                         %
    numframes = 440;                                %
    A = moviein(numframes, fig, winsize);           %
    set(fig, 'NextPlot', 'replacechildren');        %
end                                                 %
% --------------------------------------------------%

%% -------------------------------------------------- initialize
t = 1;

% --------------------------------------------------% *movie*
if config.movie == 1                                %
    A(:, t) = getframe(fig, winsize);               %
end                                                 %                    
% --------------------------------------------------%
                                                    
global data obs;

[data, obs] = Initialize(t, config);
DrawWorld(data.p(t, :), data.u(t, :), obs, config, sensor);
grid on;
    
%% -------------------------------------------------- main loop
while t < config.T
    if config.pause
        disp('--------------------------');
        pause;     
    else
        pause(0.015);
    end
    t = t + 1;
    
    % ----------------------------------------------% *movie*
    if config.movie                                 %
        A(:, t) = getframe(fig, winsize);           %
    end                                             %
    % ----------------------------------------------%
    clf;
    
    [data, form] =  Control(t, data, config, sensor, form);
    data         =  Move(t, data, config);
    data         =  Sense(t, data, obs, config, sensor);
    DrawWorld(data.p(t, :), data.u(t, :), obs, config, sensor);
    grid on;
end


%% -------------------------------------------------- save
save log.mat ...
     config sensor form ...
     data obs...
     t
 
% --------------------------------------------------% *movie*
if config.movie                                     %
    save movie.mat A;                               %
    movie2avi(A, 'movie.avi');                      %
end                                                 %
% --------------------------------------------------%


end