function segway_ra()
% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'minVOverTime' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your system
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'minVOverTime' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;
% 8. Add random disturbance (white noise)
%     add the following code:
%     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];
addpath(genpath("/home/cbd/src/mit/helperOC"));
addpath(genpath("/home/cbd/src/helperOC"));

%% Should we compute the trajectory?
compTraj = true;

%% Should we load an HJI solution from a file?
loadHJI = false;

%% Grid
grid_min = [-1.5; -pi/2; -3; -pi]; % Lower corner of computation domain
grid_max = [1.5; pi/2; 3; pi];    % Upper corner of computation domain
% N = [51; 51; 51; 51];         % Number of grid points per dimension
N = [21; 21; 21; 21];
g = createGrid(grid_min, grid_max, N);

%% target set
R = 0.2;
data0 = shapeSphere(g, [1, 0, 0, 0], R);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 2.0;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% do dStep1 here
dMax = 0*[1, 1];

% control trying to min or max value function?
uMode = 'min';
% do dStep2 here
dMode = 'max';

%% Pack problem parameters

% Define dynamic system
dSegway = Segway([0, 0, 0, 0], dMax); %do dStep3 here


% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dSegway;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
%do dStep4 here
schemeData.dMode = dMode;

%% If you have obstacles, compute them here

%% Compute value function

%HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update

% uncomment if you want to see a 2D slice
HJIextraArgs.visualize.plotData.plotDims = [1 1 0 0]; %plot x, y
HJIextraArgs.visualize.plotData.projpt = [0 0]; %project at vx = vy = 0
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D
% HJIextraArgs.visualize.valueFunction = true;

plane_1 = shapeHyperplane(g, [1; 1; 0; 0], [0.1; 0; 0; 0]);
plane_2 = shapeHyperplane(g, [-1; -1; 0; 0], [-0.1; 0; 0; 0]);
plane_3 = shapeHyperplane(g, [1; -1; 0; 0], [0.5; -0.5; 0; 0]);
plane_4 = shapeHyperplane(g, [-1; 1; 0; 0], [-0.5; 0.5; 0; 0]);
keepout = shapeIntersection(plane_1, plane_2);
keepout = shapeIntersection(keepout, plane_3);
keepout = shapeIntersection(keepout, plane_4);
max_radius = shapeComplement(shapeSphere(g, [0; 0; 0; 0], 4.0));
unsafe_zone = shapeUnion(keepout, max_radius);
HJIextraArgs.obstacles = keepout;

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
% [data, tau2, ~] = ...
%   HJIPDE_solve(data0, tau, schemeData, 'none', HJIextraArgs);
if ~loadHJI
    [data, tau2, ~] = ...
      HJIPDE_solve(data0, tau, schemeData, 'minVOverTime', HJIextraArgs);
    save("segway_hji_solution.mat", "data", "tau2", "g", "-v7.3");
else
    load("segway_hji_solution.mat", "data", "tau2", "g");
end
%% Compute optimal trajectory from some initial state
if compTraj
  
  %set the initial state
  xinit = [1, -1.0, 0.0, 0.0];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    dSegway.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.dMode = dMode;
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,5);
    
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dSegway, TrajextraArgs);
  
%     figure(6)
%     clf
%     h = visSetIm(g, data(:,:,:,:,end));
% %     h.FaceAlpha = .3;
%     hold on
%     s = scatter3(xinit(1), xinit(2), xinit(3));
%     s.SizeData = 70;
%     title('The reachable set at the end and x_init')
%     hold off
  
    %plot traj
    figure(4)
    plot(traj(1,:), traj(2,:))
    hold on
%     xlim([-1 1])
%     ylim([-1 1])
    % add the target set to that
    [g2D, data2D] = proj(g, data0, [0 0 1 1]);
    visSetIm(g2D, data2D, 'green');
    title('2D projection of the trajectory & target set')
    hold off
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end
end