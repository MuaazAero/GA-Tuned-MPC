clc; clear; close all;
format long;

tic % Start timer for computation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% MPC Controller for Quadcopter Trajectory Tracking %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% USER OPTIONS %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% slctr selects the reference trajectory (identical formulas to the LQI script):
%   slctr = 1 -> circular trajectory
%   slctr = 2 -> upward helix trajectory
%   slctr = 3 -> figure-eight trajectory
%   slctr = 4 -> upward spiral trajectory
%   slctr = 5 -> rose-petal trajectory
slctr = 1;                 % <-- CHANGE ONLY THIS VALUE FOR TRAJECTORY SELECTION

% torqueDistMode selects the external roll/pitch/yaw torque disturbance
% (identical amplitudes / frequencies / phases to the LQI script):
%   torqueDistMode = 1 -> no torque disturbance
%   torqueDistMode = 2 -> small  disturbance, 0.15 Nm peak amplitude
%   torqueDistMode = 3 -> medium disturbance, 0.30 Nm peak amplitude
%   torqueDistMode = 4 -> strong disturbance, 0.50 Nm peak amplitude
%
% IMPORTANT: this disturbance is injected ONLY into the roll/pitch/yaw
% rotational dynamics. It is NOT added to x, y, or z translational
% acceleration, and it is NOT told to the MPC. The controller must reject it
% through feedback, exactly like the LQI script.
torqueDistMode = 1;        % <-- CHANGE THIS VALUE FOR TORQUE DISTURBANCE TESTING

dt = 0.01;                 % Time step (s)
tf = 150;                   % Final time (s). Set tf = 150 to match the LQI duration
                           % (slower, because MPC solves a QP every step).

startFromGroundInitial = false; % false = start on the reference; true = start at ground/origin
enableWind = false;             % false = no x/y wind; true = add stochastic horizontal wind

enableDashboardFigure = true;   % true = one window with the 15 result tabs (same layout as the LQI script)
enableQuadAnimation   = true;   % true = play the 3D quadcopter animation inside tab 15

% ---- Controller configuration ----
% These are your GA-tuned values. The OV/MV/MVrate weights below were
% optimized for EXACTLY this configuration (hor_p = 12, hor_c = 4, frozen
% setpoint), so this is the best-performing default. The two experiment knobs
% below are OFF by default: turning them on changes the controller structure,
% which makes the existing GA weights mismatched and usually WORSENS tracking
% unless you re-tune the weights for the new structure.
hor_p = 12;                     % Prediction horizon (GA-tuned value)
hor_c = 4;                      % Control horizon (GA-tuned value)
useReferencePreview = false;    % EXPERIMENT: feed future reference over the horizon.
                                % Only helps if the weights are re-tuned for it.

% ---- Genetic Algorithm weight tuning 
% Set enableGATuning = true to search new weights with the GA before the
% simulation runs. Set it false to run quickly using the seed weights below.
% NOTE: the MPC fitness solves a QP at every step, so the GA is much slower
% than the LQI GA. Reduce gaTfinal / gaMaxSamples / gaMaxGenerations, or set
% gaUseParallel = true, to speed it up. Default is false for quick runs.
enableGATuning       = true;   % true = run GA before simulation, false = use seed weights below
saveOptimizedWeights = true;    % true = save the tuned weights to MPC_Results after the GA finishes
gaUseParallel        = true;   % true requires the Parallel Computing Toolbox

% GA speed controls. The main simulation uses tf seconds at dt; the GA uses a
% shorter/coarser internal simulation to keep tuning practical.
gaTfinal         = min(30, tf); % seconds used inside each GA fitness evaluation (LQI uses min(30,TfinalUser))
gaMaxSamples     = 3001;        % maximum samples used by each GA fitness evaluation (LQI uses 3001)
gaPopulationSize = 70;          % 30-60 is practical; increase for stronger optimization
gaMaxGenerations = 500;          % increase to 70-200 for final tuning (slow for MPC)

% Fitness term weights  Position
% tracking is dominant; the rest keep the solution practical and non-saturating.
fitW.rmsePos = 3.0;  fitW.rmseAtt = 0.8;
fitW.itaePos = 2.0;  fitW.itaeAtt = 0.5;
fitW.ssPos   = 2.0;  fitW.ssAtt   = 0.5;
fitW.effort  = 0.05; fitW.satPen  = 20.0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Platform Parameters %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m   = 1.25;    % Mass (kg)
g   = 9.81;    % Gravity (m/s^2)
Ixx = 0.0232;  % Roll inertia (kg m^2)
Iyy = 0.0232;  % Pitch inertia (kg m^2)
Izz = 0.0468;  % Yaw inertia (kg m^2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Trajectories %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Same analytic position/velocity formulas as trajectoryGenerateLQI in the
t = 0:dt:tf;      % Time array (s)
N = length(t);    % Number of time steps

% Shared trajectory constants (same as LQI script)
R       = 1;             % radius for circle / helix / figure-eight (m)
z_const = 1;             % constant altitude for circle / figure-eight (m)
wc      = 2*pi/5;        % circle angular rate (rad/s), period 5 s
wh      = 2*pi/10;       % helix angular rate (rad/s), period 10 s
w8      = 2*pi/10;       % figure-eight angular rate (rad/s), period 10 s
z_start = 0;             % helix start altitude (m)
z_end   = 5;             % helix end altitude (m)
vz      = (z_end - z_start)/tf;  % helix climb rate (m/s)

switch round(slctr)
    case 1  % Circular
        traj_x = R*sin(wc*t);        vel_x = R*wc*cos(wc*t);
        traj_y = R*cos(wc*t);        vel_y = -R*wc*sin(wc*t);
        traj_z = z_const*ones(1,N);  vel_z = zeros(1,N);
        trajName = 'Circular';

    case 2  % Upward helix
        traj_x = R*sin(wh*t);        vel_x = R*wh*cos(wh*t);
        traj_y = R*cos(wh*t);        vel_y = -R*wh*sin(wh*t);
        traj_z = z_start + vz*t;     vel_z = vz*ones(1,N);
        trajName = 'Upward Helix';

    case 3  % Figure-eight
        traj_x = R*sin(w8*t);        vel_x = R*w8*cos(w8*t);
        traj_y = R*sin(2*w8*t);      vel_y = 2*R*w8*cos(2*w8*t);
        traj_z = z_const*ones(1,N);  vel_z = zeros(1,N);
        trajName = 'Figure-Eight';

    case 4  % Upward spiral
        Rsp = 5; omega = 2*pi/(tf/3); linearRate = 0.75;
        traj_x = Rsp*cos(omega*t);   vel_x = -Rsp*omega*sin(omega*t);
        traj_y = Rsp*sin(omega*t);   vel_y =  Rsp*omega*cos(omega*t);
        traj_z = linearRate*t;       vel_z = linearRate*ones(1,N);
        trajName = 'Upward Spiral';

    case 5  % Rose-petal
        Rrose = 5; omega = 2*pi/tf; k = 2;
        ck = cos(k*omega*t); sk = sin(k*omega*t);
        c1 = cos(omega*t);   s1 = sin(omega*t);
        traj_x = Rrose*ck.*c1;
        traj_y = Rrose*ck.*s1;
        traj_z = (Rrose/4)*s1;
        vel_x  = Rrose*(-k*omega*sk.*c1 - omega*ck.*s1);
        vel_y  = Rrose*(-k*omega*sk.*s1 + omega*ck.*c1);
        vel_z  = (Rrose/4)*omega*c1;
        trajName = 'Rose-Petal';

    otherwise
        error('Unknown slctr = %g. Use 1..5.', slctr);
end

% Generate 12-state reference trajectory
% [x; dx; y; dy; z; dz; phi; dphi; theta; dtheta; psi; dpsi]
x_ref = zeros(12, N);
x_ref(1,:) = traj_x;
x_ref(2,:) = vel_x;
x_ref(3,:) = traj_y;
x_ref(4,:) = vel_y;
x_ref(5,:) = traj_z;
x_ref(6,:) = vel_z;
% Attitude references stay zero: the MPC computes the required roll/pitch
% internally from the position/velocity references (no attitude feedforward).

% Initial conditions
state = zeros(12,N); % Tracks state over time
if startFromGroundInitial
    state(:,1) = zeros(12,1);              % start at ground / origin
else
    state([1,3,5],1) = x_ref([1,3,5],1);   % start on the first reference point
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% External Roll/Pitch/Yaw Torque Disturbance %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Same 4-mode sinusoidal disturbance as the LQI script. A smooth ramp avoids
% an artificial impulse at t = 0. Injected later into dphi/dtheta/dpsi only.
switch round(torqueDistMode)
    case 1, tauAmp = 0.00; tauDesc = 'no torque disturbance';
    case 2, tauAmp = 0.15; tauDesc = 'small 0.15 Nm torque disturbance';
    case 3, tauAmp = 0.30; tauDesc = 'medium 0.30 Nm torque disturbance';
    case 4, tauAmp = 0.50; tauDesc = 'strong 0.50 Nm torque disturbance';
    otherwise
        warning('Unknown torqueDistMode = %g. Using mode 1 (no disturbance).', torqueDistMode);
        tauAmp = 0.00; tauDesc = 'no torque disturbance';
end

% Different frequencies/phases keep roll/pitch/yaw disturbances distinct.
freqRollHz  = 0.35; freqPitchHz = 0.45; freqYawHz = 0.25;
phaseRoll   = 0;    phasePitch  = pi/4; phaseYaw  = pi/2;
rampTime    = 2.0;

ramp = min(max(t/rampTime, 0), 1);
tau_dist_hist = zeros(3, N);   % [roll; pitch; yaw] disturbance torque (Nm)
tau_dist_hist(1,:) = ramp .* tauAmp .* sin(2*pi*freqRollHz  * t + phaseRoll);
tau_dist_hist(2,:) = ramp .* tauAmp .* sin(2*pi*freqPitchHz * t + phasePitch);
tau_dist_hist(3,:) = ramp .* tauAmp .* sin(2*pi*freqYawHz   * t + phaseYaw);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% WIND NOISE %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Horizontal wind, generated with exponential smoothing of Gaussian noise.
% Injected into xddot and yddot only when enableWind = true.
vel_mean = 5;            vel_std = 2;             % Wind speed mean / std (m/s)
ang_mean = deg2rad(25);  ang_std = deg2rad(5);    % Wind direction mean / std (rad)

Vel  = zeros(1,N);  Vel_x = zeros(1,N);  Vel_y = zeros(1,N);  Dir = zeros(1,N);
alpha = 0.99;  vel_noise = 0;  ang_noise = 0;

for h = 1:N
    vel_noise = alpha*vel_noise + sqrt(1 - alpha^2)*vel_std*randn();
    ang_noise = alpha*ang_noise + sqrt(1 - alpha^2)*ang_std*randn();
    vel_abs = vel_mean + vel_noise;
    ang_abs = ang_mean + ang_noise;
    Vel_x(h) = vel_abs*cos(ang_abs);
    Vel_y(h) = vel_abs*sin(ang_abs);
    Vel(h)   = vel_abs;
    Dir(h)   = ang_abs;
end

% Combine x/y wind and convert to acceleration through a simple drag model.
V = zeros(12,N);  V(1,:) = Vel_x;  V(3,:) = Vel_y;
Cd = 0.75;  areaRef = 0.1;  rho = 1.225;   % renamed area to avoid clashing with A-matrix
S = 0.5*rho*areaRef*Cd;
F_wind = S .* V.^2;         % Force due to wind
a_wind = F_wind ./ m;       % Acceleration due to wind (rows 1,3 hold x/y wind accel)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% STATE SPACE MODEL & PLANT %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = zeros(12);     % Initialize A-matrix
B = zeros(12, 4);  % Initialize B-matrix
C = eye(12);       % Full-state output (IMU and such assumed)
D = zeros(12,4);

% Populate A-matrix
A(1,2) = 1;   A(2,9)  = g;
A(3,4) = 1;   A(4,7)  = -g;
A(5,6) = 1;   A(7,8)  = 1;
A(9,10) = 1;  A(11,12) = 1;

% Populate B-matrix
B(6,1)  = 1/m;
B(8,2)  = 1/Ixx;
B(10,3) = 1/Iyy;
B(12,4) = 1/Izz;

% Create state space form and discrete plant
sys   = ss(A, B, C, D);
plant = c2d(sys, dt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% MPC PARAMETERS %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% hor_p (prediction horizon) and hor_c (control horizon) are set in USER OPTIONS.

mpcobj = mpc(plant, dt, hor_p, hor_c);

% Input operational limits (U1 is thrust deviation from hover m*g).
% Set BEFORE the weights so the GA fitness sees the same constrained controller.
mpcobj.MV(1).Min = -m*g;   mpcobj.MV(1).Max = 25 - m*g;   % Thrust (N)
mpcobj.MV(2).Min = -7;     mpcobj.MV(2).Max = 7;          % Roll torque (Nm)
mpcobj.MV(3).Min = -7;     mpcobj.MV(3).Max = 7;          % Pitch torque (Nm)
mpcobj.MV(4).Min = -7;     mpcobj.MV(4).Max = 7;          % Yaw torque (Nm)
mpcobj.Model.Nominal.U = [m*g; 0; 0; 0];                  % Nominal input (hover thrust)

% ---- Controller weights: GA-tuned or seed ----
% weights = [MV(4), MVrate(4), OV(12)]. seedWeights are used directly when
% enableGATuning = false, and as the GA starting point when it is true.
seedWeights = [0.192, 0.09, 0.16, 4.25, 0.1, 0.077, 0.2, 0.83, ...
               42.15, 200, 147, 115, 200, 22, 0.5, 8.05, 0.2, 5.125, 149, 189];

if enableGATuning
    gaOpts.tfinal         = gaTfinal;
    gaOpts.maxSamples     = gaMaxSamples;
    gaOpts.populationSize = gaPopulationSize;
    gaOpts.maxGenerations = gaMaxGenerations;
    gaOpts.useParallel    = gaUseParallel;
    gaOpts.seedWeights    = seedWeights;
    gaOpts.fitW           = fitW;

    fprintf('Running GA-MPC weight tuning on the %s trajectory (disturbance mode %d)...\n', ...
        trajName, round(torqueDistMode));
    [weights, gaBestCost, gaSeedCost] = optimizeMPCWeightsGA( ...
        A, B, mpcobj, x_ref, t, tau_dist_hist, dt, m, g, Ixx, Iyy, Izz, gaOpts);
    fprintf('GA-MPC tuning done. Seed cost %.6g -> best cost %.6g', gaSeedCost, gaBestCost);
    if gaBestCost < gaSeedCost
        fprintf(' (improved %.2f%%)\n', 100*(gaSeedCost - gaBestCost)/max(gaSeedCost, eps));
    else
        fprintf(' (no improvement; keeping seed weights)\n');
        weights = seedWeights;
    end

    if saveOptimizedWeights
        outDirGA = fullfile(pwd, 'MPC_Results');
        if ~exist(outDirGA, 'dir'); mkdir(outDirGA); end
        safeTrajGA = regexprep(char(string(trajName)), '[^\w]', '_');
        save(fullfile(outDirGA, sprintf('GA_MPC_best_weights_%s_dist%d.mat', safeTrajGA, round(torqueDistMode))), ...
             'weights', 'gaBestCost', 'gaSeedCost', 'seedWeights', 'slctr', 'torqueDistMode', 'hor_p', 'hor_c');
    end
else
    weights = seedWeights;   % quick run using the seed / previously tuned weights
end

mpcobj.Weights.ManipulatedVariables     = weights(1:4);   % Penalize input deviations
mpcobj.Weights.ManipulatedVariablesRate = weights(5:8);   % Penalize input rate of change
mpcobj.Weights.OutputVariables          = weights(9:20);  % Penalize state deviations

% Create mpc state object
mpc_state = mpcstate(mpcobj);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% INITIALIZE VARIABLES %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
U_hist = zeros(4,N);            % Tracks control inputs over time
U_hist(:,1) = [m*g; 0; 0; 0];   % Initialize control

e_x = zeros(1,N);  e_y = zeros(1,N);  e_z = zeros(1,N);  % Positional errors over time

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% SIMULATION %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:N-1

    % State error
    e = state(:,i) - x_ref(:,i);

    % MPC control input (the disturbance is NOT provided to the controller).
    % Reference previewing: hand the controller the upcoming reference samples
    % over the prediction horizon so it anticipates the trajectory instead of
    % chasing a frozen setpoint. This is what removes most of the tracking lag.
    if useReferencePreview
        previewIdx = min(i:(i + hor_p - 1), N); % future indices, clamped at trajectory end
        rSet = x_ref(:, previewIdx).';          % hor_p x 12 reference preview window
    else
        rSet = x_ref(:, i).';                   % single-point setpoint (original behavior)
    end
    U = mpcmove(mpcobj, mpc_state, state(:,i), rSet);

    % Track positional error over time for RMSE
    e_x(i) = e(1);  e_y(i) = e(3);  e_z(i) = e(5);

    % Linearized state change
    dx = A * state(:,i) + B * U;
    dx(6) = dx(6) - g; % gravity not captured by the linearized state-space form

    % External roll/pitch/yaw torque disturbance (rotational rows only).
    % Equivalent to the LQI linear injection dx += tau_dist / inertia.
    dx(8)  = dx(8)  + tau_dist_hist(1,i) / Ixx;
    dx(10) = dx(10) + tau_dist_hist(2,i) / Iyy;
    dx(12) = dx(12) + tau_dist_hist(3,i) / Izz;

    % Optional horizontal wind acceleration (xddot and yddot)
    if enableWind
        dx(2) = dx(2) + a_wind(1,i);
        dx(4) = dx(4) + a_wind(3,i);
    end

    % Updated state
    state(:,i+1) = state(:,i) + dx * dt;

    % Log inputs (add hover thrust back so U1 shows total thrust in plots)
    U(1) = U(1) + m*g;
    U_hist(:,i) = U;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Performance Calculation %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fill the final input sample so plots and metrics are clean at t = tf
U_hist(:,N) = U_hist(:,N-1);

compTime = toc; % End timer for computation
fprintf('\nComputation Time: %.3f seconds\n', compTime)
fprintf('Trajectory: %s\n', trajName)
fprintf('Torque disturbance: mode %d, %s\n', round(torqueDistMode), tauDesc)
fprintf('Wind enabled: %s\n', string(enableWind))

% Full error arrays over the whole run 
% NOTE: this MPC uses a zero attitude reference, so the roll/pitch/yaw "error"
% below is the actual attitude excursion (how far the quad tilts). This is the
% natural counterpart to the LQI attitude error for disturbance-rejection tests.
pos_error      = state([1,3,5],:)  - x_ref([1,3,5],:);
pos_error_norm = sqrt(sum(pos_error.^2, 1));
att_error      = state([7,9,11],:) - x_ref([7,9,11],:);
att_error_norm = sqrt(sum(att_error.^2, 1));

% Position RMSE
rmse_3D = sqrt(mean(pos_error_norm.^2));
rmse_x  = sqrt(mean(pos_error(1,:).^2));
rmse_y  = sqrt(mean(pos_error(2,:).^2));
rmse_z  = sqrt(mean(pos_error(3,:).^2));
mean_error = mean(pos_error_norm);
max_error  = max(pos_error_norm);

% Attitude RMSE
rmse_phi   = sqrt(mean(att_error(1,:).^2));
rmse_theta = sqrt(mean(att_error(2,:).^2));
rmse_psi   = sqrt(mean(att_error(3,:).^2));

% Control effort / energy
control_energy = trapz(t, sum(U_hist.^2, 1));

% ITAE normalized by integral(t dt), so reported units are m (position) or rad (attitude)
itaeWeight = trapz(t, t);
if itaeWeight <= eps, itaeWeight = 1; end
itaeEq_x     = trapz(t, t .* abs(pos_error(1,:))) / itaeWeight;
itaeEq_y     = trapz(t, t .* abs(pos_error(2,:))) / itaeWeight;
itaeEq_z     = trapz(t, t .* abs(pos_error(3,:))) / itaeWeight;
itaeEq_3D    = trapz(t, t .* pos_error_norm)      / itaeWeight;
itaeEq_phi   = trapz(t, t .* abs(att_error(1,:))) / itaeWeight;
itaeEq_theta = trapz(t, t .* abs(att_error(2,:))) / itaeWeight;
itaeEq_psi   = trapz(t, t .* abs(att_error(3,:))) / itaeWeight;

% Steady-state error window: last 10% of the run
ssStartIndex = max(1, floor(0.90 * N));
ssIdx = ssStartIndex:N;

% Actuator saturation: count samples where the MPC input sits at an actuator bound
satTol    = 1e-6;
thrustLim = [0, 25];   % total thrust bound (N), i.e. deviation bound plus hover
torqueLim = [-7, 7];   % roll/pitch/yaw torque bound (Nm)
sat_thrust = sum(U_hist(1,:) <= thrustLim(1)+satTol | U_hist(1,:) >= thrustLim(2)-satTol);
sat_roll   = sum(U_hist(2,:) <= torqueLim(1)+satTol | U_hist(2,:) >= torqueLim(2)-satTol);
sat_pitch  = sum(U_hist(3,:) <= torqueLim(1)+satTol | U_hist(3,:) >= torqueLim(2)-satTol);
sat_yaw    = sum(U_hist(4,:) <= torqueLim(1)+satTol | U_hist(4,:) >= torqueLim(2)-satTol);

fprintf('\nRoot Mean Squared Error for MPC Trajectory Tracker\n')
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('3-Dimensional RMSE: %.8f meters\n', rmse_3D)
fprintf('X-Position RMSE: %.8f meters\n', rmse_x)
fprintf('Y-Position RMSE: %.8f meters\n', rmse_y)
fprintf('Z-Position RMSE: %.8f meters\n', rmse_z)
fprintf('Mean 3D Position Error: %.8f meters\n', mean_error)
fprintf('Max 3D Position Error: %.8f meters\n', max_error)

fprintf('\nRoll/Pitch/Yaw Attitude RMSE for MPC Trajectory Tracker\n')
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('Roll Phi RMSE: %.8f rad\n', rmse_phi)
fprintf('Pitch Theta RMSE: %.8f rad\n', rmse_theta)
fprintf('Yaw Psi RMSE: %.8f rad\n', rmse_psi)

fprintf('\nControl Effort\n')
fprintf('- - - - - - - - - - - - - - -\n')
fprintf('Control Energy Integral: %.4f\n', control_energy)

fprintf('\nActuator Saturation Counts\n')
fprintf('- - - - - - - - - - - - - - -\n')
fprintf('Thrust saturation samples: %.0f\n', sat_thrust)
fprintf('Roll torque saturation samples: %.0f\n', sat_roll)
fprintf('Pitch torque saturation samples: %.0f\n', sat_pitch)
fprintf('Yaw torque saturation samples: %.0f\n', sat_yaw)

% ----- Report-ready summary tables (same layout as the LQI script) -----
metricSignal = {'X position'; 'Y position'; 'Z position'; '3D position'; ...
                'Roll phi'; 'Pitch theta'; 'Yaw psi'};
metricUnit   = {'m'; 'm'; 'm'; 'm'; 'rad'; 'rad'; 'rad'};

metricRMSE = [rmse_x; rmse_y; rmse_z; rmse_3D; rmse_phi; rmse_theta; rmse_psi];
metricMeanAbs = [mean(abs(pos_error(1,:))); mean(abs(pos_error(2,:))); mean(abs(pos_error(3,:))); mean_error; ...
                 mean(abs(att_error(1,:))); mean(abs(att_error(2,:))); mean(abs(att_error(3,:)))];
metricMaxAbs = [max(abs(pos_error(1,:))); max(abs(pos_error(2,:))); max(abs(pos_error(3,:))); max_error; ...
                max(abs(att_error(1,:))); max(abs(att_error(2,:))); max(abs(att_error(3,:)))];
metricITAE = [itaeEq_x; itaeEq_y; itaeEq_z; itaeEq_3D; itaeEq_phi; itaeEq_theta; itaeEq_psi];

metricFinalSSAbs = [abs(pos_error(1,end)); abs(pos_error(2,end)); abs(pos_error(3,end)); pos_error_norm(end); ...
                    abs(att_error(1,end)); abs(att_error(2,end)); abs(att_error(3,end))];
metricMeanSSAbs = [mean(abs(pos_error(1,ssIdx))); mean(abs(pos_error(2,ssIdx))); mean(abs(pos_error(3,ssIdx))); mean(pos_error_norm(ssIdx)); ...
                   mean(abs(att_error(1,ssIdx))); mean(abs(att_error(2,ssIdx))); mean(abs(att_error(3,ssIdx)))];
metricMaxSSAbs = [max(abs(pos_error(1,ssIdx))); max(abs(pos_error(2,ssIdx))); max(abs(pos_error(3,ssIdx))); max(pos_error_norm(ssIdx)); ...
                  max(abs(att_error(1,ssIdx))); max(abs(att_error(2,ssIdx))); max(abs(att_error(3,ssIdx)))];

rmseSummaryTable = table(metricSignal, metricUnit, metricRMSE, metricMeanAbs, metricMaxAbs, ...
    'VariableNames', {'Signal','Unit','RMSE','MeanAbsError','MaxAbsError'});
itaeSummaryTable = table(metricSignal, metricUnit, metricITAE, ...
    'VariableNames', {'Signal','Unit','ITAE'});
steadyStateErrorSummaryTable = table(metricSignal, metricUnit, metricFinalSSAbs, metricMeanSSAbs, metricMaxSSAbs, ...
    'VariableNames', {'Signal','Unit','FinalAbsSteadyStateError', ...
    'MeanAbsSteadyStateErrorLast10pct','MaxAbsSteadyStateErrorLast10pct'});

fprintf('\nRMSE Summary Table\n')
fprintf('- - - - - - - - - - - - - - -\n')
disp(rmseSummaryTable)

fprintf('\nITAE Summary Table - normalized to physical units m/rad\n')
fprintf('- - - - - - - - - - - - - - -\n')
disp(itaeSummaryTable)
fprintf('Note: Reported ITAE values are normalized by integral(t dt), so units are m for position and rad for attitude.\n')

fprintf('\nSteady-State Error Summary Table\n')
fprintf('- - - - - - - - - - - - - - -\n')
fprintf('Steady-state window: %.2f s to %.2f s\n', t(ssStartIndex), t(end))
disp(steadyStateErrorSummaryTable)

% Save tables to the base workspace for side-by-side comparison with the LQI run
assignin('base','rmseSummaryTable_MPC',rmseSummaryTable);
assignin('base','itaeSummaryTable_MPC',itaeSummaryTable);
assignin('base','steadyStateErrorSummaryTable_MPC',steadyStateErrorSummaryTable);

% Oscillation / smoothness analysis via path curvature
dx = gradient(state(1,:), t);
dy = gradient(state(3,:), t);
dz = gradient(state(5,:), t);
ddx = gradient(dx, t);
ddy = gradient(dy, t);
ddz = gradient(dz, t);
d  = [dx; dy; dz];
dd = [ddx; ddy; ddz];
vCrossa = cross(d', dd')';
K = sqrt(sum(vCrossa.^2, 1)) ./ (sqrt(dx.^2 + dy.^2 + dz.^2).^3);
[sharp_turns, ~] = findpeaks(K, 'MinPeakHeight', 1);
sharp_turns = length(sharp_turns);
fprintf('\n\nLocal Minima/Maxima Smoothness Analysis\n')
fprintf('- - - - - - - - - - - - - - -\n')
fprintf('Number of sharp turns: %.0f\n', sharp_turns)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Visualization %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Single-window, 15-tab dashboard with the SAME layout as the GA-tuned LQI
% script, so the two controllers can be reviewed side by side.
colors = [0.8500 0.3250 0.0980;   % 'o'
          0.4660 0.6740 0.1880;   % 'g'
          0.3010 0.7450 0.9330;   % 'c'
          0.4940 0.1840 0.5560];  % 'm'

% Dynamic trajectory labels for tab titles and legends
trajLabel = trajName;
trajTitle = sprintf('%s trajectory', upperFirst(trajLabel));

% Scalar steady-state metrics used by the summary tabs
ss_mean_abs_x  = mean(abs(pos_error(1,ssIdx)));
ss_mean_abs_y  = mean(abs(pos_error(2,ssIdx)));
ss_mean_abs_z  = mean(abs(pos_error(3,ssIdx)));
ss_mean_abs_3D = mean(pos_error_norm(ssIdx));
ss_max_abs_x   = max(abs(pos_error(1,ssIdx)));
ss_max_abs_y   = max(abs(pos_error(2,ssIdx)));
ss_max_abs_z   = max(abs(pos_error(3,ssIdx)));
ss_max_abs_3D  = max(pos_error_norm(ssIdx));

% Integral-action analog: cumulative integral of position tracking error.
% The MPC has no explicit integrator state, so this is the closest equivalent
% to the LQI integral states (it integrates ref - actual over time).
int_hist = cumtrapz(x_ref([1,3,5],:) - state([1,3,5],:), 2) * dt;

% Reference-minus-actual errors 
refMinusActualPos = -pos_error;
refMinusActualAtt = -att_error;

% Actuator limits for the control-input tab (this MPC has no separate raw
% command; the QP already enforces these bounds, so we overlay the limits)
thrustLimits = [0, 25];
torqueLimits = [-7, 7];

animationAxesForDashboard = [];
if enableDashboardFigure
    dashFig = figure('Name', sprintf('%s - MPC one-window segmented results', trajTitle), ...
        'NumberTitle','off', 'Color','w', 'Position',[80 80 1250 760]);
    tg = uitabgroup(dashFig);

    % Tab 1: X/Y/Z position tracking.
    tab = uitab(tg, 'Title','1 Position tracking');
    positionNames = {'X', 'Y', 'Z'};
    positionUnits = {'X (m)', 'Y (m)', 'Z (m)'};
    positionIdx = [1, 3, 5];
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, x_ref(positionIdx(kFig),:), 'r--', 'LineWidth', 1.3); hold(ax,'on');
        plot(ax, t, state(positionIdx(kFig),:), 'b', 'LineWidth', 1.3);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, positionUnits{kFig}); xlabel(ax,'Time (s)');
        title(ax, sprintf('%s Position Tracking', positionNames{kFig}));
        legend(ax, sprintf('%s reference', positionNames{kFig}), sprintf('%s actual', positionNames{kFig}), 'Location','best');
    end

    % Tab 2: position error.
    tab = uitab(tg, 'Title','2 Position error');
    errLabels = {'e_x (m)', 'e_y (m)', 'e_z (m)'};
    errTitles = {'X Tracking Error: x_{ref} - x', 'Y Tracking Error: y_{ref} - y', 'Z Tracking Error: z_{ref} - z'};
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, refMinusActualPos(kFig,:), 'LineWidth', 1.3);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, errLabels{kFig}); xlabel(ax,'Time (s)');
        title(ax, errTitles{kFig});
    end

    % Tab 3: attitude tracking. This MPC tracks attitude to a zero reference
    % (no attitude feedforward), so the reference line is flat at 0.
    tab = uitab(tg, 'Title','3 Attitude tracking');
    attNames = {'Roll', 'Pitch', 'Yaw'};
    attSymbols = {'\phi', '\theta', '\psi'};
    attIdx = [7, 9, 11];
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, x_ref(attIdx(kFig),:), 'r--', 'LineWidth', 1.3); hold(ax,'on');
        plot(ax, t, state(attIdx(kFig),:), 'b', 'LineWidth', 1.3);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, sprintf('%s (rad)', attSymbols{kFig})); xlabel(ax,'Time (s)');
        title(ax, sprintf('%s Tracking (reference = 0)', attNames{kFig}));
        legend(ax, sprintf('%s reference', attNames{kFig}), sprintf('%s actual', attNames{kFig}), 'Location','best');
    end

    % Tab 4: attitude error (= actual attitude excursion for this MPC).
    tab = uitab(tg, 'Title','4 Attitude error');
    attErrLabels = {'e_\phi (rad)', 'e_\theta (rad)', 'e_\psi (rad)'};
    attErrTitles = {'Roll Error: \phi_{ref} - \phi', 'Pitch Error: \theta_{ref} - \theta', 'Yaw Error: \psi_{ref} - \psi'};
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, refMinusActualAtt(kFig,:), 'LineWidth', 1.3);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, attErrLabels{kFig}); xlabel(ax,'Time (s)');
        title(ax, attErrTitles{kFig});
    end

    % Tab 5: 3D trajectory tracking.
    tab = uitab(tg, 'Title','5 3D trajectory');
    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.86 0.78]);
    hRef3D = plot3(ax, x_ref(1,:), x_ref(3,:), x_ref(5,:), 'r--', 'LineWidth', 1.5); hold(ax,'on');
    hAct3D = plot3(ax, state(1,:), state(3,:), state(5,:), 'b', 'LineWidth', 1.8);
    hStart3D = plot3(ax, x_ref(1,1), x_ref(3,1), x_ref(5,1), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 7);
    hEnd3D = plot3(ax, state(1,end), state(3,end), state(5,end), 'mo', 'MarkerFaceColor','m', 'MarkerSize', 7);
    grid(ax,'on');
    applyVisibleAltitudeAxes(ax, state, x_ref, true);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
    legend(ax,[hRef3D hAct3D hStart3D hEnd3D], {'Reference trajectory','Quadcopter trajectory','Start reference','Final actual'}, 'Location','best');
    if max(abs(x_ref(5,:) - x_ref(5,1))) < 0.05
        title(ax, {sprintf('3D Trajectory Tracking, 3D RMSE = %.8f m', rmse_3D), ...
            sprintf('Reference altitude: z = %.2f m', x_ref(5,1))});
    else
        title(ax, sprintf('3D Trajectory Tracking, 3D RMSE = %.8f m', rmse_3D));
    end

    % Tab 6: top-view trajectory.
    tab = uitab(tg, 'Title','6 Top view');
    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.86 0.78]);
    hRefTop = plot(ax, x_ref(1,:), x_ref(3,:), 'r--', 'LineWidth', 1.5); hold(ax,'on');
    hActTop = plot(ax, state(1,:), state(3,:), 'b', 'LineWidth', 2);
    hStartTop = scatter(ax, x_ref(1,1), x_ref(3,1), 75, 'go', 'filled');
    hEndTop = scatter(ax, x_ref(1,N), x_ref(3,N), 75, 'ro', 'filled');
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
    legend(ax, [hRefTop hActTop hStartTop hEndTop], {sprintf('Reference %s', trajTitle), 'Actual Path', 'Start', 'End'}, 'Location','best');
    grid(ax,'on'); axis(ax,'equal');
    title(ax, sprintf('Top View Tracking: %s', trajTitle));

    % Tab 7: RMSE and ITAE bars.
    tab = uitab(tg, 'Title','7 RMSE + ITAE');
    ax = axes('Parent',tab, 'Position',[0.08 0.58 0.38 0.32]);
    bar(ax, [rmse_x, rmse_y, rmse_z, rmse_3D]);
    set(ax, 'XTickLabel', {'X','Y','Z','3D'}); grid(ax,'on');
    ylabel(ax,'RMSE (m)'); title(ax,'Position RMSE');
    ax = axes('Parent',tab, 'Position',[0.56 0.58 0.36 0.32]);
    bar(ax, [rmse_phi, rmse_theta, rmse_psi]);
    set(ax, 'XTickLabel', {'Roll','Pitch','Yaw'}); grid(ax,'on');
    ylabel(ax,'RMSE (rad)'); title(ax,'Attitude RMSE');
    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.38 0.32]);
    bar(ax, [itaeEq_x, itaeEq_y, itaeEq_z, itaeEq_3D]);
    set(ax, 'XTickLabel', {'X','Y','Z','3D'}); grid(ax,'on');
    ylabel(ax,'ITAE (m)'); title(ax,'Position ITAE Error');
    ax = axes('Parent',tab, 'Position',[0.56 0.12 0.36 0.32]);
    bar(ax, [itaeEq_phi, itaeEq_theta, itaeEq_psi]);
    set(ax, 'XTickLabel', {'Roll','Pitch','Yaw'}); grid(ax,'on');
    ylabel(ax,'ITAE (rad)'); title(ax,'Attitude ITAE Error');

    % Tab 8: steady-state position summary bars.
    tab = uitab(tg, 'Title','8 Steady-state summary');
    ax = axes('Parent',tab, 'Position',[0.08 0.56 0.38 0.34]);
    bar(ax, [ss_mean_abs_x, ss_mean_abs_y, ss_mean_abs_z, ss_mean_abs_3D]);
    set(ax, 'XTickLabel', {'X','Y','Z','3D'}); grid(ax,'on');
    ylabel(ax,'Mean abs. SS error (m)');
    title(ax, sprintf('Mean Steady-State Error: %.2f s to %.2f s', t(ssStartIndex), t(end)));

    ax = axes('Parent',tab, 'Position',[0.56 0.56 0.36 0.34]);
    bar(ax, [ss_max_abs_x, ss_max_abs_y, ss_max_abs_z, ss_max_abs_3D]);
    set(ax, 'XTickLabel', {'X','Y','Z','3D'}); grid(ax,'on');
    ylabel(ax,'Max abs. SS error (m)');
    title(ax,'Maximum Absolute Steady-State Error');

    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.84 0.26]);
    bar(ax, [ss_mean_abs_3D, ss_max_abs_3D]);
    set(ax, 'XTickLabel', {'Mean abs. 3D','Max abs. 3D'}); grid(ax,'on');
    ylabel(ax,'3D SS error (m)');
    title(ax,'3D Position Steady-State Error Summary');

    % Tab 9: detailed steady-state position error window.
    tab = uitab(tg, 'Title','9 Detailed SS error');
    ssStatsMean = [ss_mean_abs_x, ss_mean_abs_y, ss_mean_abs_z];
    ssStatsMax = [ss_max_abs_x, ss_max_abs_y, ss_max_abs_z];
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, refMinusActualPos(kFig,:), 'b', 'LineWidth', 1.2); hold(ax,'on');
        plot(ax, t(ssIdx), refMinusActualPos(kFig,ssIdx), 'r', 'LineWidth', 1.5);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, errLabels{kFig}); xlabel(ax,'Time (s)');
        title(ax, sprintf('%s Error, SS mean abs = %.8f m, SS max abs = %.8f m', ...
            positionNames{kFig}, ssStatsMean(kFig), ssStatsMax(kFig)));
        legend(ax,'Full error','Steady-state window','Location','best');
    end

    % Tab 10: torque disturbance, one axis per direction.
    tab = uitab(tg, 'Title','10 Torque disturbance');
    disturbanceNames = {'Roll','Pitch','Yaw'};
    disturbanceLabels = {'d_\phi (N.m)', 'd_\theta (N.m)', 'd_\psi (N.m)'};
    for kFig = 1:3
        ax = axes('Parent',tab, 'Position',[0.08, 0.70-(kFig-1)*0.30, 0.86, 0.22]);
        plot(ax, t, tau_dist_hist(kFig,:), 'b', 'LineWidth', 1.3);
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        ylabel(ax, disturbanceLabels{kFig}); xlabel(ax,'Time (s)');
        title(ax, sprintf('Disturbance in %s Direction', disturbanceNames{kFig}));
        legend(ax,'Applied disturbance','Location','best');
    end

    % Tab 11: all six states in a grid.
    tab = uitab(tg, 'Title','11 States');
    stateLabels = {'x (m)', 'y (m)', 'z (m)', '\phi (rad)', '\theta (rad)', '\psi (rad)'};
    stateIndex = [1, 3, 5, 7, 9, 11];
    for j = 1:6
        row = ceil(j/2);
        col = mod(j-1,2) + 1;
        ax = axes('Parent',tab, 'Position',[0.08+(col-1)*0.46, 0.70-(row-1)*0.30, 0.38, 0.22]);
        plot(ax, t, state(stateIndex(j),:), 'b', 'LineWidth', 1.3); hold(ax,'on');
        plot(ax, t, x_ref(stateIndex(j),:), 'k--', 'LineWidth', 1.1);
        ylabel(ax, stateLabels{j}); xlabel(ax,'Time (s)');
        title(ax, stateLabels{j}); grid(ax,'on'); xlim(ax,[t(1), t(end)]);
        legend(ax,'Actual','Reference','Location','best');
    end

    % Tab 12: control inputs with actuator limits overlaid.
    tab = uitab(tg, 'Title','12 Control inputs');
    inputLabels = {'U1 - Total Thrust (N)', 'U2 - Roll Torque (Nm)', ...
                   'U3 - Pitch Torque (Nm)', 'U4 - Yaw Torque (Nm)'};
    inputLimits = {thrustLimits, torqueLimits, torqueLimits, torqueLimits};
    for kFig = 1:4
        row = ceil(kFig/2);
        col = mod(kFig-1,2) + 1;
        ax = axes('Parent',tab, 'Position',[0.08+(col-1)*0.46, 0.58-(row-1)*0.42, 0.38, 0.30]);
        plot(ax, t, U_hist(kFig,:), 'Color', colors(kFig,:), 'LineWidth', 1.4); hold(ax,'on');
        lim = inputLimits{kFig};
        yline(ax, lim(1), 'k:', 'LineWidth', 1.0);
        yline(ax, lim(2), 'k:', 'LineWidth', 1.0);
        xlabel(ax,'Time (s)'); ylabel(ax,inputLabels{kFig});
        title(ax,inputLabels{kFig}); legend(ax,'Applied command','Actuator limit','Location','best');
        grid(ax,'on'); xlim(ax,[t(1), t(end)]);
    end

    % Tab 13: combined external torque disturbance.
    tab = uitab(tg, 'Title','13 Torque combined');
    ax = axes('Parent',tab, 'Position',[0.08 0.14 0.86 0.74]);
    plot(ax, t, tau_dist_hist(1,:), 'LineWidth', 1.5); hold(ax,'on');
    plot(ax, t, tau_dist_hist(2,:), 'LineWidth', 1.5);
    plot(ax, t, tau_dist_hist(3,:), 'LineWidth', 1.5);
    yline(ax, tauAmp, 'k--', 'LineWidth', 1.0);
    yline(ax, -tauAmp, 'k--', 'LineWidth', 1.0);
    xlabel(ax,'Time (s)'); ylabel(ax,'External Torque Disturbance (Nm)');
    title(ax, sprintf('Roll/Pitch/Yaw External Torque Disturbance: mode %d, %s', ...
        round(torqueDistMode), tauDesc));
    legend(ax,'Roll disturbance','Pitch disturbance','Yaw disturbance','+Amplitude','-Amplitude','Location','best');
    grid(ax,'on'); xlim(ax,[t(1), t(end)]);
    if tauAmp > 0
        ylim(ax, 1.25 * [-tauAmp, tauAmp]);
    else
        ylim(ax, [-0.1, 0.1]);
    end

    % Tab 14: integral-action analog (cumulative position error).
    % The MPC has no explicit integrator/anti-windup clamp; input constraints
    % play that role. Both panels show the cumulative position error.
    tab = uitab(tg, 'Title','14 Integral error');
    ax = axes('Parent',tab, 'Position',[0.08 0.58 0.86 0.32]);
    plot(ax, t, int_hist(1,:), 'b', 'LineWidth', 2); hold(ax,'on');
    plot(ax, t, int_hist(2,:), 'Color', colors(2,:), 'LineWidth', 2);
    plot(ax, t, int_hist(3,:), 'Color', colors(1,:), 'LineWidth', 2);
    xlabel(ax,'Time (s)'); ylabel(ax,'Integral Position Error');
    title(ax, sprintf('Cumulative Position Error (integral-action analog) - Zoomed View: %s', trajTitle));
    legend(ax,'X','Y','Z','Location','best'); grid(ax,'on'); xlim(ax,[t(1), t(end)]);
    maxIntAbs = max(abs(int_hist(:)));
    if maxIntAbs < 1e-8
        yZoom = 1e-4;
    else
        yZoom = 1.25 * maxIntAbs;
    end
    ylim(ax, [-yZoom, yZoom]);

    ax = axes('Parent',tab, 'Position',[0.08 0.12 0.86 0.32]);
    plot(ax, t, int_hist(1,:), 'b', 'LineWidth', 1.6); hold(ax,'on');
    plot(ax, t, int_hist(2,:), 'Color', colors(2,:), 'LineWidth', 1.6);
    plot(ax, t, int_hist(3,:), 'Color', colors(1,:), 'LineWidth', 1.6);
    xlabel(ax,'Time (s)'); ylabel(ax,'Integral Position Error');
    title(ax,'Full View (MPC uses input constraints instead of an integral clamp)');
    legend(ax,'X','Y','Z','Location','best'); grid(ax,'on'); xlim(ax,[t(1), t(end)]);

    % Tab 15: 3D animation inside the same dashboard window.
    if enableQuadAnimation
        tab = uitab(tg, 'Title','15 3D animation');
        animationAxesForDashboard = axes('Parent',tab, 'Position',[0.05 0.08 0.90 0.86]);
        title(animationAxesForDashboard, '3D animation will play after dashboard setup');
        grid(animationAxesForDashboard,'on');
        tg.SelectedTab = tab;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% 3D QUADCOPTER ANIMATION %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Animation is drawn inside tab 15 of the one-window dashboard.
if enableQuadAnimation && ~isempty(animationAxesForDashboard) && isvalid(animationAxesForDashboard)
    anim.frameStep     = max(1, round(0.04 / dt)); % animation sampling step
    anim.armLength     = 0.22;                     % visual quad arm length (m)
    anim.propRadius    = 0.055;                    % visual propeller radius (m)
    anim.playbackSpeed = 2.0;                      % >1 faster, <1 slower
    animateQuadcopter3DInAxes(animationAxesForDashboard, state, x_ref, t, trajTitle, anim);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOCAL FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = upperFirst(txt)
%UPPERFIRST Capitalizes the first character of a char/string label.
    txt = char(txt);
    if isempty(txt)
        out = txt;
    else
        out = [upper(txt(1)), txt(2:end)];
    end
end

function applyVisibleAltitudeAxes(ax, state, x_ref, addHeightMarker)
%APPLYVISIBLEALTITUDEAXES Makes constant-altitude paths visibly 3-D.
% Circle and figure-eight trajectories use z_ref = 1 m. When all Z values
% are almost constant, normal axis equal scaling can visually flatten the
% path. This function forces a useful Z range and adds a vertical height
% marker so the 1 m altitude is clear.

    xyz_all = [state([1,3,5],:), x_ref([1,3,5],:)];
    minXYZ = min(xyz_all, [], 2);
    maxXYZ = max(xyz_all, [], 2);

    xyRange = max([maxXYZ(1)-minXYZ(1), maxXYZ(2)-minXYZ(2), 1e-6]);
    zRange = maxXYZ(3) - minXYZ(3);
    xyMargin = 0.18 * xyRange + 0.20;

    xlim(ax, [minXYZ(1)-xyMargin, maxXYZ(1)+xyMargin]);
    ylim(ax, [minXYZ(2)-xyMargin, maxXYZ(2)+xyMargin]);

    % If the path is nearly constant altitude, show ground/height clearly.
    if zRange < 0.05
        zCenter = mean(x_ref(5,:));
        zUpper = max(zCenter + 0.80, 1.60*zCenter);
        if zCenter <= 0.05
            zLower = minXYZ(3) - 0.25;
        else
            zLower = 0;
        end
        zlim(ax, [zLower, zUpper]);

        if addHeightMarker
            x0 = x_ref(1,1);
            y0 = x_ref(3,1);
            z0 = x_ref(5,1);
            plot3(ax, [x0 x0], [y0 y0], [zLower z0], 'k:', 'LineWidth', 1.4, ...
                'HandleVisibility','off');
        end
    else
        zMargin = 0.22 * max(zRange, 0.5) + 0.20;
        zlim(ax, [min(0, minXYZ(3)-zMargin), maxXYZ(3)+zMargin]);
    end

    view(ax, 42, 28);
    axis(ax, 'vis3d');
    grid(ax, 'on');
end

function animateQuadcopter3DInAxes(ax, state, x_ref, t, trajTitle, anim)
%ANIMATEQUADCOPTER3DINAXES Base-MATLAB 3D viewer drawn inside an existing axes.

    if nargin < 6 || isempty(anim)
        anim.frameStep = 20;
        anim.armLength = 0.45;
        anim.propRadius = 0.10;
        anim.playbackSpeed = 1.0;
    end

    N = numel(t);
    frameStep = max(1, round(anim.frameStep));
    armLength = anim.armLength;
    propRadius = anim.propRadius;
    playbackSpeed = max(anim.playbackSpeed, 0.01);

    % Body-frame geometry. The quadrotor is drawn in + configuration.
    armX_B = [-armLength, armLength; 0, 0; 0, 0];
    armY_B = [0, 0; -armLength, armLength; 0, 0];

    rotorCenters_B = [ armLength, -armLength, 0, 0;
                      0, 0, armLength, -armLength;
                      0, 0, 0, 0];

    thetaCircle = linspace(0, 2*pi, 45);
    rotorCircle_B = [propRadius * cos(thetaCircle);
                     propRadius * sin(thetaCircle);
                     zeros(size(thetaCircle))];

    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    applyVisibleAltitudeAxes(ax, state, x_ref, false);
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');

    % Full reference path and growing actual path.
    hRefAnim = plot3(ax, x_ref(1,:), x_ref(3,:), x_ref(5,:), 'r--', 'LineWidth', 1.2);

    % Height marker for constant-altitude trajectories (circle, figure-eight).
    altitudeNote = '';
    if max(abs(x_ref(5,:) - x_ref(5,1))) < 0.05
        zBase = max(0, min(x_ref(5,:)) - 1.0);
        plot3(ax, [x_ref(1,1) x_ref(1,1)], [x_ref(3,1) x_ref(3,1)], [zBase x_ref(5,1)], ...
            'k:', 'LineWidth', 1.2, 'HandleVisibility','off');
        altitudeNote = sprintf(' | reference altitude z = %.2f m', x_ref(5,1));
    end
    actualTrail = plot3(ax, state(1,1), state(3,1), state(5,1), 'b', 'LineWidth', 1.8);

    % Start and end markers.
    hStartAnim = scatter3(ax, x_ref(1,1), x_ref(3,1), x_ref(5,1), 70, 'go', 'filled');
    hEndAnim = scatter3(ax, x_ref(1,end), x_ref(3,end), x_ref(5,end), 70, 'ro', 'filled');

    % Quadcopter graphics handles.
    hArmX = plot3(ax, nan, nan, nan, 'k-', 'LineWidth', 4);
    hArmY = plot3(ax, nan, nan, nan, 'k-', 'LineWidth', 4);
    hBody = plot3(ax, nan, nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    hFront = plot3(ax, nan, nan, nan, 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 7);

    hRotors = gobjects(1,4);
    for r = 1:4
        hRotors(r) = plot3(ax, nan, nan, nan, 'LineWidth', 1.4);
    end

    legend(ax, [hRefAnim actualTrail hStartAnim hEndAnim hArmX hArmY hBody hFront], ...
                {'Reference trajectory', 'Actual trajectory', 'Start', 'End', ...
                'Quad arm X', 'Quad arm Y', 'Body', 'Front marker'}, ...
                'Location', 'best');

    title(ax, {sprintf('3D Quadcopter Animation Viewer: %s', trajTitle), ...
        sprintf('t = 0.00 s | roll = 0.000 rad | pitch = 0.000 rad | yaw = 0.000 rad%s', altitudeNote)});

    fig = ancestor(ax, 'figure');
    lastClock = tic;
    for k = 1:frameStep:N
        if isempty(fig) || ~isvalid(fig) || ~isvalid(ax)
            break;
        end

        pos = [state(1,k); state(3,k); state(5,k)];
        phi = state(7,k);
        theta = state(9,k);
        psi = state(11,k);
        R = rotationZYX(phi, theta, psi);

        armX = R * armX_B + pos;
        armY = R * armY_B + pos;
        set(hArmX, 'XData', armX(1,:), 'YData', armX(2,:), 'ZData', armX(3,:));
        set(hArmY, 'XData', armY(1,:), 'YData', armY(2,:), 'ZData', armY(3,:));
        set(hBody, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

        frontPoint = R * [armLength; 0; 0] + pos;
        set(hFront, 'XData', frontPoint(1), 'YData', frontPoint(2), 'ZData', frontPoint(3));

        for r = 1:4
            center = R * rotorCenters_B(:,r) + pos;
            circle = R * rotorCircle_B + center;
            set(hRotors(r), 'XData', circle(1,:), 'YData', circle(2,:), 'ZData', circle(3,:));
        end

        set(actualTrail, 'XData', state(1,1:k), 'YData', state(3,1:k), 'ZData', state(5,1:k));
        title(ax, {sprintf('3D Quadcopter Animation Viewer: %s', trajTitle), ...
            sprintf('t = %.2f s | roll = %.3f rad | pitch = %.3f rad | yaw = %.3f rad%s', ...
            t(k), phi, theta, psi, altitudeNote)});

        drawnow limitrate;
        desiredDelay = frameStep * (t(2)-t(1)) / playbackSpeed;
        elapsed = toc(lastClock);
        if desiredDelay > elapsed
            pause(desiredDelay - elapsed);
        end
        lastClock = tic;
    end
end

function R = rotationZYX(phi, theta, psi)
%ROTATIONZYX Rotation matrix from body frame to inertial/world frame.
% Euler order: yaw psi about z, pitch theta about y, roll phi about x.
    cphi = cos(phi);     sphi = sin(phi);
    ctheta = cos(theta); stheta = sin(theta);
    cpsi = cos(psi);     spsi = sin(psi);

    Rz = [ cpsi, -spsi, 0;
           spsi,  cpsi, 0;
              0,     0, 1];

    Ry = [ ctheta, 0, stheta;
                0, 1,      0;
          -stheta, 0, ctheta];

    Rx = [1,     0,      0;
          0,  cphi, -sphi;
          0,  sphi,  cphi];

    R = Rz * Ry * Rx;
end

function [bestWeights, bestCost, seedCost] = optimizeMPCWeightsGA(A, B, mpcobjBase, x_ref_full, t_full, tau_full, dt, m, g, Ixx, Iyy, Izz, gaOpts)
%OPTIMIZEMPCWEIGHTSGA Genetic-algorithm tuning of the 20 MPC weights.
% Decision vector weights = [MV(4), MVrate(4), OV(12)]. The fitness simulates
% the SAME controller used in the main run (frozen setpoint, same limits, same
% torque disturbance) over a shorter time window (for speed), so the tuned
% weights transfer directly to the full simulation.

    if exist('ga','file') ~= 2
        error(['Global Optimization Toolbox function "ga" was not found. ' ...
               'Install/enable it, or set enableGATuning = false.']);
    end

    % Use the actual trajectory/disturbance, truncated to a shorter GA window.
    Nfull = numel(t_full);
    N_GA  = min([round(gaOpts.tfinal/dt) + 1, gaOpts.maxSamples, Nfull]);
    N_GA  = max(N_GA, 2);
    idx   = 1:N_GA;

    cfg.A = A;  cfg.B = B;  cfg.mpcobj = mpcobjBase;
    cfg.x_ref = x_ref_full(:, idx);
    cfg.t     = t_full(idx);
    cfg.tau_dist_hist = tau_full(:, idx);
    cfg.dt = dt;  cfg.N = N_GA;
    cfg.g = g;  cfg.Ixx = Ixx;  cfg.Iyy = Iyy;  cfg.Izz = Izz;
    cfg.limits.thrust = [-m*g, 25 - m*g];
    cfg.limits.torque = [-7, 7];
    cfg.fitW = gaOpts.fitW;

    vars = 20;
    lb = [0.01*ones(1,4), 0.001*ones(1,4), 0.1*ones(1,12)];
    ub = [10*ones(1,4),   1*ones(1,4),     200*ones(1,12)];

    seed = min(max(gaOpts.seedWeights(:)', lb), ub);
    popSize = max(gaOpts.populationSize, 2*vars);
    rng(7, 'twister');
    randomPop  = rand(popSize - 1, vars) .* (ub - lb) + lb;
    initialPop = [seed; randomPop];

    seedCost = mpcGAFitness(seed, cfg);

    opts = optimoptions('ga', ...
        'InitialPopulationMatrix', initialPop, ...
        'PopulationSize', popSize, ...
        'MaxGenerations', gaOpts.maxGenerations, ...
        'EliteCount', max(2, ceil(0.05*popSize)), ...
        'CrossoverFraction', 0.80, ...
        'FunctionTolerance', 1e-6, ...
        'MaxStallGenerations', max(15, ceil(0.35*gaOpts.maxGenerations)), ...
        'UseParallel', gaOpts.useParallel, ...
        'Display', 'iter');

    fitness = @(w) mpcGAFitness(w, cfg);
    [bestWeights, bestCost] = ga(fitness, vars, [], [], [], [], lb, ub, [], opts);
    bestWeights = bestWeights(:)';
end

function cost = mpcGAFitness(weights, cfg)
%MPCGAFITNESS Simulation-based multi-term fitness for GA-MPC tuning.
% Lower cost = better tracking with less aggressive input and less saturation.
% Blends position RMSE, attitude error, normalized ITAE, steady-state error,
% normalized control effort, an actuator-saturation penalty, and an
% instability guard - the same structure as the LQI GA fitness.

    weights = weights(:)';

    if numel(weights) ~= 20 || any(~isfinite(weights)) || ...
       any(weights(1:8) < 0) || any(weights(9:20) <= 0)
        cost = 1e12; return;
    end

    try
        mpcobj = cfg.mpcobj;
        mpcobj.Weights.ManipulatedVariables     = weights(1:4);
        mpcobj.Weights.ManipulatedVariablesRate = weights(5:8);
        mpcobj.Weights.OutputVariables          = weights(9:20);
        evalc('mpc_state = mpcstate(mpcobj);');   % suppress text
    catch
        cost = 1e12; return;
    end

    A = cfg.A; B = cfg.B; x_ref = cfg.x_ref; t = cfg.t; dt = cfg.dt; N = cfg.N;
    g = cfg.g; Ixx = cfg.Ixx; Iyy = cfg.Iyy; Izz = cfg.Izz;
    tau = cfg.tau_dist_hist; lim = cfg.limits; fw = cfg.fitW;

    state = zeros(12, N);
    state([1,3,5],1) = x_ref([1,3,5],1);   % start on the reference (matches main run)

    U_hist = zeros(4, N);
    unstablePenalty = 0;

    for i = 1:N-1
        try
            U = mpcmove(mpcobj, mpc_state, state(:,i), x_ref(:,i));  % frozen setpoint
        catch
            cost = 1e12; return;
        end

        dx = A * state(:,i) + B * U;
        dx(6)  = dx(6)  - g;
        dx(8)  = dx(8)  + tau(1,i) / Ixx;
        dx(10) = dx(10) + tau(2,i) / Iyy;
        dx(12) = dx(12) + tau(3,i) / Izz;

        state(:,i+1) = state(:,i) + dx * dt;
        U_hist(:,i) = U;

        if any(~isfinite(state(:,i+1))) || norm(state([1,3,5],i+1),2) > 200 || abs(state(5,i+1)) > 100
            unstablePenalty = 1e8 + 1e5 * i;
            state(:,i+1:end) = repmat(state(:,i), 1, N-i);
            break;
        end
    end
    U_hist(:,N) = U_hist(:, max(1, N-1));

    pos_error = state([1,3,5],:) - x_ref([1,3,5],:);
    pos_error_norm = sqrt(sum(pos_error.^2, 1));
    att_error = state([7,9,11],:) - x_ref([7,9,11],:);
    att_error_norm = sqrt(sum(att_error.^2, 1));

    rmsePos = sqrt(mean(pos_error_norm.^2));
    rmseAtt = sqrt(mean(att_error_norm.^2));

    itaeW = trapz(t, t);
    if itaeW <= eps, itaeW = 1; end
    itaePos = trapz(t, t .* pos_error_norm) / itaeW;
    itaeAtt = trapz(t, t .* att_error_norm) / itaeW;

    ssIdx = max(1, floor(0.90*N)):N;
    ssPos = mean(pos_error_norm(ssIdx));
    ssAtt = mean(att_error_norm(ssIdx));

    uNorm = zeros(4, N);
    uNorm(1,:) = U_hist(1,:) / max(abs(lim.thrust));
    uNorm(2,:) = U_hist(2,:) / max(abs(lim.torque));
    uNorm(3,:) = U_hist(3,:) / max(abs(lim.torque));
    uNorm(4,:) = U_hist(4,:) / max(abs(lim.torque));
    effort = trapz(t, sum(uNorm.^2, 1)) / max(t(end) - t(1), eps);

    tol = 1e-6;
    satCount = ...
        sum(U_hist(1,:) <= lim.thrust(1)+tol | U_hist(1,:) >= lim.thrust(2)-tol) + ...
        sum(U_hist(2,:) <= lim.torque(1)+tol | U_hist(2,:) >= lim.torque(2)-tol) + ...
        sum(U_hist(3,:) <= lim.torque(1)+tol | U_hist(3,:) >= lim.torque(2)-tol) + ...
        sum(U_hist(4,:) <= lim.torque(1)+tol | U_hist(4,:) >= lim.torque(2)-tol);
    saturationRatio = satCount / max(1, 4*(N-1));

    cost = fw.rmsePos*rmsePos + fw.rmseAtt*rmseAtt + ...
           fw.itaePos*itaePos + fw.itaeAtt*itaeAtt + ...
           fw.ssPos*ssPos     + fw.ssAtt*ssAtt     + ...
           fw.effort*effort   + fw.satPen*saturationRatio + unstablePenalty;

    if ~isfinite(cost)
        cost = 1e12;
    end
end
