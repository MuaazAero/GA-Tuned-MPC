%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MPC_Export_Figures_PDF.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Re-creates the MPC result figures (the 14 analytical dashboard tabs, minus
% the live animation) as clean standalone pages and writes them to a single
% multi-page PDF. Falls back to one PDF per figure on older MATLAB releases
% that do not support appending pages.
%
% HOW TO USE:
%   1) Run MPC_Final_04_Trajectory_Disturbance_Modes.m first (same session).
%   2) Then run this script. It reads the results from the workspace.
%
% Do NOT put "clear" or "close all" in this file - it needs the simulation
% variables and must not close your dashboard window.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Auto-run the simulation if its results are not already in the workspace
% If you renamed your main simulation file, change the name below to match.
mainSimScript = 'MPC_Final_GA_Tuned.m';

if ~exist('rmse_3D','var') || ~exist('state','var') || ~exist('tau_dist_hist','var')
    if exist(mainSimScript, 'file') == 2
        fprintf('Simulation results not found in workspace - running %s first...\n', mainSimScript);
        run(mainSimScript);
    else
        error(['Simulation results are not in the workspace, and the main script "%s" ' ...
               'was not found in the current folder or on the MATLAB path.\n\n' ...
               'FIX: In MATLAB, change into the folder that holds your MPC files (or add ' ...
               'it to the path), then run this script again - it will run the simulation ' ...
               'automatically. Or run the main simulation manually first, then run this ' ...
               'script in the same session.'], mainSimScript);
    end
end

%% Guard (safety net): confirm the simulation results are present
requiredVars = {'t','N','state','x_ref','pos_error','att_error','pos_error_norm', ...
    'refMinusActualPos','refMinusActualAtt','int_hist','tau_dist_hist','U_hist', ...
    'rmse_x','rmse_y','rmse_z','rmse_3D','rmse_phi','rmse_theta','rmse_psi', ...
    'itaeEq_x','itaeEq_y','itaeEq_z','itaeEq_3D','itaeEq_phi','itaeEq_theta','itaeEq_psi', ...
    'ss_mean_abs_x','ss_mean_abs_y','ss_mean_abs_z','ss_mean_abs_3D', ...
    'ss_max_abs_x','ss_max_abs_y','ss_max_abs_z','ss_max_abs_3D', ...
    'ssIdx','ssStartIndex','tauAmp','tauDesc','torqueDistMode','trajName','trajTitle'};

missingVars = {};
for kv = 1:numel(requiredVars)
    if ~exist(requiredVars{kv}, 'var')
        missingVars{end+1} = requiredVars{kv}; %#ok<SAGROW>
    end
end
if ~isempty(missingVars)
    error(['Missing workspace variables: %s\n' ...
           'Run MPC_Final_04_Trajectory_Disturbance_Modes.m first (same MATLAB session), ' ...
           'then run this script.'], strjoin(missingVars, ', '));
end

%% Defensive defaults for a few plot helpers
if ~exist('colors','var')
    colors = [0.8500 0.3250 0.0980; 0.4660 0.6740 0.1880; 0.3010 0.7450 0.9330; 0.4940 0.1840 0.5560];
end
if ~exist('thrustLimits','var'); thrustLimits = [0, 25]; end
if ~exist('torqueLimits','var'); torqueLimits = [-7, 7]; end

%% Output location and file name
outDir = fullfile(pwd, 'MPC_Results');
if ~exist(outDir, 'dir'); mkdir(outDir); end
safeTraj = regexprep(char(string(trajName)), '[^\w]', '_');
stamp    = sprintf('%s_dist%d', safeTraj, round(torqueDistMode));
pdfFile  = fullfile(outDir, sprintf('MPC_Figures_%s.pdf', stamp));
if exist(pdfFile, 'file'); delete(pdfFile); end

%% Detect whether exportgraphics can append PDF pages (multi-page in one file)
haveExport = ~isempty(which('exportgraphics'));
supportsAppend = false;
if haveExport
    try
        capFig = figure('Visible','off','Color','w'); plot(1:2, 1:2);
        capPdf = fullfile(outDir, 'tmp_capability_test.pdf');
        if exist(capPdf,'file'); delete(capPdf); end
        exportgraphics(capFig, capPdf, 'ContentType','vector');
        exportgraphics(capFig, capPdf, 'Append', true, 'ContentType','vector');
        supportsAppend = true;
        close(capFig);
        if exist(capPdf,'file'); delete(capPdf); end
    catch
        supportsAppend = false;
        if exist('capFig','var') && isvalid(capFig); close(capFig); end
        if exist('capPdf','var') && exist(capPdf,'file'); delete(capPdf); end
    end
end

% Export state carried between pages
st.n = 0;
st.pdfFile = pdfFile;
st.outDir = outDir;
st.stamp = stamp;
st.haveExport = haveExport;
if supportsAppend
    st.mode = 'append';   % one multi-page PDF
else
    st.mode = 'perfile';  % one PDF per figure
end

posIdx = [1, 3, 5];
attIdx = [7, 9, 11];
positionUnits = {'X (m)', 'Y (m)', 'Z (m)'};
positionNames = {'X', 'Y', 'Z'};
attSymbols = {'\phi', '\theta', '\psi'};
attNames = {'Roll', 'Pitch', 'Yaw'};
errLabels = {'e_x (m)', 'e_y (m)', 'e_z (m)'};
attErrLabels = {'e_\phi (rad)', 'e_\theta (rad)', 'e_\psi (rad)'};

%% Page 1: Position tracking
fig = newPage();
for k = 1:3
    subplot(3,1,k);
    plot(t, x_ref(posIdx(k),:), 'r--', 'LineWidth', 1.3); hold on;
    plot(t, state(posIdx(k),:), 'b', 'LineWidth', 1.3);
    grid on; xlim([t(1) t(end)]);
    ylabel(positionUnits{k}); xlabel('Time (s)');
    title(sprintf('%s Position Tracking', positionNames{k}));
    legend(sprintf('%s reference', positionNames{k}), sprintf('%s actual', positionNames{k}), 'Location','best');
end
sgtitle(sprintf('Position Tracking - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 2: Position error
fig = newPage();
errTitles = {'X Tracking Error: x_{ref} - x', 'Y Tracking Error: y_{ref} - y', 'Z Tracking Error: z_{ref} - z'};
for k = 1:3
    subplot(3,1,k);
    plot(t, refMinusActualPos(k,:), 'LineWidth', 1.3);
    grid on; xlim([t(1) t(end)]);
    ylabel(errLabels{k}); xlabel('Time (s)');
    title(errTitles{k});
end
sgtitle(sprintf('Position Error - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 3: Attitude tracking (reference = 0 for this MPC)
fig = newPage();
for k = 1:3
    subplot(3,1,k);
    plot(t, x_ref(attIdx(k),:), 'r--', 'LineWidth', 1.3); hold on;
    plot(t, state(attIdx(k),:), 'b', 'LineWidth', 1.3);
    grid on; xlim([t(1) t(end)]);
    ylabel(sprintf('%s (rad)', attSymbols{k})); xlabel('Time (s)');
    title(sprintf('%s Tracking (reference = 0)', attNames{k}));
    legend(sprintf('%s reference', attNames{k}), sprintf('%s actual', attNames{k}), 'Location','best');
end
sgtitle(sprintf('Attitude Tracking - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 4: Attitude error
fig = newPage();
attErrTitles = {'Roll Error: \phi_{ref} - \phi', 'Pitch Error: \theta_{ref} - \theta', 'Yaw Error: \psi_{ref} - \psi'};
for k = 1:3
    subplot(3,1,k);
    plot(t, refMinusActualAtt(k,:), 'LineWidth', 1.3);
    grid on; xlim([t(1) t(end)]);
    ylabel(attErrLabels{k}); xlabel('Time (s)');
    title(attErrTitles{k});
end
sgtitle(sprintf('Attitude Error - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 5: 3D trajectory
fig = newPage();
ax = axes('Parent', fig); hold(ax, 'on');
hRef = plot3(ax, x_ref(1,:), x_ref(3,:), x_ref(5,:), 'r--', 'LineWidth', 1.5);
hAct = plot3(ax, state(1,:), state(3,:), state(5,:), 'b', 'LineWidth', 1.8);
hSt  = plot3(ax, x_ref(1,1), x_ref(3,1), x_ref(5,1), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 7);
hEn  = plot3(ax, state(1,end), state(3,end), state(5,end), 'mo', 'MarkerFaceColor','m', 'MarkerSize', 7);
grid(ax, 'on');
applyVisibleAltitudeAxes(ax, state, x_ref, true);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
legend(ax, [hRef hAct hSt hEn], {'Reference trajectory','Quadcopter trajectory','Start reference','Final actual'}, 'Location','best');
title(ax, sprintf('3D Trajectory Tracking, 3D RMSE = %.8f m - %s', rmse_3D, trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 6: Top view
fig = newPage();
plot(x_ref(1,:), x_ref(3,:), 'r--', 'LineWidth', 1.5); hold on;
plot(state(1,:), state(3,:), 'b', 'LineWidth', 2);
scatter(x_ref(1,1), x_ref(3,1), 75, 'go', 'filled');
scatter(x_ref(1,N), x_ref(3,N), 75, 'ro', 'filled');
xlabel('X (m)'); ylabel('Y (m)'); grid on; axis equal;
legend(sprintf('Reference %s', trajTitle), 'Actual Path', 'Start', 'End', 'Location','best');
title(sprintf('Top View Tracking - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 7: RMSE + ITAE bars
fig = newPage();
subplot(2,2,1); bar([rmse_x, rmse_y, rmse_z, rmse_3D]);
set(gca,'XTickLabel',{'X','Y','Z','3D'}); grid on; ylabel('RMSE (m)'); title('Position RMSE');
subplot(2,2,2); bar([rmse_phi, rmse_theta, rmse_psi]);
set(gca,'XTickLabel',{'Roll','Pitch','Yaw'}); grid on; ylabel('RMSE (rad)'); title('Attitude RMSE');
subplot(2,2,3); bar([itaeEq_x, itaeEq_y, itaeEq_z, itaeEq_3D]);
set(gca,'XTickLabel',{'X','Y','Z','3D'}); grid on; ylabel('ITAE (m)'); title('Position ITAE');
subplot(2,2,4); bar([itaeEq_phi, itaeEq_theta, itaeEq_psi]);
set(gca,'XTickLabel',{'Roll','Pitch','Yaw'}); grid on; ylabel('ITAE (rad)'); title('Attitude ITAE');
sgtitle(sprintf('RMSE and ITAE Summary - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 8: Steady-state summary bars
fig = newPage();
subplot(2,2,1); bar([ss_mean_abs_x, ss_mean_abs_y, ss_mean_abs_z, ss_mean_abs_3D]);
set(gca,'XTickLabel',{'X','Y','Z','3D'}); grid on; ylabel('Mean abs. SS error (m)');
title(sprintf('Mean SS Error: %.2f-%.2f s', t(ssStartIndex), t(end)));
subplot(2,2,2); bar([ss_max_abs_x, ss_max_abs_y, ss_max_abs_z, ss_max_abs_3D]);
set(gca,'XTickLabel',{'X','Y','Z','3D'}); grid on; ylabel('Max abs. SS error (m)');
title('Maximum Absolute SS Error');
subplot(2,2,[3 4]); bar([ss_mean_abs_3D, ss_max_abs_3D]);
set(gca,'XTickLabel',{'Mean abs. 3D','Max abs. 3D'}); grid on; ylabel('3D SS error (m)');
title('3D Position Steady-State Error Summary');
sgtitle(sprintf('Steady-State Error Summary - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 9: Detailed steady-state error window
fig = newPage();
ssMean = [ss_mean_abs_x, ss_mean_abs_y, ss_mean_abs_z];
ssMax  = [ss_max_abs_x, ss_max_abs_y, ss_max_abs_z];
for k = 1:3
    subplot(3,1,k);
    plot(t, refMinusActualPos(k,:), 'b', 'LineWidth', 1.2); hold on;
    plot(t(ssIdx), refMinusActualPos(k,ssIdx), 'r', 'LineWidth', 1.5);
    grid on; xlim([t(1) t(end)]);
    ylabel(errLabels{k}); xlabel('Time (s)');
    title(sprintf('%s Error, SS mean abs = %.8f m, SS max abs = %.8f m', positionNames{k}, ssMean(k), ssMax(k)));
    legend('Full error','Steady-state window','Location','best');
end
sgtitle(sprintf('Detailed Steady-State Error - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 10: Torque disturbance per axis
fig = newPage();
disturbanceNames = {'Roll','Pitch','Yaw'};
disturbanceLabels = {'d_\phi (N.m)', 'd_\theta (N.m)', 'd_\psi (N.m)'};
for k = 1:3
    subplot(3,1,k);
    plot(t, tau_dist_hist(k,:), 'b', 'LineWidth', 1.3);
    grid on; xlim([t(1) t(end)]);
    ylabel(disturbanceLabels{k}); xlabel('Time (s)');
    title(sprintf('Disturbance in %s Direction', disturbanceNames{k}));
end
sgtitle(sprintf('Torque Disturbance (per axis) - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 11: All six states
fig = newPage();
stateLabels = {'x (m)', 'y (m)', 'z (m)', '\phi (rad)', '\theta (rad)', '\psi (rad)'};
stateIndex = [1, 3, 5, 7, 9, 11];
for j = 1:6
    subplot(3,2,j);
    plot(t, state(stateIndex(j),:), 'b', 'LineWidth', 1.3); hold on;
    plot(t, x_ref(stateIndex(j),:), 'k--', 'LineWidth', 1.1);
    ylabel(stateLabels{j}); xlabel('Time (s)');
    title(stateLabels{j}); grid on; xlim([t(1) t(end)]);
    legend('Actual','Reference','Location','best');
end
sgtitle(sprintf('Quadcopter States - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 12: Control inputs with actuator limits
fig = newPage();
inputLabels = {'U1 - Total Thrust (N)', 'U2 - Roll Torque (Nm)', 'U3 - Pitch Torque (Nm)', 'U4 - Yaw Torque (Nm)'};
inputLimits = {thrustLimits, torqueLimits, torqueLimits, torqueLimits};
for k = 1:4
    subplot(2,2,k);
    plot(t, U_hist(k,:), 'Color', colors(k,:), 'LineWidth', 1.4); hold on;
    lim = inputLimits{k};
    yline(lim(1), 'k:', 'LineWidth', 1.0);
    yline(lim(2), 'k:', 'LineWidth', 1.0);
    xlabel('Time (s)'); ylabel(inputLabels{k}); title(inputLabels{k});
    legend('Applied command','Actuator limit','Location','best');
    grid on; xlim([t(1) t(end)]);
end
sgtitle(sprintf('Control Inputs - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Page 13: Combined torque disturbance
fig = newPage();
plot(t, tau_dist_hist(1,:), 'LineWidth', 1.5); hold on;
plot(t, tau_dist_hist(2,:), 'LineWidth', 1.5);
plot(t, tau_dist_hist(3,:), 'LineWidth', 1.5);
yline(tauAmp, 'k--', 'LineWidth', 1.0);
yline(-tauAmp, 'k--', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('External Torque Disturbance (Nm)'); grid on; xlim([t(1) t(end)]);
legend('Roll','Pitch','Yaw','+Amplitude','-Amplitude','Location','best');
title(sprintf('Combined Torque Disturbance: mode %d, %s', round(torqueDistMode), char(string(tauDesc))));
if tauAmp > 0
    ylim(1.25*[-tauAmp, tauAmp]);
else
    ylim([-0.1 0.1]);
end
st = exportPage(fig, st); close(fig);

%% Page 14: Integral-action analog (cumulative position error)
fig = newPage();
subplot(2,1,1);
plot(t, int_hist(1,:), 'b', 'LineWidth', 2); hold on;
plot(t, int_hist(2,:), 'Color', colors(2,:), 'LineWidth', 2);
plot(t, int_hist(3,:), 'Color', colors(1,:), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Integral Position Error'); grid on; xlim([t(1) t(end)]);
legend('X','Y','Z','Location','best');
title('Cumulative Position Error (integral-action analog) - Zoomed View');
maxIntAbs = max(abs(int_hist(:)));
if maxIntAbs < 1e-8, yZoom = 1e-4; else, yZoom = 1.25*maxIntAbs; end
ylim([-yZoom, yZoom]);
subplot(2,1,2);
plot(t, int_hist(1,:), 'b', 'LineWidth', 1.6); hold on;
plot(t, int_hist(2,:), 'Color', colors(2,:), 'LineWidth', 1.6);
plot(t, int_hist(3,:), 'Color', colors(1,:), 'LineWidth', 1.6);
xlabel('Time (s)'); ylabel('Integral Position Error'); grid on; xlim([t(1) t(end)]);
legend('X','Y','Z','Location','best');
title('Full View (MPC uses input constraints instead of an integral clamp)');
sgtitle(sprintf('Integral Error - %s', trajTitle));
st = exportPage(fig, st); close(fig);

%% Report where the PDF(s) were written
fprintf('\n==================== MPC FIGURE EXPORT ====================\n');
fprintf('Exported %d figure pages.\n', st.n);
if strcmp(st.mode, 'append')
    fprintf('Single multi-page PDF: %s\n', pdfFile);
else
    fprintf('One PDF per figure written to: %s\n', outDir);
    fprintf('(Your MATLAB release does not support appending PDF pages.)\n');
end
fprintf('==========================================================\n');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOCAL FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fig = newPage()
%NEWPAGE Creates an off-screen white figure sized for a PDF page.
    fig = figure('Visible','off', 'Color','w', 'InvertHardcopy','off', ...
        'Units','pixels', 'Position',[80 80 1000 760]);
end

function st = exportPage(fig, st)
%EXPORTPAGE Writes one figure as a PDF page (append mode) or its own PDF file.
    drawnow;
    st.n = st.n + 1;
    if strcmp(st.mode, 'append')
        if st.n == 1
            exportgraphics(fig, st.pdfFile, 'ContentType','vector');
        else
            exportgraphics(fig, st.pdfFile, 'Append', true, 'ContentType','vector');
        end
    else
        perFile = fullfile(st.outDir, sprintf('MPC_Fig_%02d_%s.pdf', st.n, st.stamp));
        if st.haveExport
            exportgraphics(fig, perFile, 'ContentType','vector');
        else
            set(fig, 'PaperOrientation','landscape');
            print(fig, perFile, '-dpdf', '-bestfit');
        end
    end
end

function applyVisibleAltitudeAxes(ax, state, x_ref, addHeightMarker)
%APPLYVISIBLEALTITUDEAXES Makes constant-altitude paths visibly 3-D.
    xyz_all = [state([1,3,5],:), x_ref([1,3,5],:)];
    minXYZ = min(xyz_all, [], 2);
    maxXYZ = max(xyz_all, [], 2);

    xyRange = max([maxXYZ(1)-minXYZ(1), maxXYZ(2)-minXYZ(2), 1e-6]);
    zRange = maxXYZ(3) - minXYZ(3);
    xyMargin = 0.18 * xyRange + 0.20;

    xlim(ax, [minXYZ(1)-xyMargin, maxXYZ(1)+xyMargin]);
    ylim(ax, [minXYZ(2)-xyMargin, maxXYZ(2)+xyMargin]);

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
            plot3(ax, [x_ref(1,1) x_ref(1,1)], [x_ref(3,1) x_ref(3,1)], [zLower x_ref(5,1)], ...
                'k:', 'LineWidth', 1.4, 'HandleVisibility','off');
        end
    else
        zMargin = 0.22 * max(zRange, 0.5) + 0.20;
        zlim(ax, [min(0, minXYZ(3)-zMargin), maxXYZ(3)+zMargin]);
    end

    view(ax, 42, 28);
    axis(ax, 'vis3d');
    grid(ax, 'on');
end
