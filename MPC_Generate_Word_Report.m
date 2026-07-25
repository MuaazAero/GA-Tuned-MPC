%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MPC_Generate_Word_Report.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Collects the MPC performance tables (RMSE, ITAE, steady-state, run summary)
% into ONE Word document. Tries, in order:
%   1) MATLAB Report Generator  -> .docx  (if that toolbox is installed)
%   2) Microsoft Word automation -> .docx (Windows with Word installed)
%   3) RTF fallback              -> .rtf  (opens in Word on any platform)
%
% HOW TO USE:
%   Put this file in the same folder as MPC_Final_04_Trajectory_Disturbance_Modes.m,
%   make that folder the MATLAB Current Folder, and run this script. If the
%   simulation has not been run yet, this script runs it automatically first.
%
% Do NOT put "clear" in this file - it needs the simulation variables.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Auto-run the simulation if its results are not already in the workspace
mainSimScript = 'MPC_Final_GA_Tuned.m';
if ~exist('rmse_3D','var') || ~exist('state','var') || ~exist('pos_error','var')
    if exist(mainSimScript, 'file') == 2
        fprintf('Simulation results not found in workspace - running %s first...\n', mainSimScript);
        run(mainSimScript);
    else
        error(['Simulation results are not in the workspace, and the main script "%s" ' ...
               'was not found in the current folder or on the MATLAB path.\n\n' ...
               'FIX: change into the folder that holds your MPC files, then run this ' ...
               'script again (it runs the simulation automatically), or run the main ' ...
               'simulation manually first in the same session.'], mainSimScript);
    end
end

%% Guard (safety net): confirm the simulation results are present
requiredVars = {'t','state','x_ref','pos_error','att_error','pos_error_norm', ...
    'rmse_x','rmse_y','rmse_z','rmse_3D','rmse_phi','rmse_theta','rmse_psi', ...
    'itaeEq_x','itaeEq_y','itaeEq_z','itaeEq_3D','itaeEq_phi','itaeEq_theta','itaeEq_psi', ...
    'ssIdx','ssStartIndex','mean_error','max_error','control_energy', ...
    'sat_thrust','sat_roll','sat_pitch','sat_yaw','sharp_turns','compTime', ...
    'trajName','torqueDistMode','tauDesc','tf','dt'};
missingVars = {};
for kv = 1:numel(requiredVars)
    if ~exist(requiredVars{kv}, 'var')
        missingVars{end+1} = requiredVars{kv}; %#ok<SAGROW>
    end
end
if ~isempty(missingVars)
    error('Missing workspace variables: %s', strjoin(missingVars, ', '));
end

%% Build the four tables (identical content to the CSV/Excel outputs)
signalNames = {'X position';'Y position';'Z position';'3D position'; ...
               'Roll phi';'Pitch theta';'Yaw psi'};
signalUnits = {'m';'m';'m';'m';'rad';'rad';'rad'};

RMSE = [rmse_x; rmse_y; rmse_z; rmse_3D; rmse_phi; rmse_theta; rmse_psi];
MeanAbsError = [mean(abs(pos_error(1,:))); mean(abs(pos_error(2,:))); mean(abs(pos_error(3,:))); mean_error; ...
                mean(abs(att_error(1,:))); mean(abs(att_error(2,:))); mean(abs(att_error(3,:)))];
MaxAbsError  = [max(abs(pos_error(1,:))); max(abs(pos_error(2,:))); max(abs(pos_error(3,:))); max_error; ...
                max(abs(att_error(1,:))); max(abs(att_error(2,:))); max(abs(att_error(3,:)))];
RMSE_Table = table(signalNames, signalUnits, RMSE, MeanAbsError, MaxAbsError, ...
    'VariableNames', {'Signal','Unit','RMSE','MeanAbsError','MaxAbsError'});

ITAE = [itaeEq_x; itaeEq_y; itaeEq_z; itaeEq_3D; itaeEq_phi; itaeEq_theta; itaeEq_psi];
ITAE_Table = table(signalNames, signalUnits, ITAE, ...
    'VariableNames', {'Signal','Unit','ITAE'});

FinalAbsSteadyState = [abs(pos_error(1,end)); abs(pos_error(2,end)); abs(pos_error(3,end)); pos_error_norm(end); ...
                       abs(att_error(1,end)); abs(att_error(2,end)); abs(att_error(3,end))];
MeanAbsSteadyState  = [mean(abs(pos_error(1,ssIdx))); mean(abs(pos_error(2,ssIdx))); mean(abs(pos_error(3,ssIdx))); mean(pos_error_norm(ssIdx)); ...
                       mean(abs(att_error(1,ssIdx))); mean(abs(att_error(2,ssIdx))); mean(abs(att_error(3,ssIdx)))];
MaxAbsSteadyState   = [max(abs(pos_error(1,ssIdx))); max(abs(pos_error(2,ssIdx))); max(abs(pos_error(3,ssIdx))); max(pos_error_norm(ssIdx)); ...
                       max(abs(att_error(1,ssIdx))); max(abs(att_error(2,ssIdx))); max(abs(att_error(3,ssIdx)))];
SteadyState_Table = table(signalNames, signalUnits, FinalAbsSteadyState, MeanAbsSteadyState, MaxAbsSteadyState, ...
    'VariableNames', {'Signal','Unit','FinalAbsSteadyState','MeanAbsSteadyState_last10pct','MaxAbsSteadyState_last10pct'});

sumMetric = {'Trajectory'; 'Disturbance mode'; 'Disturbance description'; ...
    'Simulation duration'; 'Time step'; ...
    '3D position RMSE'; 'Mean 3D position error'; 'Max 3D position error'; ...
    'Roll RMSE'; 'Pitch RMSE'; 'Yaw RMSE'; ...
    'Control energy integral'; ...
    'Thrust saturation samples'; 'Roll torque saturation samples'; ...
    'Pitch torque saturation samples'; 'Yaw torque saturation samples'; ...
    'Sharp turns'; 'Computation time'};
sumValue = {char(string(trajName)); num2str(round(torqueDistMode)); char(string(tauDesc)); ...
    num2str(tf); num2str(dt); ...
    num2str(rmse_3D, '%.8f'); num2str(mean_error, '%.8f'); num2str(max_error, '%.8f'); ...
    num2str(rmse_phi, '%.8f'); num2str(rmse_theta, '%.8f'); num2str(rmse_psi, '%.8f'); ...
    num2str(control_energy, '%.4f'); ...
    num2str(sat_thrust); num2str(sat_roll); num2str(sat_pitch); num2str(sat_yaw); ...
    num2str(sharp_turns); num2str(compTime, '%.3f')};
sumUnit = {'';'';''; 's';'s'; 'm';'m';'m'; 'rad';'rad';'rad'; '-'; ...
    'samples';'samples';'samples';'samples'; 'count'; 's'};
RunSummary_Table = table(sumMetric, sumValue, sumUnit, ...
    'VariableNames', {'Metric','Value','Unit'});

%% Assemble document content
outDir = fullfile(pwd, 'MPC_Results');
if ~exist(outDir, 'dir'); mkdir(outDir); end
safeTraj = regexprep(char(string(trajName)), '[^\w]', '_');
stamp    = sprintf('%s_dist%d', safeTraj, round(torqueDistMode));
docBase  = fullfile(outDir, sprintf('MPC_Report_%s', stamp));

titleStr = 'MPC Quadcopter Trajectory Tracking - Performance Report';
metaCell = { ...
    'Trajectory',          char(string(trajName)); ...
    'Disturbance',         sprintf('mode %d (%s)', round(torqueDistMode), char(string(tauDesc))); ...
    'Simulation duration', sprintf('%g s', tf); ...
    'Time step',           sprintf('%g s', dt); ...
    'Steady-state window', sprintf('%.2f s to %.2f s', t(ssStartIndex), t(end)); ...
    'Generated',           datestr(now) };

tables      = {RMSE_Table, ITAE_Table, SteadyState_Table, RunSummary_Table};
tableTitles = {'Root Mean Square Error (RMSE)', 'ITAE (normalized to m / rad)', ...
               'Steady-State Error (last 10% window)', 'Run Summary'};

%% Write the Word document (tiered: Report Generator -> Word COM -> RTF)
writtenFile = '';
methodUsed  = '';

if isempty(writtenFile)
    [ok, fout] = local_tryReportGenerator(docBase, titleStr, metaCell, tables, tableTitles);
    if ok, writtenFile = fout; methodUsed = 'MATLAB Report Generator (.docx)'; end
end
if isempty(writtenFile) && ispc
    [ok, fout] = local_tryWordCOM([docBase '.docx'], titleStr, metaCell, tables, tableTitles);
    if ok, writtenFile = fout; methodUsed = 'Microsoft Word automation (.docx)'; end
end
if isempty(writtenFile)
    fout = local_writeRTF([docBase '.rtf'], titleStr, metaCell, tables, tableTitles);
    writtenFile = fout; methodUsed = 'RTF (.rtf, opens in Microsoft Word)';
end

fprintf('\n==================== MPC WORD REPORT ====================\n');
fprintf('Method: %s\n', methodUsed);
fprintf('Saved : %s\n', writtenFile);
if endsWith(lower(writtenFile), '.rtf')
    fprintf('Tip   : the .rtf opens directly in Word. To convert to .docx,\n');
    fprintf('        use File > Save As > Word Document.\n');
end
fprintf('========================================================\n');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOCAL FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ok, fout] = local_tryReportGenerator(docBase, titleStr, metaCell, tables, tableTitles)
%LOCAL_TRYREPORTGENERATOR Uses the Report Generator toolbox if available.
    ok = false; fout = '';
    if isempty(which('mlreportgen.dom.Document'))
        return;
    end
    try
        import mlreportgen.dom.*
        d = Document(docBase, 'docx');
        open(d);

        append(d, Heading(1, titleStr));
        for i = 1:size(metaCell,1)
            append(d, Paragraph(sprintf('%s: %s', metaCell{i,1}, metaCell{i,2})));
        end
        append(d, Paragraph(' '));

        for ti = 1:numel(tables)
            append(d, Heading(2, tableTitles{ti}));
            T = tables{ti};
            headers = T.Properties.VariableNames;
            C = [headers; table2cellStr(T)];
            tbl = Table(C);
            tbl.Border = 'solid';
            tbl.ColSep = 'solid';
            tbl.RowSep = 'solid';
            try
                r1 = tbl.row(1);
                for c = 1:numel(headers)
                    r1.entry(c).Style = {Bold(true)};
                end
            catch
            end
            append(d, tbl);
            append(d, Paragraph(' '));
        end

        close(d);
        fout = d.OutputPath;
        ok = true;
    catch
        ok = false; fout = '';
    end
end

function [ok, fout] = local_tryWordCOM(docxFile, titleStr, metaCell, tables, tableTitles)
%LOCAL_TRYWORDCOM Builds a real .docx via Microsoft Word automation (Windows).
    ok = false; fout = '';
    word = [];
    try
        word = actxserver('Word.Application');
        word.Visible = false;
        doc = word.Documents.Add();

        localWordPara(doc, titleStr, true, 16);
        for i = 1:size(metaCell,1)
            localWordPara(doc, sprintf('%s: %s', metaCell{i,1}, metaCell{i,2}), false, 11);
        end
        localWordPara(doc, '', false, 11);

        for ti = 1:numel(tables)
            localWordPara(doc, tableTitles{ti}, true, 13);
            T = tables{ti};
            headers = T.Properties.VariableNames;
            body = table2cellStr(T);
            ncols = numel(headers);
            nrows = size(body,1) + 1;

            rng = doc.Content; rng.Collapse(0);        % 0 = wdCollapseEnd
            tbl = doc.Tables.Add(rng, nrows, ncols);
            tbl.Borders.Enable = 1;
            tbl.Range.Font.Size = 10;
            for c = 1:ncols
                hc = tbl.Cell(1, c);
                hc.Range.Text = headers{c};
                hc.Range.Font.Bold = 1;
            end
            for r = 1:size(body,1)
                for c = 1:ncols
                    tbl.Cell(r+1, c).Range.Text = body{r, c};
                end
            end
            localWordPara(doc, '', false, 11);
        end

        doc.SaveAs2(docxFile, 16);                     % 16 = wdFormatDocumentDefault (.docx)
        doc.Close(0);
        word.Quit();
        fout = docxFile;
        ok = true;
    catch
        ok = false; fout = '';
        try
            if ~isempty(word); word.Quit(); end
        catch
        end
    end
end

function localWordPara(doc, txt, bold, sizePt)
%LOCALWORDPARA Appends one styled paragraph at the end of a Word document.
    rng = doc.Content; rng.Collapse(0);
    if ~isempty(txt)
        rng.Text = txt;
        rng.Font.Bold = double(bold);
        rng.Font.Size = sizePt;
    end
    rng2 = doc.Content; rng2.Collapse(0);
    rng2.InsertParagraphAfter();
end

function fout = local_writeRTF(rtfFile, titleStr, metaCell, tables, tableTitles)
%LOCAL_WRITERTF Toolbox-free RTF writer; the resulting file opens in Word.
    fid = fopen(rtfFile, 'w');
    if fid == -1
        error('Could not open %s for writing.', rtfFile);
    end

    fprintf(fid, '{\\rtf1\\ansi\\ansicpg1252\\deff0\n');
    fprintf(fid, '{\\fonttbl{\\f0\\fswiss\\fcharset0 Calibri;}}\n');
    fprintf(fid, '\\viewkind4\\uc1\\pard\\f0\\fs22\n');

    fprintf(fid, '\\pard\\sa200\\b\\fs32 %s\\b0\\fs22\\par\n', rtfEscape(titleStr));
    for i = 1:size(metaCell,1)
        fprintf(fid, '\\pard\\sa40 {\\b %s:} %s\\par\n', rtfEscape(metaCell{i,1}), rtfEscape(metaCell{i,2}));
    end
    fprintf(fid, '\\pard\\sa120\\par\n');

    totalTwips = 9360;   % 6.5 inch usable text width
    for ti = 1:numel(tables)
        fprintf(fid, '\\pard\\sa120\\b\\fs26 %s\\b0\\fs22\\par\n', rtfEscape(tableTitles{ti}));

        T = tables{ti};
        headers = T.Properties.VariableNames;
        allRows = [headers; table2cellStr(T)];
        ncols = numel(headers);

        colChars = zeros(1, ncols);
        for c = 1:ncols
            m = 0;
            for r = 1:size(allRows,1)
                m = max(m, length(allRows{r,c}));
            end
            colChars(c) = max(m, 3);
        end
        colW = round(totalTwips * colChars / sum(colChars));
        bounds = cumsum(colW);
        bounds(end) = totalTwips;

        for r = 1:size(allRows,1)
            fprintf(fid, '\\trowd\\trgaph108\\trleft0\n');
            for c = 1:ncols
                fprintf(fid, ['\\clbrdrt\\brdrs\\brdrw10\\clbrdrl\\brdrs\\brdrw10' ...
                              '\\clbrdrb\\brdrs\\brdrw10\\clbrdrr\\brdrs\\brdrw10\\cellx%d\n'], bounds(c));
            end
            for c = 1:ncols
                if r == 1
                    fprintf(fid, '\\pard\\intbl\\b %s\\b0\\cell\n', rtfEscape(allRows{r,c}));
                else
                    fprintf(fid, '\\pard\\intbl %s\\cell\n', rtfEscape(allRows{r,c}));
                end
            end
            fprintf(fid, '\\row\n');
        end
        fprintf(fid, '\\pard\\sa120\\par\n');
    end

    fprintf(fid, '}\n');
    fclose(fid);
    fout = rtfFile;
end

function C = table2cellStr(T)
%TABLE2CELLSTR Converts a table's contents to a cell array of display strings.
    nr = height(T); nc = width(T);
    C = cell(nr, nc);
    for r = 1:nr
        for c = 1:nc
            v = T{r, c};
            if iscell(v); v = v{1}; end
            if isnumeric(v) || islogical(v)
                C{r, c} = fmtNum(double(v));
            else
                C{r, c} = char(string(v));
            end
        end
    end
end

function s = fmtNum(x)
%FMTNUM Compact numeric formatting (integers as integers, else 6 sig figs).
    if ~isfinite(x)
        s = num2str(x);
    elseif x == round(x) && abs(x) < 1e15
        s = sprintf('%d', round(x));
    else
        s = sprintf('%.6g', x);
    end
end

function s = rtfEscape(s)
%RTFESCAPE Escapes RTF special characters in a text string.
    s = char(s);
    s = strrep(s, '\', '\\');
    s = strrep(s, '{', '\{');
    s = strrep(s, '}', '\}');
end
