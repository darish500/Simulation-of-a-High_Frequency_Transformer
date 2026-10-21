function export_report_figures(mode)
%EXPORT_REPORT_FIGURES  Save the report figures as PNG files in ./figures
%
%   export_report_figures(true)       also exports the Simulink diagrams (do this once)
%   export_report_figures             waveform data and plots only
%   export_report_figures('fromdata') rebuild the comparison figures from the two saved
%                                     data files (figdata_case1.mat, figdata_case2.mat),
%                                     without needing the simulation output
%
%   Workflow:
%       case_sel = 1;  inverter;  run the model;  export_report_figures(true)
%       case_sel = 2;  inverter;  run the model;  export_report_figures
%
%   This is a function, so it does not touch your workspace variables.
%   Needed logged signals: V_load, I_load, V_dc.   Optional: V_inv (for fig08).

model  = 'high_frequency_inverter';
outdir = fullfile(pwd, 'figures');
if ~isfolder(outdir), mkdir(outdir); end
fprintf('Saving figures to %s\n', outdir);

export_diagrams = false;
fromData = false;
if nargin >= 1
    if ischar(mode) || isstring(mode)
        fromData = strcmpi(mode, 'fromdata');
    else
        export_diagrams = logical(mode);
    end
end

%% ---------------- Rebuild the comparison figures from saved data ----------------
if fromData
    if ~isfile('figdata_case1.mat') || ~isfile('figdata_case2.mat')
        error('figdata_case1.mat and figdata_case2.mat must both exist in %s', pwd);
    end
    a = load('figdata_case1.mat'); a = a.D;
    b = load('figdata_case2.mat'); b = b.D;
    makeFigures(a, b, outdir);
    return
end

out      = evalin('base', 'out');
case_sel = evalin('base', 'case_sel');
Ts       = evalin('base', 'Ts_power');
f_out    = evalin('base', 'f_out');
Tstop    = evalin('base', 'Tstop');
fsw      = evalin('base', 'fsw');

%% ---------------- 1. Simulink diagrams (only needed once) ----------------
if export_diagrams
    load_system(model);
    items = { ...
        '',                         'fig02_model_overview'; ...
        'DC_Source',                'fig03_dc_source'; ...
        'DC_DC_Converter',          'fig04_flyback_subsystem'; ...
        'PWM_Generator',            'fig06_pwm_subsystem'; ...
        'H_Bridge_Inverter',        'fig07_hbridge_subsystem'; ...
        'T_LCL_Filter',             'fig09_filter_subsystem'; ...
        'HF_Transformer_1to1',      'fig11_transformer_subsystem'; ...
        'Load_and_PF_Measurement',  'fig12_load_subsystem'; ...
        'PI_Controller',            'fig13_pi_subsystem'};
    for k = 1:size(items, 1)
        sysPath = model;
        if ~isempty(items{k, 1}), sysPath = [model '/' items{k, 1}]; end
        try
            print(['-s' sysPath], '-dpng', '-r150', fullfile(outdir, [items{k, 2} '.png']));
            fprintf('Saved %s.png\n', items{k, 2});
        catch ME
            warning('Could not export %s: %s', items{k, 2}, ME.message);
        end
    end
end

%% ---------------- 2. Read the logged signals of this run ----------------
vlTs = getVals(out, {'V_load'}, true);
ilTs = getVals(out, {'I_load'}, true);
vdTs = getVals(out, {'V_dc'}, true);
viTs = getVals(out, {'V_inv', 'V_inv_Meas'}, false);     % optional

t   = vlTs.Time(:);
vl  = vec(vlTs.Data);
il  = interp1(ilTs.Time(:), vec(ilTs.Data), t, 'linear');
dt  = median(diff(t));

if abs(dt - Ts) / Ts > 0.01 || abs(t(end) - Tstop) > 10 * Ts
    error(['The logged data does not match case %d (time step %.3g s, end %.4g s).\n' ...
           'Run the MODEL again for this case, then call this function.'], ...
           case_sel, dt, t(end));
end

D = struct('caseNum', case_sel, 'fsw', fsw, 'f_out', f_out);

% --- DC-link voltage, decimated to about 20 us ---
stepD = max(1, round(20e-6 / dt));
D.tD  = vdTs.Time(1:stepD:end);
vdAll = vec(vdTs.Data);
D.vD  = vdAll(1:stepD:end);

% --- RMS voltage computed from the waveform (one-cycle trailing window) ---
stepR = max(1, round(10e-6 / dt));
tR    = t(1:stepR:end);
vR    = vl(1:stepR:end);
win   = max(1, round(1 / (f_out * stepR * dt)));
vrms  = sqrt(movmean(vR .^ 2, [win - 1 0]));
vrms(tR < tR(1) + 1 / f_out) = NaN;                 % first cycle is not meaningful
D.tR  = tR;
D.vrms = vrms;

% --- last three output cycles of load voltage and current ---
idx   = t >= t(end) - 3 / f_out;
D.t3  = (t(idx) - t(find(idx, 1))) * 1e3;            % ms
D.v3  = vl(idx);
D.i3  = il(idx);

% --- harmonic spectrum over the last 5 cycles (harmonics 2 to 50) ---
Nc = 5;
N  = round(Nc / (f_out * Ts));
tg = t(end) - N * Ts + (0:N - 1)' * Ts;
vg = interp1(t, vl, tg, 'linear');
Y  = fft(vg) / N;
A  = 2 * abs(Y(2:floor(N / 2)));
A1 = A(Nc);
hs = (2:50) * Nc;
D.h    = (2:50)';
D.Hpct = 100 * A(hs) / A1;
D.THD  = sqrt(sum(D.Hpct .^ 2));

% --- inverter output voltage near the crest (needs V_inv logged) ---
D.vinv_t = [];
D.vinv   = [];
if ~isempty(viTs)
    last   = t >= t(end) - 1 / f_out;
    tl     = t(last);
    [~, k] = max(vl(last));
    tc     = tl(k);
    w      = viTs.Time >= tc - 0.5e-3 & viTs.Time <= tc + 0.5e-3;
    viAll  = vec(viTs.Data);
    D.vinv_t = (viTs.Time(w) - tc) * 1e3;           % ms around the crest
    D.vinv   = viAll(w);
else
    fprintf('Note: V_inv is not logged, so fig08 will be skipped.\n');
end

save(sprintf('figdata_case%d.mat', case_sel), 'D');
fprintf('Saved figdata_case%d.mat  (THD 2-50 = %.2f %%)\n', case_sel, D.THD);

%% ---------------- 3. Comparison figures when both cases exist ----------------
if isfile('figdata_case1.mat') && isfile('figdata_case2.mat')
    a = load('figdata_case1.mat'); a = a.D;
    b = load('figdata_case2.mat'); b = b.D;
    makeFigures(a, b, outdir);
else
    fprintf('Run the other case as well to create the comparison figures.\n');
end
end

%% =====================================================================
function makeFigures(a, b, outdir)
lab1 = sprintf('%g kHz (conventional)', a.fsw / 1e3);
lab2 = sprintf('%g kHz (high-frequency)', b.fsw / 1e3);

jobs  = {@fig05, @fig08, @fig10, @fig14, @fig18, @fig19};
names = {'fig05', 'fig08', 'fig10', 'fig14', 'fig18', 'fig19'};
for k = 1:numel(jobs)
    fprintf('Creating %s ...\n', names{k});
    try
        jobs{k}(a, b, lab1, lab2, outdir);
    catch ME
        warning('%s failed: %s', names{k}, ME.message);
        close(findall(0, 'Type', 'figure', 'Visible', 'off'));
    end
end
fprintf('Done. Figures are in %s\n', outdir);
end

function fig05(a, b, lab1, lab2, outdir)
f = figure('Visible', 'off', 'Position', [100 100 820 380]);
plot(a.tD, a.vD, 'LineWidth', 1.1); hold on;
plot(b.tD, b.vD, 'LineWidth', 1.1);
grid on; xlabel('Time (s)'); ylabel('DC-link voltage (V)');
legend(lab1, lab2, 'Location', 'southeast');
title('DC-link voltage during start-up and steady state (1 kW load)');
saveFig(f, fullfile(outdir, 'fig05_flyback_startup.png'));
end

function fig08(a, b, lab1, lab2, outdir)
if isempty(a.vinv_t) || isempty(b.vinv_t)
    warning('fig08 skipped: V_inv was not logged in both runs.');
    return
end
f = figure('Visible', 'off', 'Position', [100 100 820 520]);
subplot(2, 1, 1); plot(a.vinv_t, a.vinv, 'LineWidth', 1);
grid on; ylabel('v_{AB} (V)'); title(['Bridge output voltage near the crest, ' lab1]);
subplot(2, 1, 2); plot(b.vinv_t, b.vinv, 'LineWidth', 1);
grid on; xlabel('Time around the crest (ms)'); ylabel('v_{AB} (V)');
title(['Bridge output voltage near the crest, ' lab2]);
saveFig(f, fullfile(outdir, 'fig08_vinv_zoom.png'));
end

function fig10(a, b, lab1, lab2, outdir)
f = figure('Visible', 'off', 'Position', [100 100 820 520]);
subplot(2, 1, 1); plot(a.t3, a.v3, 'LineWidth', 1.1);
grid on; ylim([-400 400]); ylabel('Load voltage (V)'); title(lab1);
subplot(2, 1, 2); plot(b.t3, b.v3, 'LineWidth', 1.1);
grid on; ylim([-400 400]); xlabel('Time (ms)'); ylabel('Load voltage (V)'); title(lab2);
saveFig(f, fullfile(outdir, 'fig10_output_voltage_comparison.png'));
end

function fig14(a, b, lab1, lab2, outdir)
f = figure('Visible', 'off', 'Position', [100 100 820 380]);
plot(a.tR, a.vrms, 'LineWidth', 1.2); hold on;
plot(b.tR, b.vrms, 'LineWidth', 1.2);
yline(230, '--k', '230 V target');
grid on; xlabel('Time (s)'); ylabel('Output RMS voltage (V)');
legend(lab1, lab2, 'Location', 'southeast');
title('Output RMS voltage under PI control (one-cycle moving window)');
saveFig(f, fullfile(outdir, 'fig14_vrms_settling.png'));
end

function fig18(a, b, lab1, lab2, outdir)
f = figure('Visible', 'off', 'Position', [100 100 820 520]);
ds = {a, b};
labs = {lab1, lab2};
for k = 1:2
    d  = ds{k};
    n2 = d.t3 <= 2e3 / d.f_out;                      % first two cycles of the window
    subplot(2, 1, k);
    yyaxis left;  plot(d.t3(n2), d.v3(n2), 'LineWidth', 1.1); ylabel('Voltage (V)');
    yyaxis right; plot(d.t3(n2), d.i3(n2), 'LineWidth', 1.1); ylabel('Current (A)');
    grid on; title([labs{k} ': voltage and current (lagging, PF about 0.9)']);
    if k == 2, xlabel('Time (ms)'); end
end
saveFig(f, fullfile(outdir, 'fig18_voltage_current.png'));
end

function fig19(a, b, lab1, lab2, outdir)
f = figure('Visible', 'off', 'Position', [100 100 820 400]);
semilogy(a.h, a.Hpct, 'o-', 'LineWidth', 1); hold on;
semilogy(b.h, b.Hpct, 's-', 'LineWidth', 1);
grid on; xlabel('Harmonic order (60 Hz fundamental)'); ylabel('Amplitude (% of fundamental)');
legend(sprintf('%s, THD %.2f %%', lab1, a.THD), sprintf('%s, THD %.2f %%', lab2, b.THD), ...
       'Location', 'northeast');
title('Load voltage spectrum, harmonics 2 to 50');
saveFig(f, fullfile(outdir, 'fig19_harmonic_spectrum.png'));
end

%% =====================================================================
function saveFig(f, path)
exportgraphics(f, path, 'Resolution', 200);
close(f);
fprintf('Saved %s\n', path);
end

function x = vec(d)
x = squeeze(d);
x = x(:);
end

function ts = getVals(out, names, required)
ts = [];
for k = 1:numel(names)
    try
        el = out.logsout.getElement(names{k});
    catch
        el = [];
    end
    if ~isempty(el)
        ts = el.Values;
        return
    end
end
if required
    error('Signal %s is not logged in out.logsout.', names{1});
end
end