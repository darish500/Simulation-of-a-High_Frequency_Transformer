%% =====================================================================
%  analyze_run.m  -  run this AFTER each simulation (case 1, then case 2)
%  Needs these logged signals in out.logsout (signal logging on):
%     V_in  I_in  V_dc  I_dc  V_load  I_load
%  Uses variables left in the workspace by inverter.m
%  =====================================================================
Ncyc = 5;                                   % number of output cycles analysed (the last ones)

%% ----- 1. Fetch the logged signals -----
if ~exist('out','var') || ~isa(out,'Simulink.SimulationOutput')
    error('Variable "out" not found. Run the model first (it creates "out").');
end
names = {'V_in','I_in','V_dc','I_dc','V_load','I_load'};
S = struct();
for k = 1:numel(names)
    n = names{k};
    el = out.logsout.getElement(n);
    if isempty(el)
        error('Signal %s is not logged. Check the signal names in out.logsout.', n);
    end
    S.(n) = el.Values;                      % timeseries with .Time and .Data
end

%% ----- 1b. Safety check: is the logged data from the case now in the workspace? -----
dt = median(diff(S.V_in.Time));
if abs(dt - Ts_power)/Ts_power > 0.01
    error(['The logged data has a time step of %.3g s, but Ts_power for case %d is %.3g s.\n' ...
           'Run the MODEL again for this case, then run analyze_run.'], dt, case_sel, Ts_power);
end
if abs(S.V_in.Time(end) - Tstop) > 10*Ts_power
    error(['The logged data ends at %.4g s, but Tstop is %.4g s.\n' ...
           'Run the MODEL again, then run analyze_run.'], S.V_in.Time(end), Tstop);
end

%% ----- 2. Resample the last Ncyc cycles onto one uniform grid -----
Ts = Ts_power;
N  = round(Ncyc/(f_out*Ts));                % number of samples in the window
T1 = S.V_in.Time(end);
tg = T1 - N*Ts + (0:N-1)'*Ts;
g  = @(n) interp1(S.(n).Time, squeeze(S.(n).Data), tg, 'linear');

v_in_w = g('V_in');   i_in_w = g('I_in');
v_dc_w = g('V_dc');   i_dc_w = g('I_dc');
v_ld_w  = g('V_load'); i_ld_w  = g('I_load');

%% ----- 3. Powers and efficiencies -----
R.caseNum  = case_sel;
R.fsw      = fsw;
R.fsw_dc   = fsw_dc;
R.Pin      = mean(v_in_w.*i_in_w);                % input power (W)
R.Plink    = mean(v_dc_w.*i_dc_w);                % power leaving the flyback (W)
R.Pout     = mean(v_ld_w.*i_ld_w);                  % power into the load (W)
R.loss_fly = R.Pin   - R.Plink;             % flyback + source losses (W)
R.loss_inv = R.Plink - R.Pout;              % bridge + filter + transformer losses (W)
R.eta_fly  = 100*R.Plink/R.Pin;
R.eta_inv  = 100*R.Pout/R.Plink;
R.eta      = 100*R.Pout/R.Pin;
R.Vdc_avg  = mean(v_dc_w);
R.Vrms     = sqrt(mean(v_ld_w.^2));

%% ----- 4. THD of the load voltage (harmonics 2..50) -----
Y  = fft(v_ld_w)/N;
A  = 2*abs(Y(2:floor(N/2)));                % amplitude of bin k is A(k)
A1 = A(Ncyc);                               % fundamental sits at bin Ncyc+1
hs = (2:50)*Ncyc;
R.THD50 = 100*sqrt(sum(A(hs).^2))/A1;

%% ----- 5. Sizes (from inverter.m) -----
R.L1 = L1;  R.L2 = L2;  R.Cf = Cf;
R.E_L = E_L;  R.E_C = E_C;  R.VsFly = VsFly;  R.Lm_fly = Lm_fly;

%% ----- 6. Print this run and save it -----
fprintf('\n=== Case %d  (fsw = %g kHz, fsw_dc = %g kHz) ===\n', case_sel, fsw/1e3, fsw_dc/1e3);
fprintf('Pin = %.1f W | Plink = %.1f W | Pout = %.1f W\n', R.Pin, R.Plink, R.Pout);
fprintf('Flyback stage:   efficiency %.1f %%  (loss %.1f W)\n', R.eta_fly, R.loss_fly);
fprintf('Bridge+filter+tx: efficiency %.1f %%  (loss %.1f W)\n', R.eta_inv, R.loss_inv);
fprintf('Total efficiency %.1f %% | Vdc %.1f V | Vrms %.1f V | THD(2..50) %.2f %%\n', ...
        R.eta, R.Vdc_avg, R.Vrms, R.THD50);
save(sprintf('results_case%d.mat', case_sel), 'R');

%% ----- 7. If both cases are saved, print the comparison table -----
if isfile('results_case1.mat') && isfile('results_case2.mat')
    a = load('results_case1.mat'); a = a.R;
    b = load('results_case2.mat'); b = b.R;
    row = @(lbl, x, y, f) fprintf(['%-30s ' f ' ' f '\n'], lbl, x, y);
    fprintf('\n%-30s %14s %14s\n', 'COMPARISON', 'Case 1', 'Case 2');
    row('Switching frequency (kHz)', a.fsw/1e3,   b.fsw/1e3,   '%14.1f');
    row('Vrms (V)',                  a.Vrms,      b.Vrms,      '%14.1f');
    row('Output power (W)',          a.Pout,      b.Pout,      '%14.1f');
    row('Input power (W)',           a.Pin,       b.Pin,       '%14.1f');
    row('Flyback efficiency (%)',    a.eta_fly,   b.eta_fly,   '%14.1f');
    row('Bridge+filter eff. (%)',    a.eta_inv,   b.eta_inv,   '%14.1f');
    row('Total efficiency (%)',      a.eta,       b.eta,       '%14.1f');
    row('Flyback loss (W)',          a.loss_fly,  b.loss_fly,  '%14.1f');
    row('Bridge+filter loss (W)',    a.loss_inv,  b.loss_inv,  '%14.1f');
    row('THD, harmonics 2-50 (%)',   a.THD50,     b.THD50,     '%14.2f');
    row('L1 (mH)',                   a.L1*1e3,    b.L1*1e3,    '%14.2f');
    row('L2 (mH)',                   a.L2*1e3,    b.L2*1e3,    '%14.2f');
    row('Cf (uF)',                   a.Cf*1e6,    b.Cf*1e6,    '%14.1f');
    row('Inductor energy (J)',       a.E_L,       b.E_L,       '%14.3f');
    row('Capacitor energy (J)',      a.E_C,       b.E_C,       '%14.3f');
    row('Flyback Lm (uH)',           a.Lm_fly*1e6,b.Lm_fly*1e6,'%14.1f');
    row('Flyback volt-seconds (uVs)',a.VsFly*1e6, b.VsFly*1e6, '%14.0f');
end