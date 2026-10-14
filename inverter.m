%% =====================================================================
%  inverter.m  -  workspace parameters for high_frequency_inverter.slx
%  Run this before the model. Change case_sel to switch designs.
%  All values are placeholders until the client confirms specs.
%  =====================================================================

%% ===== CASE SWITCH =====
case_sel =2;            % 1 = conventional, 2 = high-frequency (20 kHz switching)

spp = 100;               % samples per fastest switching period (lower = faster, coarser)

f_out = 60;              % output frequency (Hz), client value, same in both cases

switch case_sel
    case 1               % conventional inverter (low switching frequency)
        fsw    = 5e3;    % inverter PWM carrier (Hz), placeholder
        fsw_dc = 5e3;    % DC-DC (flyback) switching frequency (Hz), placeholder
    case 2               % high-frequency inverter
        fsw    = 20e3;   % inverter PWM carrier (Hz), client value
        fsw_dc = 20e3;   % DC-DC (flyback) switching frequency (Hz)
    otherwise
        error('case_sel must be 1 or 2');
end

%% ===== Simulation timing =====
Ts_power = 1/(spp*max(fsw, fsw_dc));   % power-circuit sample time (s)
Ts_ctrl  = 1/fsw;                      % controller sample time (s), used later by the PI
Tstop    = 0.05 + 10/f_out;            % start-up time + 10 output cycles (s)

%% ===== Ratings and targets =====
Vin       = 48;          % DC source voltage (V)
Vdc       = 400;         % DC link voltage after DC-DC (V)
P_load    = 1000;        % active power (W)
PF_ref    = 0.9;         % PF setpoint
Vout_rms  = 230;         % output voltage (Vrms)

%% ===== Device parameters =====
% Replace Rds_on and R_src with these (placeholders, client to confirm devices)
Rds_on_dc  = 5e-3;    % low-voltage flyback MOSFET (ohm)
Rds_on_inv = 50e-3;   % 400 V bridge MOSFETs (ohm)
R_src      = 2e-3;    % source series resistance (ohm)
Rd_diode  = 0.01;        % diode resistance (ohm)
Vf_diode  = 0.8;         % diode forward voltage (V)
Rsn       = 1e5;         % snubber resistance (ohm)
Csn       = inf;         % snubber capacitance (F), inf = none
%R_src     = 0.01;        % DC source series resistance (ohm)
R_gnd     = 1e6;         % ground reference resistor (ohm)

%% ===== Flyback DC-DC (HF transformer stage) =====
n_fly     = 8;                  % turns ratio Ns/Np
D_fly     = 0.55;               % duty cycle
dIm       = 24.5;               % magnetizing current ripple kept constant (A)
Lm_fly    = Vin*D_fly/(fsw_dc*dIm);   % magnetizing inductance, primary side (H)
Llk_fly = 0.01*Lm_fly;     % was 0.5e-6, leakage = 1 % of the magnetizing inductance
Llk2_fly  = Llk_fly*n_fly^2;    % secondary leakage, referred to secondary (H)
Rp_fly    = 0.01;               % primary winding resistance (ohm)
Rs_fly    = Rp_fly*n_fly^2;     % secondary winding resistance (ohm)
Rm_fly    = 10e3;               % core-loss resistance (ohm)
Co_fly    = 470e-6;             % output capacitor (F)
V1_fly    = Vin;                % transformer rated primary voltage (V)
V2_fly    = n_fly*Vin;          % transformer rated secondary voltage (V)
Pn_fly    = 1.5e3;              % transformer rating (VA)
N_fly     = round(1/(fsw_dc*Ts_power));   % samples per flyback period
W_fly     = round(D_fly*N_fly);           % ON samples per flyback period

%% ===== Inverter bridge / PWM =====
m_a       = 0.8;                % modulation index
Vtri_pk   = 1;                  % carrier peak amplitude
t_tri     = [0 1/(2*fsw) 1/fsw];      % triangle time points (s)
v_tri     = Vtri_pk*[-1 1 -1];        % triangle values
w_out     = 2*pi*f_out;               % sine frequency (rad/s)

%% ===== T-LCL filter (sized from the frequencies) =====
Ipk   = sqrt(2)*P_load/(Vout_rms*PF_ref);              % peak load current (A)
dI    = 0.2*Ipk;                                       % 20 % current ripple (A)
L1    = Vdc/(16*fsw*dI);                               % inverter-side inductor (H)
L2    = L1/2;                                          % output-side inductor (H)
Cf    = 0.10*(P_load/PF_ref)/(2*pi*f_out*Vout_rms^2);  % 10 % reactive-power budget (F)
f_res = (1/(2*pi))*sqrt((L1+L2)/(L1*L2*Cf));           % resulting resonance, for checking (Hz)
R_d   = 0.3*sqrt((L1+L2)/Cf);                          % damping resistor in series with Cf (ohm)
R_L1  = 0.05;                                          % inductor series resistance (ohm)
R_L2  = 0.05;
%% ===== Output transformer (1:1) =====
Sn_tx     = 2e3;         % rating (VA)
fn_tx     = f_out;       % rated frequency (Hz)
V1_tx     = 230;         % primary voltage (Vrms)
V2_tx     = 230;         % secondary voltage (Vrms)
Rm_tx     = 500;         % magnetization resistance (pu)
Lm_tx     = 500;         % magnetization inductance (pu)

%% ===== Load and temporary test loads =====
R_ac_test = Vout_rms^2/P_load;   % temporary AC test load (ohm)
R_test    = 160;                 % temporary DC-link test load (ohm), unused now
%% 
% R1_tx = 0.005;   % primary winding resistance (pu)
% L1_tx = 0.02;    % primary leakage inductance (pu)
% R2_tx = 0.005;   % secondary winding resistance (pu)
% L2_tx = 0.02;    % secondary leakage inductance (pu)

% %% ===== PI controller (tuned later) =====
% Kp        = 0.5;
% Ki        = 50;
% 


%% ===== Summary =====
fprintf('Case %d | f_out = %g Hz | fsw = %g kHz | fsw_dc = %g kHz\n', ...
        case_sel, f_out, fsw/1e3, fsw_dc/1e3);
fprintf('Ts_power = %g ns | Tstop = %g s | steps = %.2g\n', ...
        Ts_power*1e9, Tstop, Tstop/Ts_power);
fprintf('L1 = %g uH | L2 = %g uH | Cf = %g uF | R_d = %g ohm | f_res = %g kHz\n', ...
        L1*1e6, L2*1e6, Cf*1e6, R_d, f_res/1e3);
fprintf('Lm_fly = %g uH | N_fly = %d | W_fly = %d\n', Lm_fly*1e6, N_fly, W_fly);

R1_tx = 0.005;   % primary winding resistance (pu)
L1_tx = 0.02;    % primary leakage inductance (pu)
R2_tx = 0.005;   % secondary winding resistance (pu)
L2_tx = 0.02;    % secondary leakage inductance (pu)
Q_load = P_load*tan(acos(PF_ref));   % inductive reactive power for the PF target (var) 

% PI controller (regulates Vrms by adjusting the modulation index)
Kp     = 0.002;                    % was 0.5, far too large for this plant
Ki     = 0.03;                      % was 50
m_ff   = sqrt(2)*Vout_rms/Vdc;     % feedforward modulation index (about 0.81)
m_trim = 0.1;                      % PI correction limit
m_max  = 0.95;                     % keep below over-modulation
Tstop  = 0.3 + 10/f_out;           % longer run so the PI can settle

%% ===== Size indicators for the comparison =====
E_L   = 0.5*(L1+L2)*Ipk^2;            % inductor stored energy (J)
E_C   = 0.5*Cf*(sqrt(2)*Vout_rms)^2;  % capacitor stored energy (J)
VsFly = Vin*D_fly/fsw_dc;             % flyback volt-seconds (V*s), core size
% 
%  indicator
fprintf('E_L = %.3f J | E_C = %.3f J | Flyback V*s = %.2f uV*s\n', E_L, E_C, VsFly*1e6);

%% ===== Switching-loss estimate (assumed device data, post-processing) =====
t_sw    = 40e-9;      % rise + fall time (s)
Coss    = 200e-12;    % bridge MOSFET output capacitance (F)
Ipk_pri = P_load/(0.85*Vin*D_fly) + dIm/2;       % flyback peak current (A)
P_sw_fly = 0.5*(Vin + Vdc/n_fly)*Ipk_pri*t_sw*fsw_dc;
P_sw_inv = 0.5*Vdc*Ipk*t_sw*fsw*(2/pi)*2 + 4*0.5*Coss*Vdc^2*fsw;
fprintf('Estimated switching loss: flyback %.1f W | bridge %.1f W | total %.1f W\n', ...
        P_sw_fly, P_sw_inv, P_sw_fly + P_sw_inv);