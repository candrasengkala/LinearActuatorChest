%% estimate_actuator_tf_tfest.m
% Alternative to estimate_actuator_tf.m -- uses tfest/procest (System
% Identification Toolbox) instead of a manual fminsearch fit.
%
% Same saturation-handling logic as before (excludes the end-stop
% collision region), but lets tfest/procest handle the actual optimization,
% and compares a 1st-order vs 2nd-order model fit automatically.
%
% Requires: System Identification Toolbox (tfest, procest, iddata).
%
% Expected CSV columns (header row): time,delta_t,pwm,position_reading,position_estimate,velocity_estimate
%   time               - milliseconds
%   delta_t            - loop dt in seconds (unused here; t is rebuilt from time instead)
%   pwm                - commanded PWM duty cycle (assumed constant step)
%   position_reading   - raw potentiometer reading, raw units (unused here)
%   position_estimate  - Kalman-filtered position estimate, raw units (/10 -> mm)
%   velocity_estimate  - Kalman-filtered velocity estimate, raw units/s (/10 -> mm/s)
clear; clc; close all;

%% --- Load data ---
csv_path = "p_updated_q_05_r_measurement_45_extend_100ms.csv";  % <-- change if your filename differs
try
    T = readtable(csv_path, "TextType", "string", "Encoding", "UTF-16LE");
catch
    T = readtable(csv_path, "TextType", "string");
end
t_ms = T.time;
pwm  = T.pwm;
pos  = T.position_estimate; % raw units -> mm
vel  = T.velocity_estimate; % raw units/s -> mm/s
t = (t_ms - t_ms(1)) / 1000;   % ms -> s, re-zero to step start

if numel(unique(pwm)) > 1
    warning("PWM is not constant across the capture. tfest can technically " + ...
        "handle time-varying input, but this script assumes a clean step.");
end
pwm_step = pwm(1);

%% --- Detect and exclude end-stop saturation ---
% Skip the initial startup transient (KF init/settling spike -- see setQ/init
% discussion) so it isn't mistaken for the end-stop saturation peak. Only
% search for the saturation peak after this many samples.
skip_startup = 15;
search_start = min(skip_startup + 1, numel(vel));
[~, rel_peak_idx] = max(vel(search_start:end));
peak_idx = search_start - 1 + rel_peak_idx;
margin_samples = 15;
cutoff_idx = max(1, peak_idx - margin_samples);

if cutoff_idx < 10
    error("tf_estimation:tooFewSamples", ...
        ["Only %d sample(s) remain before the detected saturation point (t = %.2f s).\n" ...
         "This usually means the run is too short, or the end-stop peak detector is " ...
         "still catching a transient rather than real saturation.\n" ...
         "Try increasing skip_startup (currently %d) or inspect vel visually with:\n" ...
         "    plot(t, vel); xline(t(peak_idx), 'r--');"], ...
        cutoff_idx, t(peak_idx), skip_startup);
end

fprintf("Detected end-stop saturation near t = %.2f s (sample %d of %d).\n", ...
    t(peak_idx), peak_idx, numel(t));
fprintf("Using samples 1:%d (t = 0 to %.2f s) for identification.\n\n", ...
    cutoff_idx, t(cutoff_idx));

t_fit   = t(1:cutoff_idx);
vel_fit = vel(1:cutoff_idx);
pos_fit = pos(1:cutoff_idx);
pwm_fit = pwm(1:cutoff_idx);

%% --- Resample onto a uniform time grid ---
% tfest/iddata expect (or strongly prefer) a fixed sample time.
Ts = median(diff(t_fit));   % estimated sample time from the data itself
t_uniform = (0:Ts:t_fit(end))';
pwm_uniform = interp1(t_fit, pwm_fit, t_uniform, "previous", "extrap");
vel_uniform = interp1(t_fit, vel_fit, t_uniform, "linear", "extrap");
pos_uniform = interp1(t_fit, pos_fit, t_uniform, "linear", "extrap");
fprintf("Resampled to uniform Ts = %.4f s (%d samples).\n\n", Ts, numel(t_uniform));

%% --- Build iddata objects ---
data_vel = iddata(vel_uniform, pwm_uniform, Ts);
data_vel.InputName  = "PWM";
data_vel.OutputName = "Velocity";
data_vel.TimeUnit   = "seconds";

data_pos = iddata(pos_uniform, pwm_uniform, Ts);
data_pos.InputName  = "PWM";
data_pos.OutputName = "Position";
data_pos.TimeUnit   = "seconds";

%% --- Fit transfer functions: PWM -> velocity ---
% Try 1st-order and 2nd-order (no zeros) and compare fit quality.
opt = tfestOptions("Display", "off");
G_vel_1 = tfest(data_vel, 1, 0, opt);   % 1 pole, 0 zeros: K/(tau*s+1)
G_vel_2 = tfest(data_vel, 2, 0, opt);   % 2 poles, 0 zeros

fprintf("=== 1st-order fit (PWM -> velocity) ===\n");
G_vel_1
fprintf("Fit quality: %.2f%% (NRMSE)\n\n", G_vel_1.Report.Fit.FitPercent);

fprintf("=== 2nd-order fit (PWM -> velocity) ===\n");
G_vel_2
fprintf("Fit quality: %.2f%% (NRMSE)\n\n", G_vel_2.Report.Fit.FitPercent);

%% --- Fit transfer function: PWM -> position directly ---
% Position TF must have an integrator (pole at origin). We force this structure
% using a process model via procest with 'P1I' (1 pole, 1 integrator).
opt_proc = procestOptions("Display", "off");
G_pos_direct = procest(data_pos, "P1I", opt_proc); 

fprintf("=== Constrained process direct fit (PWM -> position) ===\n");
G_pos_direct
fprintf("Fit quality: %.2f%% (NRMSE)\n\n", G_pos_direct.Report.Fit.FitPercent);

% Derived position TFs from the identified velocity TFs
s = tf('s');
G_pos_from_vel1 = G_vel_1 / s;
G_pos_from_vel2 = G_vel_2 / s;

%% --- Compare fits visually ---
figure("Name", "tfest Model Comparison: Velocity");
compare(data_vel, G_vel_1, G_vel_2);
title("PWM -> Velocity: 1st-order vs 2nd-order tfest fit");
grid on;

figure("Name", "tfest Model Comparison: Position");
compare(data_pos, G_pos_direct);
hold on;

% Plot derived position from 1st-order velocity
[y_derived1, t_derived1] = step(pwm_step * G_pos_from_vel1, t_fit(end));
plot(t_derived1, y_derived1, "g--", "LineWidth", 1.5, ...
    "DisplayName", "Derived from 1st-order velocity (G\_vel\_1/s)");

% Plot derived position from 2nd-order velocity
[y_derived2, t_derived2] = step(pwm_step * G_pos_from_vel2, t_fit(end));
plot(t_derived2, y_derived2, "m:", "LineWidth", 1.5, ...
    "DisplayName", "Derived from 2nd-order velocity (G\_vel\_2/s)");

legend("show", "Location", "best");
title("PWM -> Position: direct fit vs derived-from-velocity");
grid on;

%% --- Pole/zero and stability check ---
fprintf("=== Pole locations (1st-order velocity model) ===\n");
disp(pole(G_vel_1));

fprintf("=== Pole locations (2nd-order velocity model) ===\n");
disp(pole(G_vel_2));

if any(real(pole(G_vel_2)) > 0)
    warning("2nd-order fit produced an unstable pole -- likely overfitting " + ...
        "noise rather than capturing real dynamics. Prefer the 1st-order model.");
end

%% --- Recommendation ---
fprintf("\n=== Summary ===\n");
if G_vel_2.Report.Fit.FitPercent - G_vel_1.Report.Fit.FitPercent > 5
    fprintf("2nd-order model fits meaningfully better (+%.1f%% FIT) -- consider using it.\n", ...
        G_vel_2.Report.Fit.FitPercent - G_vel_1.Report.Fit.FitPercent);
else
    fprintf("2nd-order model does not fit meaningfully better than 1st-order.\n");
    fprintf("Stick with the simpler 1st-order model (G_vel_1) unless you have\n");
    fprintf("a physical reason (e.g. motor inductance) to expect 2nd-order dynamics.\n");
end