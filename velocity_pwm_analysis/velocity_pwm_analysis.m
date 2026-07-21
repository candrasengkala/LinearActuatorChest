%% analyze_velocity.m
% Computes velocity (mm/s) from position-vs-time CSV logs and plots
% position, velocity, and PWM/dir for each file. Also builds a
% velocity-vs-PWM (duty cycle) characterization plot across all files.
%
% Expected CSV columns: time_ms, dir, pwm, position (UTF-16LE encoded)
% dir: 1 = extend (+), 0 = retract (-)

clear; clc; close all;

files = { ...
    'position_50_200.csv', ...
    'position_100_100.csv', ...
    'position_200_50.csv', ...
    'position_50_255.csv', ...
    'position_255_50.csv'};

figure('Name','Position & Velocity');

% aggregate storage for velocity-vs-PWM plot
pwm_all = [];
v_all   = [];
file_id = [];

for k = 1:numel(files)
    T = readtable(files{k}, 'Encoding', 'UTF-16LE');

    t    = T.time_ms / 1000;     % s
    pos  = T.position;           % mm
    dirn = T.dir;
    pwm  = T.pwm;

    % velocity via central difference (mm/s), smoothed to cut quantization noise
    v = gradient(pos, t);
    v = smoothdata(v, 'movmean', 5);

    fprintf('--- %s ---\n', files{k});
    fprintf('  max |v|   = %.2f mm/s\n', max(abs(v)));
    fprintf('  mean |v|  = %.2f mm/s (moving only, |v|>1)\n', mean(abs(v(abs(v)>1))));
    fprintf('  pwm range = %.0f - %.0f\n\n', min(pwm), max(pwm));

    subplot(numel(files), 2, 2*k-1);
    plot(t, pos, 'LineWidth', 1.2);
    ylabel('Position (mm)'); title(files{k}, 'Interpreter', 'none');
    xlabel('Time (s)'); grid on;

    subplot(numel(files), 2, 2*k);
    plot(t, v, 'LineWidth', 1.2);
    ylabel('Velocity (mm/s)'); xlabel('Time (s)');
    title('Velocity'); grid on;

    % raw (unsigned) pwm — sign is assigned later from *measured* velocity,
    % not from the dir flag, since dir's physical sign isn't reliable.
    pwm_all = [pwm_all; pwm];               %#ok<AGROW>
    v_all   = [v_all;   v];                 %#ok<AGROW>
    file_id = [file_id; k*ones(size(v))];   %#ok<AGROW>
end

%% Velocity vs PWM duty cycle (aggregated, steady-state points only)
% Exclude transient/near-zero samples so each PWM level reflects
% steady-state speed rather than acceleration ramps.
steady = abs(v_all) > 1;

% Sign PWM by the *measured* velocity direction so -v always maps to
% -pwm and +v always maps to +pwm, regardless of the dir flag's
% (apparently inconsistent) physical convention.
signed_pwm_all = pwm_all .* sign(v_all);

figure('Name','Velocity vs PWM Duty Cycle');
hold on;
marker_set = {'o','s','^','d','v','p','h','x','+','*'};
markers = marker_set(mod(0:numel(files)-1, numel(marker_set)) + 1);
colors  = lines(numel(files));
for k = 1:numel(files)
    idx = steady & (file_id == k);
    scatter(signed_pwm_all(idx), v_all(idx), 18, colors(k,:), markers{k}, 'filled', ...
        'DisplayName', files{k});
end

% mean velocity per unique signed PWM level (steady-state), across all files
uniq_pwm = unique(signed_pwm_all(steady));
mean_v   = arrayfun(@(p) mean(v_all(steady & signed_pwm_all==p)), uniq_pwm);

plot(uniq_pwm, mean_v, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, ...
    'DisplayName', 'Mean (steady-state)');

% Smooth monotonic interpolation through the aggregated points (pchip
% avoids overshoot/ringing that a cubic spline can introduce).
if numel(uniq_pwm) >= 2
    pwm_fine = linspace(min(uniq_pwm), max(uniq_pwm), 200);
    v_fine   = interp1(uniq_pwm, mean_v, pwm_fine, 'pchip');
    plot(pwm_fine, v_fine, 'k-', 'LineWidth', 1.5, ...
        'DisplayName', 'Interpolated characteristic (pchip)');

    % linear least-squares fit through the mean steady-state points
    % (gradient = actuator gain, mm/s per PWM count)
    p_fit = polyfit(uniq_pwm, mean_v, 1);
    fprintf('--- Velocity-PWM characteristic (linear fit) ---\n');
    fprintf('  v = %.5f * pwm + %.4f\n', p_fit(1), p_fit(2));
    fprintf('  Gradient = %.5f mm/s per PWM count (%.3f PWM counts per mm/s)\n\n', ...
        p_fit(1), 1/p_fit(1));

    plot(pwm_fine, polyval(p_fit, pwm_fine), 'r--', 'LineWidth', 1.2, ...
        'DisplayName', sprintf('Linear fit (gradient = %.4f mm/s/PWM)', p_fit(1)));

    % --- Inverse direction: PWM as a function of velocity ---
    % Fit directly (not just algebraically inverted) since least-squares
    % is not symmetric under inversion — this is the fit you actually
    % want for velocity feedforward (desired v -> commanded PWM).
    p_fit_inv = polyfit(mean_v, uniq_pwm, 1);
    fprintf('--- PWM-Velocity characteristic (inverse linear fit) ---\n');
    fprintf('  pwm = %.5f * v + %.4f\n', p_fit_inv(1), p_fit_inv(2));
    fprintf('  Gradient = %.5f PWM counts per mm/s (%.4f mm/s per PWM count)\n', ...
        p_fit_inv(1), 1/p_fit_inv(1));
    fprintf('  (compare to direct-fit inversion: %.3f PWM counts per mm/s -- ', 1/p_fit(1));
    fprintf('difference reflects fit-direction asymmetry)\n\n');
end

xlabel('Signed PWM (sign from measured velocity direction)');
ylabel('Velocity (mm/s)');
title('Velocity vs PWM Duty Cycle');
legend('Location', 'best');
grid on; hold off;