clear; clc; close all;

files = { ...
    'determined_20mm_05mm-sec.csv', ...
    'determined_30mm_05mm-sec.csv', ...
    'determined_50mm_10mm-sec.csv'};

figure('Name','Position & Velocity');

% aggregate storage for velocity-vs-PWM plot
pwm_all = [];
v_all   = [];
file_id = [];

for k = 1:numel(files)
    T = readtable(files{k}, 'Encoding', 'UTF-16LE');

    % --- CLIP INITIAL HOMING DATA UNTIL POSITION REACHES 0 ---
    idx_zero = find(T.position == 0, 1, 'first');
    if ~isempty(idx_zero)
        T = T(idx_zero:end, :);
    end

    % Re-zero time starting from new clipped point
    t    = (T.time_ms - T.time_ms(1)) / 1000;     % s
    pos  = T.position;           % mm
    dirn = T.dir;
    pwm  = T.pwm;

    % velocity via central difference (mm/s), smoothed to cut quantization noise
    v = gradient(pos, t);
    v = smoothdata(v, 'movmean', 5);

    % --- FREQUENCY COUNTER ---
    % Detect peaks in position to find average period (T) and frequency (f)
    [pks, locs] = findpeaks(pos, t, ...
        'MinPeakHeight', max(pos) * 0.5, ...   % filter out small ripples
        'MinPeakDistance', 2);                 % minimum 2 sec between peaks

    if numel(locs) > 1
        periods    = diff(locs);               % time delta between consecutive peaks (s)
        avg_period = mean(periods);            % average period T (s)
        freq_hz    = 1 / avg_period;           % frequency f (Hz)
        freq_cpm   = freq_hz * 60;             % cycles per minute
    else
        avg_period = NaN; freq_hz = NaN; freq_cpm = NaN;
    end

    % Console Output
    fprintf('--- %s ---\n', files{k});
    if ~isempty(idx_zero)
        fprintf('  clipped start row = %d\n', idx_zero);
    end
    fprintf('  max |v|        = %.2f mm/s\n', max(abs(v)));
    fprintf('  mean |v|       = %.2f mm/s (moving only, |v|>1)\n', mean(abs(v(abs(v)>1))));
    fprintf('  pwm range      = %.0f - %.0f\n', min(pwm), max(pwm));
    fprintf('  osc. period    = %.2f s\n', avg_period);
    fprintf('  osc. frequency = %.4f Hz (%.2f cpm)\n\n', freq_hz, freq_cpm);

    % Plot Position
    subplot(numel(files), 2, 2*k-1);
    plot(t, pos, 'LineWidth', 1.2); hold on;
    if ~isempty(locs)
        plot(locs, pks, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4); % peak markers
    end
    hold off;
    ylabel('Position (mm)'); 
    title(sprintf('%s (f = %.3f Hz / %.1f cpm)', files{k}, freq_hz, freq_cpm), 'Interpreter', 'none');
    xlabel('Time (s)'); grid on;

    % Plot Velocity
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