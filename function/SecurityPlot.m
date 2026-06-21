%% =========================================================
%  SecurityPlot.m — Chi-squared gate security analysis
%  ---------------------------------------------------------
%  Produces a 3-panel figure for the research committee:
%
%  Panel 1 (top):    X Position over time
%     — Real (ground truth)
%     — Noisy V2V (raw received, includes attack)
%     — Standard KF (vulnerable — corrupted by attack)
%     — Gated KF    (protected  — rejects attack)
%     — Shaded region shows attack window
%
%  Panel 2 (middle): Velocity over time (same 4 lines)
%
%  Panel 3 (bottom): NIS over time
%     — NIS at each filter step
%     — Gate threshold γ = 5.991 (chi2inv(0.95,2))
%     — Red markers where gate fires (measurement rejected)
%     — Running mean NIS (filter consistency check)
%
%  CALL: SecurityPlot(LR, L1, target_car_id, attack_t0, attack_tend)
%
%  Example:
%    target_car_id = 'v10000';
%    SecurityPlot(LR, L1, target_car_id, 30, 70);
% =========================================================

function SecurityPlot(LR, L1, target_car_id, attack_t0, attack_tend)

    %% --- Find the LR car and L1 target ---
    selected_LR = [];
    for k = 1:length(L1)
        if L1(k).OgLnID == 2
            selected_LR = L1(k);
            break;
        end
    end
    if isempty(selected_LR)
        error('No merged LR car found in L1. Check OgLnID assignments.');
    end

    % Find L1 car matching target_car_id
    selected_L1 = [];
    for k = 1:length(L1)
        if sprintf('v%d', L1(k).ID) == target_car_id
            selected_L1 = L1(k);
            break;
        end
    end
    if isempty(selected_L1)
        error('Target car %s not found.', target_car_id);
    end

    %% --- Extract data ---
    % Time
    t = selected_L1.t;

    % Real (ground truth)
    x_real = selected_L1.str_X;
    v_real = selected_L1.str_V;

    % Noisy V2V
    x_noisy = selected_LR.str_v2v_data.(target_car_id).X;
    v_noisy = selected_LR.str_v2v_data.(target_car_id).V;

    % Standard KF (vulnerable — no gate)
    KF_std = selected_LR.KF_v2v.(target_car_id);
    x_KF_std = KF_std.xhatstore(1, :);
    v_KF_std = KF_std.xhatstore(2, :);

    % Gated KF (protected — chi2 gate)
    KF_gated = selected_LR.KF_gated_v2v.(target_car_id);
    x_KF_gate = KF_gated.xhatstore(1, :);
    v_KF_gate = KF_gated.xhatstore(2, :);
    NIS        = KF_gated.NIS_store;
    anomaly    = KF_gated.anomaly_store;

    % Ensure all vectors are row vectors for consistency
    x_real  = x_real(:)';  v_real  = v_real(:)';
    x_noisy = x_noisy(:)'; v_noisy = v_noisy(:)';

    % Truncate/align to same length as t
    n = min([length(t), length(x_real), length(x_noisy), ...
             length(x_KF_std), length(x_KF_gate)]);
    t        = t(1:n);
    x_real   = x_real(1:n);   v_real   = v_real(1:n);
    x_noisy  = x_noisy(1:n);  v_noisy  = v_noisy(1:n);
    x_KF_std = x_KF_std(1:n); v_KF_std = v_KF_std(1:n);
    x_KF_gate= x_KF_gate(1:n);v_KF_gate= v_KF_gate(1:n);
    NIS      = NIS(1:n);
    anomaly  = anomaly(1:n);

    %% --- Gate threshold ---
    try
        gate_threshold = chi2inv(0.95, 2);
    catch
        gate_threshold = 5.9915;   % chi2inv(0.95,2) hardcoded
    end

    %% --- Colours ---
    col_real  = [0.20 0.40 0.80];   % blue  — ground truth
    col_noisy = [0.40 0.70 0.30];   % green — raw V2V
    col_std   = [0.85 0.20 0.20];   % red   — standard KF (attacked)
    col_gate  = [0.05 0.55 0.35];   % teal  — gated KF  (protected)
    col_atk   = [1.00 0.88 0.88];   % light red shading — attack window
    lw = 1.5;

    %% --- Create figure ---
    fig = figure('Name', 'Security Analysis: Chi-squared Innovation Gate', ...
                 'Units', 'inches', 'Position', [0.5, 0.5, 14, 9]);

    % =========================================================
    %  PANEL 1 — X Position
    % =========================================================
    ax1 = subplot(3, 1, 1);
    hold on; box on; grid on;

    % Attack window shading
    yl = [min([x_real, x_noisy, x_KF_std, x_KF_gate]) - 20, ...
          max([x_real, x_noisy, x_KF_std, x_KF_gate]) + 20];
    fill([attack_t0 attack_tend attack_tend attack_t0], ...
         [yl(1) yl(1) yl(2) yl(2)], col_atk, ...
         'EdgeColor', 'none', 'FaceAlpha', 0.5);
    text(attack_t0 + 0.5, yl(2) - 40, 'Spoofing attack', ...
         'Color', [0.7 0.1 0.1], 'FontSize', 10);

    plot(t, x_noisy,  '--',  'Color', col_noisy, 'LineWidth', lw - 0.5);
    plot(t, x_KF_std, '-.',  'Color', col_std,   'LineWidth', lw, 'LineStyle', '-.');
    plot(t, x_KF_gate,'-',   'Color', col_gate,  'LineWidth', lw + 0.5);
    plot(t, x_real,   '-',   'Color', col_real,  'LineWidth', lw);

    ylim(yl);
    ylabel('X position (m)', 'FontSize', 11);
    title(sprintf('X position — Standard KF vs Chi²-gated KF under spoofing attack (%s)', ...
          target_car_id), 'FontSize', 12, 'FontWeight', 'bold');
    legend({'Noisy V2V', 'Standard KF (attacked)', 'Gated KF (protected)', 'Real'}, ...
           'Location', 'northwest', 'FontSize', 10);

    % Annotate divergence
    [max_err, idx_max] = max(abs(x_KF_std - x_real));
    if idx_max <= n
        plot(t(idx_max), x_KF_std(idx_max), 'rv', 'MarkerSize', 10, 'LineWidth', 2);
        text(t(idx_max) + 1, x_KF_std(idx_max), ...
             sprintf('Max error: %.1f m', max_err), ...
             'Color', col_std, 'FontSize', 9);
    end

    % =========================================================
    %  PANEL 2 — Velocity
    % =========================================================
    ax2 = subplot(3, 1, 2);
    hold on; box on; grid on;

    ylv = [min([v_real, v_noisy, v_KF_std, v_KF_gate]) - 0.5, ...
           max([v_real, v_noisy, v_KF_std, v_KF_gate]) + 0.5];
    fill([attack_t0 attack_tend attack_tend attack_t0], ...
         [ylv(1) ylv(1) ylv(2) ylv(2)], col_atk, ...
         'EdgeColor', 'none', 'FaceAlpha', 0.5);

    plot(t, v_noisy,  '--',  'Color', col_noisy, 'LineWidth', lw - 0.5);
    plot(t, v_KF_std, '-.',  'Color', col_std,   'LineWidth', lw);
    plot(t, v_KF_gate,'-',   'Color', col_gate,  'LineWidth', lw + 0.5);
    plot(t, v_real,   '-',   'Color', col_real,  'LineWidth', lw);

    ylim(ylv);
    ylabel('Velocity (m/s)', 'FontSize', 11);
    legend({'Noisy V2V', 'Standard KF (attacked)', 'Gated KF (protected)', 'Real'}, ...
           'Location', 'best', 'FontSize', 10);

    % =========================================================
    %  PANEL 3 — NIS time series (filter consistency + anomaly)
    % =========================================================
    ax3 = subplot(3, 1, 3);
    hold on; box on; grid on;

    % NIS trace
    plot(t, NIS, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, ...
         'DisplayName', 'NIS');

    % Gate threshold line
    yline(gate_threshold, 'r--', 'LineWidth', 1.5, ...
          'DisplayName', sprintf('Gate threshold \\gamma = %.3f', gate_threshold));

    % Expected value under H0: E[NIS] = nz = 2
    yline(2, 'b:', 'LineWidth', 1.2, ...
          'DisplayName', 'E[NIS] = n_z = 2  (consistent filter)');

    % Mark rejections (anomalies)
    if any(anomaly)
        scatter(t(anomaly), NIS(anomaly), 40, col_std, '^', 'filled', ...
                'DisplayName', sprintf('Rejected (%d measurements)', sum(anomaly)));
    end

    % Running mean NIS (consistency monitor)
    win = 10;  % window size
    NIS_smooth = movmean(NIS, win);
    plot(t, NIS_smooth, '-', 'Color', col_gate, 'LineWidth', 2, ...
         'DisplayName', sprintf('Running mean NIS (window=%d)', win));

    % Annotations
    ylim([0, max(gate_threshold * 4, max(NIS(NIS < gate_threshold * 5)) * 1.2)]);
    xlabel('Time (s)', 'FontSize', 11);
    ylabel('NIS', 'FontSize', 11);
    title('Normalised Innovation Squared — gate fires when NIS > \gamma', ...
          'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 9);

    % Add attack region
    yl3 = ylim();
    fill([attack_t0 attack_tend attack_tend attack_t0], ...
         [yl3(1) yl3(1) yl3(2) yl3(2)], col_atk, ...
         'EdgeColor', 'none', 'FaceAlpha', 0.5);

    % Text box with filter consistency result
    mean_NIS = mean(NIS(NIS > 0));
    nz = 2;
    consist_str = sprintf('Mean NIS = %.2f   (expected = %d,   ratio = %.2f)', ...
                           mean_NIS, nz, mean_NIS/nz);
    text(2, yl3(2) * 0.92, consist_str, 'FontSize', 9, 'Color', col_gate);

    linkaxes([ax1 ax2 ax3], 'x');
    xlim([t(1) t(end)]);

    %% --- Numerical summary ---
    fprintf('\n===== Security Analysis Summary =====\n');
    fprintf('Attack window:    %.0f – %.0f s\n', attack_t0, attack_tend);
    fprintf('Attack offset:    δX = 50 m, δV = 5 m/s\n\n');

    % NRMSE in attack window
    atk_idx = t >= attack_t0 & t <= attack_tend;
    nrmse_range_std  = range(x_real(atk_idx));
    nrmse_range_gate = range(x_real(atk_idx));

    nrmse_std  = sqrt(mean((x_KF_std(atk_idx)  - x_real(atk_idx)).^2)) / nrmse_range_std;
    nrmse_gate = sqrt(mean((x_KF_gate(atk_idx) - x_real(atk_idx)).^2)) / nrmse_range_gate;

    fprintf('During attack (X position):\n');
    fprintf('  Standard KF NRMSE:   %.4f\n', nrmse_std);
    fprintf('  Gated KF  NRMSE:     %.4f\n', nrmse_gate);
    fprintf('  Improvement:         %.1f%%\n\n', (nrmse_std - nrmse_gate)/nrmse_std * 100);

    fprintf('Gate statistics:\n');
    fprintf('  Total measurements:  %d\n',   sum(NIS > 0));
    fprintf('  Rejected (NIS > γ):  %d\n',   sum(anomaly));
    fprintf('  False alarm rate:    %.1f%%\n', sum(anomaly(~(t >= attack_t0 & t <= attack_tend))) / ...
                                               max(1, sum(~(t >= attack_t0 & t <= attack_tend))) * 100);
    fprintf('  Detection rate:      %.1f%%\n', sum(anomaly(atk_idx)) / max(1, sum(atk_idx)) * 100);
    fprintf('  Mean NIS (all):      %.3f  (expected = %d)\n', mean_NIS, nz);
    fprintf('=====================================\n\n');

end