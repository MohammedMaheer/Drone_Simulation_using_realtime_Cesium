function flight_analysis(fl)
% FLIGHT_ANALYSIS  HD 12-panel post-flight analysis dashboard with auto-save.
%
%   flight_analysis(flight_log)
%
%   Opens a full-screen 12-panel figure with detailed flight metrics,
%   auto-saves flight data (.mat) and high-resolution plots (.png) to
%   the flight_logs/ directory.
%
%   Panels:
%     1-2. 3D Flight Path (large, altitude-colored with ground plane)
%     3.   Altitude vs Time (actual + target + error shading)
%     4.   Velocity Components (horizontal, vertical, 3D with fill)
%     5.   Attitude Angles (roll, pitch, yaw)
%     6.   Angular Rates (p, q, r)
%     7.   Power & Battery SOC (dual-axis with fill)
%     8.   G-Force & Vibration (with peak annotation)
%     9.   Motor RPMs (all motors overlaid)
%    10.   Motor Balance bar chart (avg RPM per motor)
%    11.   Energy & Distance over time (dual-axis)
%    12.   Flight Envelope (speed vs altitude scatter)
%
%   Plus summary statistics bar and auto-save to flight_logs/

    %% ================================================================
    %% EXTRACT AND PREPROCESS DATA
    %% ================================================================
    t     = fl.time;
    pos   = fl.pos;
    vel   = fl.vel;
    eul   = rad2deg(fl.euler);
    omega = rad2deg(fl.omega);
    rpms  = fl.mot_rpm;
    pwr   = fl.power;
    soc   = fl.soc;
    gf    = fl.gforce;
    tgt   = fl.alt_tgt;
    p     = fl.perf;
    dp    = fl.dp;
    n_mot = fl.n_mot;

    alt = -pos(:,3);
    h_speed = sqrt(vel(:,1).^2 + vel(:,2).^2);
    speed_3d = sqrt(sum(vel.^2, 2));
    vz = -vel(:,3);
    n_pts = length(t);

    % Derived signals
    dt_log = diff(t);
    dt_log(dt_log < 1e-6) = 1e-6;
    energy_cum = cumtrapz(t, pwr) / 3600;     % Cumulative energy [Wh]
    dist_cum = cumtrapz(t, h_speed);           % Cumulative ground distance [m]

    % Smoothed signals (for cleaner plots)
    if n_pts > 20
        k = min(15, max(2, round(n_pts * 0.02)));
        pwr_smooth = movmean(pwr, k);
        gf_smooth = movmean(gf, k);
    else
        pwr_smooth = pwr;
        gf_smooth = gf;
    end

    %% ================================================================
    %% CREATE HD FIGURE (near full-screen, dark theme)
    %% ================================================================
    screen = get(0, 'ScreenSize');
    fig_w = min(screen(3) - 40, 2560);
    fig_h = min(screen(4) - 80, 1440);
    fig = figure('Name', 'POST-FLIGHT ANALYSIS — HD DASHBOARD', ...
        'NumberTitle', 'off', ...
        'Position', [20, 40, fig_w, fig_h], ...
        'Color', [0.04 0.04 0.07], ...
        'Renderer', 'opengl', ...
        'GraphicsSmoothing', 'on');

    %% Color palette
    C.cyan    = [0.25 0.85 1.0];
    C.green   = [0.25 0.95 0.45];
    C.orange  = [1.0 0.55 0.15];
    C.red     = [1.0 0.25 0.30];
    C.yellow  = [1.0 0.88 0.25];
    C.purple  = [0.70 0.45 1.0];
    C.pink    = [1.0 0.45 0.65];
    C.white   = [0.88 0.88 0.92];
    C.gray    = [0.55 0.55 0.60];
    C.dim     = [0.35 0.35 0.40];
    C.bg      = [0.06 0.06 0.09];
    C.grid    = [0.20 0.20 0.25];
    C.text    = [0.75 0.75 0.80];

    %% ================================================================
    %% Panel 1-2: 3D Flight Path (large, left side)
    %% ================================================================
    ax1 = subplot(4, 6, [1 2 3 7 8 9], 'Parent', fig);
    hold(ax1, 'on');
    if n_pts > 2
        patch(ax1, [pos(:,1); NaN], [pos(:,2); NaN], [alt; NaN], [alt; NaN], ...
            'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 3.0);
        % Custom colormap blue->cyan->green->yellow->red
        nc = 256; cmap = zeros(nc,3);
        for ci=1:nc
            f=(ci-1)/(nc-1);
            if f<0.25;     s=f/0.25;     cmap(ci,:)=[0.1 0.2 0.8]*(1-s)+[0.1 0.8 1.0]*s;
            elseif f<0.5;  s=(f-0.25)/0.25; cmap(ci,:)=[0.1 0.8 1.0]*(1-s)+[0.2 0.95 0.3]*s;
            elseif f<0.75; s=(f-0.5)/0.25;  cmap(ci,:)=[0.2 0.95 0.3]*(1-s)+[1.0 0.9 0.1]*s;
            else;          s=(f-0.75)/0.25;  cmap(ci,:)=[1.0 0.9 0.1]*(1-s)+[1.0 0.15 0.1]*s;
            end
        end
        colormap(ax1, cmap);
        cb = colorbar(ax1, 'Color', C.text, 'FontSize', 8);
        cb.Label.String = 'Altitude [m]'; cb.Label.Color = C.text;
    end
    plot3(ax1, pos(1,1), pos(1,2), alt(1), 'o', 'MarkerSize', 14, ...
        'MarkerFaceColor', C.green, 'MarkerEdgeColor', 'w', 'LineWidth', 2);
    plot3(ax1, pos(end,1), pos(end,2), alt(end), 's', 'MarkerSize', 14, ...
        'MarkerFaceColor', C.red, 'MarkerEdgeColor', 'w', 'LineWidth', 2);
    plot(ax1, pos(:,1), pos(:,2), '-', 'Color', [0.4 0.4 0.4 0.3], 'LineWidth', 1.0);
    xr = [min(pos(:,1))-2, max(pos(:,1))+2];
    yr = [min(pos(:,2))-2, max(pos(:,2))+2];
    fill3(ax1, [xr(1) xr(2) xr(2) xr(1)], [yr(1) yr(1) yr(2) yr(2)], ...
        [0 0 0 0], [0.12 0.18 0.08], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
    grid(ax1, 'on'); axis(ax1, 'equal');
    style_axes_hd(ax1, C);
    xlabel(ax1, 'X [m]', 'Color', C.text, 'FontWeight', 'bold');
    ylabel(ax1, 'Y [m]', 'Color', C.text, 'FontWeight', 'bold');
    zlabel(ax1, 'Altitude [m]', 'Color', C.text, 'FontWeight', 'bold');
    title(ax1, '3D FLIGHT PATH', 'Color', C.cyan, 'FontSize', 14, 'FontWeight', 'bold');
    view(ax1, 35, 25);
    legend(ax1, {'Path','Start','End','Ground'}, 'TextColor', C.text, ...
        'Color', [0.10 0.10 0.14], 'EdgeColor', C.dim, 'Location', 'northwest', 'FontSize', 8);

    %% ================================================================
    %% Panel 3: Altitude vs Time (with error band)
    %% ================================================================
    ax2 = subplot(4, 6, [4 5 6], 'Parent', fig);
    hold(ax2, 'on');
    if n_pts > 2
        fill(ax2, [t; flipud(t)], [alt; flipud(tgt)], C.cyan, ...
            'FaceAlpha', 0.12, 'EdgeColor', 'none');
    end
    plot(ax2, t, alt, '-', 'Color', C.cyan, 'LineWidth', 2.0);
    plot(ax2, t, tgt, '--', 'Color', C.orange, 'LineWidth', 1.5);
    grid(ax2, 'on'); style_axes_hd(ax2, C);
    xlabel(ax2, 'Time [s]', 'Color', C.text);
    ylabel(ax2, 'Altitude [m]', 'Color', C.text);
    title(ax2, 'ALTITUDE TRACKING', 'Color', C.cyan, 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax2, {'Error Band','Actual','Target'}, 'TextColor', C.text, ...
        'Color', [0.10 0.10 0.14], 'EdgeColor', C.dim, 'FontSize', 7);
    alt_err = alt - tgt;
    alt_rmse = sqrt(mean(alt_err.^2));
    alt_max_err = max(abs(alt_err));
    text(ax2, 0.98, 0.95, sprintf('RMSE: %.3f m\nMax Err: %.3f m\nMean: %+.3f m', ...
        alt_rmse, alt_max_err, mean(alt_err)), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'FontName', 'Consolas', 'FontSize', 8, 'Color', C.yellow, ...
        'BackgroundColor', [0.08 0.08 0.12 0.85], 'EdgeColor', C.dim);

    %% ================================================================
    %% Panel 4: Velocity Components
    %% ================================================================
    ax3 = subplot(4, 6, [10 11 12], 'Parent', fig);
    hold(ax3, 'on');
    area(ax3, t, h_speed, 'FaceColor', C.green, 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    plot(ax3, t, h_speed, '-', 'Color', C.green, 'LineWidth', 1.8);
    plot(ax3, t, vz, '-', 'Color', C.yellow, 'LineWidth', 1.3);
    plot(ax3, t, speed_3d, ':', 'Color', C.purple, 'LineWidth', 1.2);
    grid(ax3, 'on'); style_axes_hd(ax3, C);
    xlabel(ax3, 'Time [s]', 'Color', C.text);
    ylabel(ax3, 'Speed [m/s]', 'Color', C.text);
    title(ax3, 'VELOCITY', 'Color', C.green, 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax3, {'','Horizontal','Vertical','3D'}, 'TextColor', C.text, ...
        'Color', [0.10 0.10 0.14], 'EdgeColor', C.dim, 'FontSize', 7);

    %% ================================================================
    %% Panel 5: Attitude Angles
    %% ================================================================
    ax4 = subplot(4, 6, 13, 'Parent', fig);
    hold(ax4, 'on');
    plot(ax4, t, eul(:,1), '-', 'Color', C.red, 'LineWidth', 1.5);
    plot(ax4, t, eul(:,2), '-', 'Color', C.cyan, 'LineWidth', 1.5);
    plot(ax4, t, eul(:,3), '-', 'Color', C.gray, 'LineWidth', 0.9);
    yline(ax4, 0, '--', 'Color', C.dim, 'LineWidth', 0.5);
    grid(ax4, 'on'); style_axes_hd(ax4, C);
    xlabel(ax4, 'Time [s]', 'Color', C.text);
    ylabel(ax4, 'Angle [deg]', 'Color', C.text);
    title(ax4, 'ATTITUDE', 'Color', C.orange, 'FontSize', 10, 'FontWeight', 'bold');
    legend(ax4, {'Roll','Pitch','Yaw'}, 'TextColor', C.text, ...
        'Color', [0.10 0.10 0.14], 'EdgeColor', C.dim, 'FontSize', 6);
    roll_rms = sqrt(mean(eul(:,1).^2));
    pitch_rms = sqrt(mean(eul(:,2).^2));
    text(ax4, 0.98, 0.95, sprintf('R_rms:%.1f°\nP_rms:%.1f°', roll_rms, pitch_rms), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'FontName', 'Consolas', 'FontSize', 7, 'Color', C.yellow, ...
        'BackgroundColor', [0.08 0.08 0.12 0.85]);

    %% ================================================================
    %% Panel 6: Angular Rates
    %% ================================================================
    ax5 = subplot(4, 6, 14, 'Parent', fig);
    hold(ax5, 'on');
    plot(ax5, t, omega(:,1), '-', 'Color', C.red, 'LineWidth', 1.2);
    plot(ax5, t, omega(:,2), '-', 'Color', C.cyan, 'LineWidth', 1.2);
    plot(ax5, t, omega(:,3), '-', 'Color', C.gray, 'LineWidth', 0.9);
    yline(ax5, 0, '--', 'Color', C.dim, 'LineWidth', 0.5);
    grid(ax5, 'on'); style_axes_hd(ax5, C);
    xlabel(ax5, 'Time [s]', 'Color', C.text);
    ylabel(ax5, 'Rate [deg/s]', 'Color', C.text);
    title(ax5, 'ANGULAR RATES', 'Color', C.pink, 'FontSize', 10, 'FontWeight', 'bold');
    legend(ax5, {'p','q','r'}, 'TextColor', C.text, ...
        'Color', [0.10 0.10 0.14], 'EdgeColor', C.dim, 'FontSize', 6);

    %% ================================================================
    %% Panel 7: Power & Battery
    %% ================================================================
    ax6 = subplot(4, 6, 15, 'Parent', fig);
    hold(ax6, 'on');
    yyaxis(ax6, 'left');
    area(ax6, t, pwr_smooth, 'FaceColor', C.red, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    plot(ax6, t, pwr_smooth, '-', 'Color', C.red, 'LineWidth', 1.5);
    ylabel(ax6, 'Power [W]', 'Color', C.red);
    set(ax6, 'YColor', C.red);
    yyaxis(ax6, 'right');
    plot(ax6, t, soc * 100, '-', 'Color', C.green, 'LineWidth', 1.8);
    ylabel(ax6, 'SOC [%]', 'Color', C.green);
    set(ax6, 'YColor', C.green);
    grid(ax6, 'on'); style_axes_hd(ax6, C);
    xlabel(ax6, 'Time [s]', 'Color', C.text);
    title(ax6, 'POWER & BATTERY', 'Color', C.red, 'FontSize', 10, 'FontWeight', 'bold');

    %% ================================================================
    %% Panel 8: G-Force
    %% ================================================================
    ax7 = subplot(4, 6, 16, 'Parent', fig);
    hold(ax7, 'on');
    area(ax7, t, gf_smooth, 'FaceColor', C.yellow, 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    plot(ax7, t, gf_smooth, '-', 'Color', C.yellow, 'LineWidth', 1.5);
    yline(ax7, 1, '--', 'Color', C.dim, 'LineWidth', 0.8);
    grid(ax7, 'on'); style_axes_hd(ax7, C);
    xlabel(ax7, 'Time [s]', 'Color', C.text);
    ylabel(ax7, 'G-Force', 'Color', C.text);
    title(ax7, 'G-FORCE', 'Color', C.yellow, 'FontSize', 10, 'FontWeight', 'bold');
    text(ax7, 0.98, 0.95, sprintf('Peak: %.2fG\nAvg: %.2fG', max(gf), mean(gf)), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'FontName', 'Consolas', 'FontSize', 7, 'Color', C.yellow, ...
        'BackgroundColor', [0.08 0.08 0.12 0.85]);

    %% ================================================================
    %% Panel 9: Motor RPMs
    %% ================================================================
    ax8 = subplot(4, 6, [19 20], 'Parent', fig);
    hold(ax8, 'on');
    mot_cmap = [C.red; C.cyan; C.green; C.yellow; C.purple; C.pink; C.orange; C.white];
    leg_labels = cell(1, n_mot);
    for mi = 1:n_mot
        ci = mod(mi-1, size(mot_cmap,1)) + 1;
        plot(ax8, t, rpms(:,mi), '-', 'Color', [mot_cmap(ci,:) 0.85], 'LineWidth', 1.3);
        leg_labels{mi} = sprintf('M%d', mi);
    end
    grid(ax8, 'on'); style_axes_hd(ax8, C);
    xlabel(ax8, 'Time [s]', 'Color', C.text);
    ylabel(ax8, 'RPM', 'Color', C.text);
    title(ax8, 'MOTOR RPMs', 'Color', C.purple, 'FontSize', 10, 'FontWeight', 'bold');
    if n_mot <= 8
        legend(ax8, leg_labels, 'TextColor', C.text, ...
            'Color', [0.10 0.10 0.14], 'EdgeColor', C.dim, 'FontSize', 7, ...
            'Orientation', 'horizontal', 'Location', 'south');
    end

    %% ================================================================
    %% Panel 10: Motor Balance (bar chart)
    %% ================================================================
    ax9 = subplot(4, 6, 21, 'Parent', fig);
    avg_rpms = mean(rpms);
    bar_colors = zeros(n_mot, 3);
    for mi = 1:n_mot
        ci = mod(mi-1, size(mot_cmap,1)) + 1;
        bar_colors(mi,:) = mot_cmap(ci,:);
    end
    b = bar(ax9, 1:n_mot, avg_rpms, 'FaceColor', 'flat', 'EdgeColor', 'none');
    b.CData = bar_colors;
    grid(ax9, 'on'); style_axes_hd(ax9, C);
    xlabel(ax9, 'Motor #', 'Color', C.text);
    ylabel(ax9, 'Avg RPM', 'Color', C.text);
    title(ax9, 'MOTOR BALANCE', 'Color', C.purple, 'FontSize', 10, 'FontWeight', 'bold');
    set(ax9, 'XTick', 1:n_mot);
    rpm_mean_all = mean(avg_rpms);
    if rpm_mean_all > 10
        rpm_spread = (max(avg_rpms) - min(avg_rpms)) / rpm_mean_all * 100;
    else
        rpm_spread = 0;
    end
    text(ax9, 0.98, 0.95, sprintf('Spread: %.1f%%', rpm_spread), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'FontName', 'Consolas', 'FontSize', 7, 'Color', C.yellow, ...
        'BackgroundColor', [0.08 0.08 0.12 0.85]);

    %% ================================================================
    %% Panel 11: Energy & Distance over time
    %% ================================================================
    ax10 = subplot(4, 6, 22, 'Parent', fig);
    hold(ax10, 'on');
    yyaxis(ax10, 'left');
    area(ax10, t, energy_cum, 'FaceColor', C.orange, 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    plot(ax10, t, energy_cum, '-', 'Color', C.orange, 'LineWidth', 1.5);
    ylabel(ax10, 'Energy [Wh]', 'Color', C.orange);
    set(ax10, 'YColor', C.orange);
    yyaxis(ax10, 'right');
    plot(ax10, t, dist_cum, '-', 'Color', C.cyan, 'LineWidth', 1.3);
    ylabel(ax10, 'Distance [m]', 'Color', C.cyan);
    set(ax10, 'YColor', C.cyan);
    grid(ax10, 'on'); style_axes_hd(ax10, C);
    xlabel(ax10, 'Time [s]', 'Color', C.text);
    title(ax10, 'ENERGY & DISTANCE', 'Color', C.orange, 'FontSize', 10, 'FontWeight', 'bold');

    %% ================================================================
    %% Panel 12: Flight Envelope (Speed vs Altitude)
    %% ================================================================
    ax11 = subplot(4, 6, [23 24], 'Parent', fig);
    hold(ax11, 'on');
    scatter(ax11, h_speed, alt, 8, t, 'filled', 'MarkerFaceAlpha', 0.5);
    colormap(ax11, hot(256));
    cb2 = colorbar(ax11, 'Color', C.text, 'FontSize', 7);
    cb2.Label.String = 'Time [s]'; cb2.Label.Color = C.text;
    grid(ax11, 'on'); style_axes_hd(ax11, C);
    xlabel(ax11, 'Horizontal Speed [m/s]', 'Color', C.text);
    ylabel(ax11, 'Altitude [m]', 'Color', C.text);
    title(ax11, 'FLIGHT ENVELOPE', 'Color', C.white, 'FontSize', 10, 'FontWeight', 'bold');
    xline(ax11, dp.max_speed, '--', 'Color', C.red, 'LineWidth', 1, 'Alpha', 0.6);
    text(ax11, 0.97, 0.05, sprintf('V_{max}=%.0fm/s', dp.max_speed), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', ...
        'FontName', 'Consolas', 'FontSize', 7, 'Color', C.red);

    %% ================================================================
    %% SUMMARY STATISTICS BAR
    %% ================================================================
    flight_time = t(end);
    if p.distance_2d > 0.01
        specific_energy = p.energy_total / (p.distance_2d / 1000);
    else
        specific_energy = 0;
    end
    if p.flight_time > 0.5
        avg_power = p.energy_total * 3600 / p.flight_time;
        hover_pct = p.hover_time / p.flight_time * 100;
    else
        avg_power = 0; hover_pct = 0;
    end
    avg_rpm_all = mean(rpms(:));
    if avg_rpm_all > 10
        motor_balance = 100 * (1 - (max(mean(rpms)) - min(mean(rpms))) / avg_rpm_all);
    else
        motor_balance = 100;
    end
    tip2tip_mm = (2*dp.arm_length + dp.d_prop) * 1000;

    summary = sprintf([...
        '  FLIGHT SUMMARY  |  %s  |  %dM  |  %.3f kg  |  Arm %.0fmm  |  Span %.0fmm  |  %.0f"x%.1f" Kv%d  |  %s %.1fAh  |  T/W %.1f:1\n', ...
        '  Duration: %.1fs  |  Airborne: %.1fs  |  Max Alt: %.2fm  |  Max Speed: %.2f m/s  |  Max Climb: +%.2f m/s  |  Max G: %.2fG  |  Alt RMSE: %.3fm\n', ...
        '  Energy: %.2f Wh  |  Avg Pwr: %.0fW  |  Peak: %.0fW  |  Dist: %.1fm  |  Efficiency: %.1f Wh/km  |  SOC End: %.1f%%  |  Balance: %.1f%%  |  Hover: %.0f%%'], ...
        upper(dp.frame_type), n_mot, dp.mass, dp.arm_length*1000, tip2tip_mm, ...
        dp.d_prop/0.0254, dp.d_pitch/0.0254, dp.Kv, dp.battery_type, dp.capacity_Ah, ...
        dp.thrust_to_weight, ...
        flight_time, p.flight_time, p.max_alt, p.max_speed, p.max_climb, p.max_g, alt_rmse, ...
        p.energy_total, avg_power, p.max_power, p.distance_2d, specific_energy, soc(end)*100, ...
        motor_balance, hover_pct);

    annotation(fig, 'textbox', [0.005 0.005 0.99 0.065], ...
        'String', summary, 'FontName', 'Consolas', 'FontSize', 7.5, ...
        'Color', [0.95 0.92 0.30], 'BackgroundColor', [0.06 0.06 0.10], ...
        'EdgeColor', [0.30 0.30 0.40], 'LineWidth', 1.5, ...
        'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left', ...
        'FitBoxToText', 'off', 'Interpreter', 'none');

    % Title bar
    annotation(fig, 'textbox', [0.30 0.96 0.40 0.035], ...
        'String', sprintf('POST-FLIGHT ANALYSIS  —  %s %dM  —  %.1fs Flight', ...
            upper(dp.frame_type), n_mot, flight_time), ...
        'FontName', 'Consolas', 'FontSize', 12, 'FontWeight', 'bold', ...
        'Color', C.cyan, 'BackgroundColor', [0.06 0.06 0.10], ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FitBoxToText', 'off');

    %% ================================================================
    %% AUTO-SAVE flight data and HD dashboard image
    %% ================================================================
    save_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'flight_logs');
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end

    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    base_name = sprintf('%s_%s_%dM', timestamp, dp.frame_type, n_mot);

    % Save flight data
    mat_file = fullfile(save_dir, [base_name '.mat']);
    flight_log = fl; %#ok<NASGU>
    save(mat_file, 'flight_log');

    % Save high-res PNG
    png_file = fullfile(save_dir, [base_name '_dashboard.png']);
    try
        exportgraphics(fig, png_file, 'Resolution', 200);
        fprintf('\n  Dashboard saved: %s\n', png_file);
    catch
        try
            print(fig, png_file, '-dpng', '-r200');
            fprintf('\n  Dashboard saved: %s\n', png_file);
        catch me2
            fprintf('\n  Could not save PNG: %s\n', me2.message);
        end
    end

    fprintf('  Flight data saved: %s\n', mat_file);
    fprintf('\n%s\n', summary);
    fprintf('\n  Replay:  fl = load(''%s''); flight_analysis(fl.flight_log);\n\n', mat_file);
end


%% ================================================================
%% Helper: HD dark-theme axes styling
%% ================================================================
function style_axes_hd(ax, C)
    set(ax, 'Color', C.bg, ...
        'GridColor', C.grid, 'GridAlpha', 0.5, ...
        'MinorGridColor', C.grid, 'MinorGridAlpha', 0.2, ...
        'XColor', C.dim, 'YColor', C.dim, 'ZColor', C.dim, ...
        'FontSize', 7.5, 'FontName', 'Consolas', ...
        'Box', 'on', 'LineWidth', 0.6);
end
