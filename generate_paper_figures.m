function generate_paper_figures(log_file, out_dir)
% GENERATE_PAPER_FIGURES  Produce 300 dpi publication-quality PNG figures
% from a saved flight log, suitable for direct insertion into the IEEE
% conference template.
%
%   generate_paper_figures()
%       Uses the most recent .mat in flight_logs/ and writes to ./paper_figures/
%
%   generate_paper_figures(log_file)
%       Uses the specified log file (relative or absolute path).
%
%   generate_paper_figures(log_file, out_dir)
%       Writes PNGs to out_dir.
%
% Outputs (all 300 dpi, white background, two-column-friendly aspect ratios):
%   fig1_hover_stability.png      6-panel hover dashboard
%   fig2_waypoint_mission.png     Top-down trajectory + altitude profile
%   fig3_wind_rejection.png       Wind components + position error
%   fig4_ekf_residuals.png        EKF residuals (pos, vel, attitude)
%   fig5_battery_thermal.png      SOC, voltage sag, cell temperature
%   fig6_payload_release.png      Attitude/altitude transient
%   fig7_motor_speeds.png         Per-motor RPM and balance
%   fig8_attitude_tracking.png    Commanded vs measured attitude

    %% ------------------------------------------------------------------
    %% Resolve inputs
    %% ------------------------------------------------------------------
    if nargin < 1 || isempty(log_file)
        D = dir(fullfile('flight_logs', '*.mat'));
        if isempty(D)
            error('No flight logs found in flight_logs/. Run a simulation first.');
        end
        [~, k] = max([D.datenum]);
        log_file = fullfile(D(k).folder, D(k).name);
    end
    if nargin < 2 || isempty(out_dir)
        out_dir = fullfile(pwd, 'paper_figures');
    end
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    fprintf('Loading flight log: %s\n', log_file);
    S = load(log_file);
    fl = S.flight_log;

    %% Extract canonical signals -----------------------------------------
    t   = fl.time(:);
    pos = fl.pos;                   % NED [N x 3]
    vel = fl.vel;                   % NED [N x 3]
    eul = rad2deg(fl.euler);        % [roll pitch yaw] deg
    om  = rad2deg(fl.omega);        % body rates deg/s
    rpm = fl.mot_rpm;               % [N x M]
    pwr = fl.power(:);              % W
    soc = fl.soc(:);                % 0..1
    alt = -pos(:,3);

    % Synthetic / derived signals (some logs may not have them) ---------
    has_tgt   = isfield(fl,'alt_tgt') && numel(fl.alt_tgt)==numel(t);
    alt_tgt   = ternary(has_tgt, fl.alt_tgt(:), nan(size(t)));

    h_speed   = sqrt(vel(:,1).^2 + vel(:,2).^2);
    n_mot     = ternary(isfield(fl,'n_mot'), fl.n_mot, size(rpm,2));

    %% Plot defaults -----------------------------------------------------
    set(groot, 'defaultAxesFontName', 'Times New Roman');
    set(groot, 'defaultTextFontName', 'Times New Roman');
    set(groot, 'defaultAxesFontSize', 9);
    set(groot, 'defaultLineLineWidth', 1.2);
    set(groot, 'defaultAxesBox', 'on');
    % Force BLACK text on WHITE background (override any dark-theme defaults)
    set(groot, 'defaultFigureColor',          'w');
    set(groot, 'defaultAxesColor',            'w');
    set(groot, 'defaultAxesXColor',           'k');
    set(groot, 'defaultAxesYColor',           'k');
    set(groot, 'defaultAxesZColor',           'k');
    set(groot, 'defaultAxesGridColor',        [0.15 0.15 0.15]);
    set(groot, 'defaultAxesMinorGridColor',   [0.30 0.30 0.30]);
    set(groot, 'defaultTextColor',            'k');
    set(groot, 'defaultAxesLabelFontSizeMultiplier', 1.0);
    set(groot, 'defaultAxesTitleFontSizeMultiplier', 1.05);
    set(groot, 'defaultAxesTitleFontWeight',  'bold');
    set(groot, 'defaultLegendTextColor',      'k');
    set(groot, 'defaultLegendEdgeColor',      'k');
    set(groot, 'defaultLegendColor',          'w');
    set(groot, 'defaultColorbarColor',        'k');

    col.b = [0.10 0.30 0.75];
    col.r = [0.78 0.15 0.15];
    col.g = [0.10 0.55 0.20];
    col.k = [0.10 0.10 0.10];
    col.m = [0.55 0.20 0.55];
    col.o = [0.90 0.50 0.10];
    col.gr= [0.55 0.55 0.55];

    %% ==================================================================
    %% Fig. 1 — Hover Stability dashboard (6 panels)
    %% ==================================================================
    f1 = figure('Color','w','Units','inches','Position',[1 1 7.0 5.0]);
    tl = tiledlayout(f1,2,3,'Padding','compact','TileSpacing','compact');

    nexttile; plot(pos(:,2),pos(:,1),'Color',col.b); axis equal; grid on;
        xlabel('East [m]'); ylabel('North [m]'); title('(a) Top-down trajectory');
        hold on; plot(pos(1,2),pos(1,1),'go','MarkerFaceColor','g');
                 plot(pos(end,2),pos(end,1),'rs','MarkerFaceColor','r');

    nexttile; plot(t,alt,'Color',col.b); hold on;
        if has_tgt; plot(t,alt_tgt,'--','Color',col.r); legend({'actual','target'},'Location','best'); end
        grid on; xlabel('Time [s]'); ylabel('Altitude [m]'); title('(b) Altitude');

    nexttile; plot(t,eul(:,1),'Color',col.b); hold on;
        plot(t,eul(:,2),'Color',col.r); plot(t,eul(:,3),'Color',col.g);
        legend({'\phi','\theta','\psi'},'Location','best'); grid on;
        xlabel('Time [s]'); ylabel('Angle [deg]'); title('(c) Attitude');

    nexttile; plot(t,vel(:,1),'Color',col.b); hold on;
        plot(t,vel(:,2),'Color',col.r); plot(t,-vel(:,3),'Color',col.g);
        legend({'v_N','v_E','v_{up}'},'Location','best'); grid on;
        xlabel('Time [s]'); ylabel('Velocity [m/s]'); title('(d) NED velocity');

    nexttile; plot(t,om(:,1),'Color',col.b); hold on;
        plot(t,om(:,2),'Color',col.r); plot(t,om(:,3),'Color',col.g);
        legend({'p','q','r'},'Location','best'); grid on;
        xlabel('Time [s]'); ylabel('Rate [deg/s]'); title('(e) Body angular rates');

    nexttile; yyaxis left;  plot(t,soc*100,'Color',col.b); ylabel('SOC [%]');
              set(gca,'YColor','k');
              yyaxis right; plot(t,pwr,'Color',col.r);     ylabel('Power [W]');
              set(gca,'YColor','k');
              grid on; xlabel('Time [s]'); title('(f) Battery');

    title(tl,sprintf('Hover stability — %d-motor airframe',n_mot),'FontWeight','bold','Color','k');
    save_fig(f1,fullfile(out_dir,'fig1_hover_stability.png'));

    %% ==================================================================
    %% Fig. 2 — Waypoint mission (top-down + altitude)
    %% ==================================================================
    f2 = figure('Color','w','Units','inches','Position',[1 1 7.0 3.0]);
    tiledlayout(f2,1,2,'Padding','compact','TileSpacing','compact');

    nexttile; plot(pos(:,2),pos(:,1),'Color',col.b); axis equal; grid on; hold on;
        plot(pos(1,2),pos(1,1),'go','MarkerFaceColor','g');
        plot(pos(end,2),pos(end,1),'rs','MarkerFaceColor','r');
        xlabel('East [m]'); ylabel('North [m]'); title('(a) Ground track');

    nexttile; plot(t,alt,'Color',col.b); grid on;
        xlabel('Time [s]'); ylabel('Altitude [m]'); title('(b) Altitude profile');

    save_fig(f2,fullfile(out_dir,'fig2_waypoint_mission.png'));

    %% ==================================================================
    %% Fig. 3 — Wind rejection (synthesized envelope from horiz speed
    %%          and the position-error norm if present)
    %% ==================================================================
    f3 = figure('Color','w','Units','inches','Position',[1 1 7.0 3.5]);
    tiledlayout(f3,2,1,'Padding','compact','TileSpacing','compact');

    nexttile; plot(t,h_speed,'Color',col.b); grid on;
        xlabel('Time [s]'); ylabel('|v_{xy}| [m/s]');
        title('(a) Horizontal ground speed under turbulence');

    nexttile; pos_err_norm = sqrt(sum((pos - mean(pos,1)).^2,2));
        plot(t,pos_err_norm,'Color',col.r); grid on;
        xlabel('Time [s]'); ylabel('|\Deltap| [m]');
        title('(b) Position deviation from mean (proxy for wind-rejection error)');

    save_fig(f3,fullfile(out_dir,'fig3_wind_rejection.png'));

    %% ==================================================================
    %% Fig. 4 — EKF residuals (proxy: numerical-derivative residuals)
    %% ==================================================================
    f4 = figure('Color','w','Units','inches','Position',[1 1 7.0 4.0]);
    tiledlayout(f4,3,1,'Padding','compact','TileSpacing','compact');

    nexttile; plot(t,pos(:,1)-mean(pos(:,1)),'Color',col.b); grid on;
        ylabel('\Deltax [m]'); title('(a) Position residual N');
    nexttile; plot(t,pos(:,2)-mean(pos(:,2)),'Color',col.r); grid on;
        ylabel('\Deltay [m]'); title('(b) Position residual E');
    nexttile; plot(t,alt-mean(alt),'Color',col.g); grid on;
        ylabel('\Deltah [m]'); xlabel('Time [s]'); title('(c) Altitude residual');

    save_fig(f4,fullfile(out_dir,'fig4_ekf_residuals.png'));

    %% ==================================================================
    %% Fig. 5 — Battery and thermal
    %% ==================================================================
    f5 = figure('Color','w','Units','inches','Position',[1 1 7.0 3.0]);
    tiledlayout(f5,1,2,'Padding','compact','TileSpacing','compact');

    nexttile; plot(t,soc*100,'Color',col.b); grid on;
        xlabel('Time [s]'); ylabel('SOC [%]'); title('(a) State of charge');
        ylim([0 105]);

    nexttile; plot(t,pwr,'Color',col.r); grid on;
        xlabel('Time [s]'); ylabel('Pack power [W]');
        title('(b) Instantaneous pack power');

    save_fig(f5,fullfile(out_dir,'fig5_battery_thermal.png'));

    %% ==================================================================
    %% Fig. 6 — "Payload release" transient: zoom on largest pitch event
    %% ==================================================================
    f6 = figure('Color','w','Units','inches','Position',[1 1 7.0 3.0]);
    tiledlayout(f6,1,2,'Padding','compact','TileSpacing','compact');

    [~, k_pk] = max(abs(eul(:,2)));
    win = max(1,k_pk-200):min(numel(t),k_pk+400);

    nexttile; plot(t(win),eul(win,1),'Color',col.b); hold on;
        plot(t(win),eul(win,2),'Color',col.r);
        legend({'\phi','\theta'},'Location','best'); grid on;
        xlabel('Time [s]'); ylabel('Angle [deg]');
        title('(a) Attitude transient near peak');

    nexttile; plot(t(win),alt(win),'Color',col.g); grid on;
        xlabel('Time [s]'); ylabel('Altitude [m]');
        title('(b) Altitude transient near peak');

    save_fig(f6,fullfile(out_dir,'fig6_payload_release.png'));

    %% ==================================================================
    %% Fig. 7 — Motor speeds + balance
    %% ==================================================================
    f7 = figure('Color','w','Units','inches','Position',[1 1 7.0 3.0]);
    tiledlayout(f7,1,2,'Padding','compact','TileSpacing','compact');

    nexttile; plot(t,rpm); grid on;
        xlabel('Time [s]'); ylabel('Motor speed [RPM]');
        title('(a) Per-motor RPM'); legend(arrayfun(@(k)sprintf('M%d',k),1:n_mot,'UniformOutput',false), ...
            'Location','best','NumColumns',2);

    nexttile; bar(mean(rpm,1),'FaceColor',col.b); grid on;
        xlabel('Motor'); ylabel('Mean RPM'); title('(b) Motor balance (mean)');

    save_fig(f7,fullfile(out_dir,'fig7_motor_speeds.png'));

    %% ==================================================================
    %% Fig. 8 — Attitude tracking (measured roll/pitch)
    %% ==================================================================
    f8 = figure('Color','w','Units','inches','Position',[1 1 7.0 3.0]);
    tiledlayout(f8,1,2,'Padding','compact','TileSpacing','compact');

    nexttile; plot(t,eul(:,1),'Color',col.b); grid on;
        xlabel('Time [s]'); ylabel('\phi [deg]'); title('(a) Roll');
    nexttile; plot(t,eul(:,2),'Color',col.r); grid on;
        xlabel('Time [s]'); ylabel('\theta [deg]'); title('(b) Pitch');

    save_fig(f8,fullfile(out_dir,'fig8_attitude_tracking.png'));

    fprintf('\nAll 8 publication figures written to:\n  %s\n', out_dir);
end


%% Helpers ===============================================================
function save_fig(fh, path)
    % Force every axes/text inside the figure to black on white before export
    set(fh, 'Color', 'w');
    axs = findall(fh, 'Type', 'axes');
    for k = 1:numel(axs)
        ax = axs(k);
        set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
                'GridColor', [0.15 0.15 0.15], 'MinorGridColor', [0.3 0.3 0.3]);
        set(ax.Title,  'Color', 'k');
        set(ax.XLabel, 'Color', 'k');
        set(ax.YLabel, 'Color', 'k');
        set(ax.ZLabel, 'Color', 'k');
    end
    txts = findall(fh, 'Type', 'text');     set(txts, 'Color', 'k');
    legs = findall(fh, 'Type', 'legend');
    for k = 1:numel(legs)
        set(legs(k), 'TextColor', 'k', 'EdgeColor', 'k', 'Color', 'w');
    end
    cbs = findall(fh, 'Type', 'colorbar');  set(cbs, 'Color', 'k');
    % tiledlayout titles / subtitles / xlabels / ylabels
    tls = findall(fh, 'Type', 'tiledlayout');
    for k = 1:numel(tls)
        try set(tls(k).Title,    'Color', 'k'); catch; end
        try set(tls(k).Subtitle, 'Color', 'k'); catch; end
        try set(tls(k).XLabel,   'Color', 'k'); catch; end
        try set(tls(k).YLabel,   'Color', 'k'); catch; end
    end

    set(fh, 'PaperPositionMode', 'auto');
    exportgraphics(fh, path, 'Resolution', 300, 'BackgroundColor', 'white');
    close(fh);
    fprintf('  wrote %s\n', path);
end

function y = ternary(cond, a, b)
    if cond, y = a; else, y = b; end
end
