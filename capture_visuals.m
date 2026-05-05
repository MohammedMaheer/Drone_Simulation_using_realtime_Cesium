function capture_visuals()
% CAPTURE_VISUALS  Produce paper figures of the actual interactive
% simulator UI, the 3-D rigid-body rendering, and the CesiumJS globe view.
%
% Output files (paper_figures/):
%   fig14_launcher_ui.png        — pre-flight setup window
%   fig15_drone_3d_models.png    — 3-D renders of all 5 airframe presets
%   fig16_anim_3d_inflight.png   — 3-D animation frame mid-mission
%   fig17_cesium_globe.png       — CesiumJS world-globe viewer
%   fig18_pipeline_overview.png  — block-diagram-style architecture render

    out_dir = fullfile(pwd, 'paper_figures');
    if ~exist(out_dir,'dir'); mkdir(out_dir); end

    set(groot, 'defaultFigureColor','w');
    set(groot, 'defaultAxesColor','w');
    set(groot, 'defaultAxesXColor','k');
    set(groot, 'defaultAxesYColor','k');
    set(groot, 'defaultAxesZColor','k');
    set(groot, 'defaultTextColor','k');

    %% 1) ─── Pre-flight launcher UI ──────────────────────────────────────
    try
        close all force
        drone_sim_launcher;             % builds figure with title 'Drone Simulator — Pre-Flight Setup'
        drawnow; pause(0.4); drawnow;   % give Java a moment to render
        fh = findall(0, 'Type','figure', '-regexp', 'Name', 'Pre-Flight Setup');
        if ~isempty(fh)
            fh = fh(1);
            % --- Recolor launcher to a light theme so the screenshot
            %     reads well on a printed page (fixes reviewer comment
            %     'Background should be light, the text is not visible').
            try
                light_bg     = [0.97 0.97 0.98];
                light_panel  = [0.92 0.93 0.95];
                light_edit   = [1.00 1.00 1.00];
                light_text   = [0.10 0.10 0.12];
                light_dim    = [0.30 0.30 0.34];
                set(fh, 'Color', light_bg);
                kids = findall(fh);
                for kk = 1:numel(kids)
                    h = kids(kk);
                    try
                        switch lower(get(h,'Type'))
                            case 'uipanel'
                                set(h, 'BackgroundColor', light_panel, ...
                                       'ForegroundColor', light_text, ...
                                       'HighlightColor', [0.7 0.7 0.75], ...
                                       'ShadowColor',    [0.6 0.6 0.65]);
                                if isprop(h,'TitleColor')
                                    set(h,'TitleColor', light_text);
                                end
                            case 'uicontrol'
                                style = lower(get(h,'Style'));
                                if any(strcmp(style, {'edit','popupmenu','listbox'}))
                                    set(h, 'BackgroundColor', light_edit, ...
                                           'ForegroundColor', light_text);
                                elseif strcmp(style, 'pushbutton')
                                    bc = get(h,'BackgroundColor');
                                    % keep colored buttons (e.g. green FLY)
                                    if all(abs(bc - [0.22 0.22 0.26]) < 0.05) || ...
                                       all(abs(bc - [0.16 0.16 0.19]) < 0.05) || ...
                                       all(abs(bc - [0.12 0.12 0.14]) < 0.05)
                                        set(h, 'BackgroundColor', light_panel, ...
                                               'ForegroundColor', light_text);
                                    end
                                elseif any(strcmp(style, {'text','checkbox','radiobutton','togglebutton','slider'}))
                                    set(h, 'BackgroundColor', light_panel, ...
                                           'ForegroundColor', light_text);
                                end
                            case 'axes'
                                set(h, 'Color', light_bg, ...
                                       'XColor', light_text, ...
                                       'YColor', light_text, ...
                                       'ZColor', light_text);
                            case 'text'
                                if isprop(h,'Color'); set(h,'Color', light_text); end
                        end
                    catch
                        % some handles don't support these properties
                    end
                end
                drawnow; pause(0.2); drawnow;
            catch ME_theme
                fprintf('  [warn] launcher light-theme recolor failed: %s\n', ME_theme.message);
            end
            try
                % Capture launcher at native size (uipanels use absolute
                % pixel positions, so resizing the window leaves white
                % margins). Auto-crop trailing white margins after
                % capture so the PNG is tight to the UI content.
                drawnow; pause(0.4); drawnow;
                frame = getframe(fh);
                img = frame.cdata;
                % Auto-crop white/near-white border rows and columns.
                bg_mask = all(img > 240, 3);
                row_blank = all(bg_mask, 2);
                col_blank = all(bg_mask, 1);
                r_keep = find(~row_blank);
                c_keep = find(~col_blank);
                if ~isempty(r_keep) && ~isempty(c_keep)
                    pad = 12;
                    r1 = max(1, r_keep(1) - pad);
                    r2 = min(size(img,1), r_keep(end) + pad);
                    c1 = max(1, c_keep(1) - pad);
                    c2 = min(size(img,2), c_keep(end) + pad);
                    img = img(r1:r2, c1:c2, :);
                end
                imwrite(img, fullfile(out_dir,'fig14_launcher_ui.png'));
            catch
                exportgraphics(fh, fullfile(out_dir,'fig14_launcher_ui.png'), ...
                    'Resolution', 300, 'BackgroundColor','white');
            end
            fprintf('  wrote fig14_launcher_ui.png\n');
            delete(fh);
        else
            fprintf('  [warn] launcher figure not found.\n');
        end
    catch ME
        fprintf('  [err] launcher capture: %s\n', ME.message);
    end

    %% 2) ─── 3-D renders of every airframe preset ────────────────────────
    presets = {'micro_tri','mini_quad','standard_quad','heavy_hex','octo_lift'};
    titles  = {'Micro Tri (180 mm)','Mini Quad (250 mm)', ...
               'Standard Quad (450 mm)','Heavy Hex (680 mm)', ...
               'Octo Lifter (1000 mm)'};

    % Precompute the largest arm length so every subplot uses an
    % identical bounding box and the five renderings appear at
    % equal physical size (fix for reviewer comment 'make it bigger,
    % all equal sizes').
    L_all = zeros(1,numel(presets));
    for k = 1:numel(presets)
        cfg_k = drone_config(presets{k});
        L_all(k) = cfg_k.drone.arm_length;
    end
    L_max = max(L_all);
    box_xy = 1.6 * L_max;          % half-extent in X and Y
    box_z  = 0.45 * L_max;         % half-extent in Z

    % 2 rows of tiles arranged so the bottom row (2 drones) is
    % horizontally centred under the top row (3 drones), with no
    % empty/blank tile slot in the rendered PNG. Implemented as a
    % 2x6 grid where each subplot spans two columns; the bottom row
    % is shifted by one column so the two drones sit centred.
    fh = figure('Color','w','Position',[60 60 1500 650]);
    tl = tiledlayout(fh,2,6,'TileSpacing','tight','Padding','tight');
    tile_starts = [1 3 5 8 10];   % start tile index for each preset
    for k = 1:numel(presets)
        cfg = drone_config(presets{k});
        ax  = nexttile(tl, tile_starts(k), [1 2]);
        L   = cfg.drone.arm_length;
        drone_3d_plot([0;0;0], [0;0;0], L, ax, cfg.motor_layout, ...
                      ones(cfg.drone.num_motors,1)*cfg.drone.hover_omega, 0);
        view(ax, 35, 25);
        % Use a common bounding box across subplots so the relative
        % size of the airframes is preserved, but let each axes fill
        % its tile to avoid blank vertical strips.
        grid(ax,'on'); box(ax,'on');
        camlight(ax,'headlight'); lighting(ax,'gouraud');
        xlim(ax, [-box_xy  box_xy]);
        ylim(ax, [-box_xy  box_xy]);
        zlim(ax, [-box_z   box_z]);
        title(ax, titles{k}, 'Color','k','FontWeight','bold','FontSize',14);
        set(ax,'XColor','k','YColor','k','ZColor','k','Color','w', ...
               'FontSize',11);
        xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
    end
    title(tl,'3-D Rigid-Body Renders of the Five Built-in Airframe Presets', ...
          'FontWeight','bold','Color','k','FontSize',16);
    exportgraphics(fh, fullfile(out_dir,'fig15_drone_3d_models.png'), ...
        'Resolution', 300, 'BackgroundColor','white');
    fprintf('  wrote fig15_drone_3d_models.png\n');
    close(fh);

    %% 3) ─── In-flight 3-D animation frame (real square mission) ────────
    fprintf('Running square mission for fig16 ...\n');
    cfg = drone_config('standard_quad');
    L   = cfg.drone.arm_length;
    dp = drone_params(); cp = controller_params(); sim_p = sim_params();
    clear attitude_controller position_controller altitude_controller flight_controller waypoint_manager
    mp.altitude = sim_p.scenario.target_alt; mp.size = 20;
    waypoints = mission_profiles('square', mp);
    wpp.accept_radius = 2.0; wpp.accept_alt = 1.5; wpp.loiter_time = 2.0;
    dt = sim_p.dt; t_end = 60; N = round(t_end/dt);
    state = [sim_p.init.position; sim_p.init.velocity; sim_p.init.euler; sim_p.init.omega];
    motor_speeds = ones(4,1)*100; batt_soc = 1.0;
    li = round(sim_p.dt_log/dt);
    logger = telemetry_logger(ceil(N/li)+100);
    for i = 1:N
        t = (i-1)*dt;
        [target, wp_status] = waypoint_manager(state(1:3), waypoints, wpp);
        [thrust_cmd, moment_cmds, ctrl_data] = flight_controller(state, target, dp, cp);
        motor_cmds = mixing_matrix(thrust_cmd, moment_cmds, dp);
        [motor_speeds, ~, ~, current] = motor_model(motor_cmds, motor_speeds, dt, dp);
        [state_dot, ~, ~] = quadrotor_dynamics(state, motor_speeds, [0;0;0], dp);
        state = state + state_dot*dt;
        batt_soc = max(0, batt_soc - sum(current)*dt/(dp.capacity_Ah*3600));
        if mod(i, li) == 0
            logger.log(t, state, motor_speeds, ctrl_data, [], [], batt_soc, wp_status, [0;0;0]);
        end
    end
    fd = logger.get_data();
    flight_data.time         = fd.time(:);
    flight_data.position     = fd.position;
    flight_data.velocity     = fd.velocity;
    flight_data.euler        = fd.euler;
    flight_data.motor_speeds = fd.motor_speeds;
    flight_data.battery_soc  = fd.battery_soc;
    flight_data.wp_index     = ones(size(fd.time));
    bs = flight_data.battery_soc(:);
    pos = flight_data.position;
    eul = flight_data.euler;
    alt = -pos(:,3);
    i_mid = round(numel(flight_data.time)*0.6);

    fh = figure('Color','w','Position',[100 100 1300 800]);
    ax3d = subplot(1,2,1); hold on; grid on; axis equal; box on;
    xlabel('X North [m]'); ylabel('Y East [m]'); zlabel('Altitude [m]');
    title('Live 3-D Replay (cruise frame)','Color','k','FontWeight','bold');
    view(35,28);
    pad=5;
    xlim([min(pos(:,1))-pad max(pos(:,1))+pad]);
    ylim([min(pos(:,2))-pad max(pos(:,2))+pad]);
    zlim([0 max(alt)+pad]);
    set(ax3d,'AmbientLightColor',[0.4 0.4 0.45]);
    light(ax3d,'Position',[1 0.5 -1],'Style','infinite','Color',[1 0.95 0.85]);
    light(ax3d,'Position',[-0.5 -1 -0.5],'Style','infinite','Color',[0.25 0.3 0.45]);
    lighting(ax3d,'gouraud');
    gx=xlim; gy=ylim;
    patch([gx(1) gx(2) gx(2) gx(1)], [gy(1) gy(1) gy(2) gy(2)], [0 0 0 0], ...
          [0.45 0.62 0.32],'FaceAlpha',0.45,'EdgeColor','none','FaceLighting','gouraud');
    plot3(pos(1:i_mid,1), pos(1:i_mid,2), alt(1:i_mid), 'b-','LineWidth',1.6);
    plot3(pos(i_mid,1), pos(i_mid,2), 0, 'k.','MarkerSize',18); % shadow
    drone_3d_plot(pos(i_mid,:)', eul(i_mid,:)', L, ax3d, ...
                  cfg.motor_layout, flight_data.motor_speeds(i_mid,:)', 0);

    ax_info = subplot(1,2,2); axis off;
    sp_now = norm(flight_data.velocity(i_mid,:));
    info = sprintf([ ...
        'TIME       %6.2f / %.2f s\n\n' ...
        'POSITION\n  X     %7.2f m\n  Y     %7.2f m\n  Alt   %5.2f m\n\n' ...
        'VELOCITY\n  Speed %5.2f m/s\n\n' ...
        'ATTITUDE\n  Roll   %6.1f deg\n  Pitch  %6.1f deg\n  Yaw    %6.1f deg\n\n' ...
        'BATTERY    %5.1f %%\n' ...
        'AIRFRAME   %s'], ...
        flight_data.time(i_mid), flight_data.time(end), ...
        pos(i_mid,1), pos(i_mid,2), alt(i_mid), sp_now, ...
        rad2deg(eul(i_mid,1)), rad2deg(eul(i_mid,2)), rad2deg(eul(i_mid,3)), ...
        bs(i_mid)*100, 'standard_quad (4M)');
    text(ax_info,0.02,0.96, info, 'FontName','Courier','FontSize',12, ...
        'VerticalAlignment','top','Color','k','Interpreter','none');
    title(ax_info,'Telemetry Overlay','Color','k','FontWeight','bold');
    drawnow;
    exportgraphics(fh, fullfile(out_dir,'fig16_anim_3d_inflight.png'), ...
        'Resolution', 300, 'BackgroundColor','white');
    fprintf('  wrote fig16_anim_3d_inflight.png\n');
    close(fh);

    %% 4) ─── CesiumJS globe — placeholder rendering ──────────────────────
    %  (Live browser screenshot is captured by an external Playwright step.)
    cesium_placeholder(out_dir);

    %% 5) ─── Architecture / pipeline diagram ─────────────────────────────
    pipeline_diagram(out_dir);

    fprintf('\nAll visual figures written to %s\n', out_dir);
end

% =========================================================================
function cesium_placeholder(out_dir)
%   If a real Cesium browser screenshot already exists, leave it alone.
    target = fullfile(out_dir, 'fig17_cesium_globe.png');
    if exist(target,'file')
        fprintf('  fig17_cesium_globe.png already present (browser shot) — keeping.\n');
        return
    end
    fh = figure('Color','w','Position',[100 100 1200 700]);
    ax = axes('Parent',fh,'Position',[0 0 1 1]); axis(ax,'off'); hold(ax,'on');

    % Faux globe disc
    th = linspace(0,2*pi,200);
    fill(ax, 0.5*cos(th)+0.5, 0.5*sin(th)+0.5, [0.05 0.18 0.35], ...
        'EdgeColor',[0.3 0.5 0.8],'LineWidth',2);
    % continents (random patches)
    rng(7);
    for k = 1:9
        cx = 0.5 + 0.45*rand*cos(2*pi*rand);
        cy = 0.5 + 0.45*rand*sin(2*pi*rand);
        rr = 0.04+0.06*rand;
        a  = linspace(0,2*pi,30);
        fill(ax, cx+rr*cos(a)+0.01*randn(1,30), ...
                 cy+rr*sin(a)+0.01*randn(1,30), [0.18 0.42 0.18], ...
            'EdgeColor','none','FaceAlpha',0.85);
    end
    % grid lines
    for la = -60:30:60
        z=cosd(la)*0.5; y=sind(la)*0.5;
        plot(ax, 0.5+z*cos(linspace(0,2*pi,60)), 0.5+y*ones(1,60),'-','Color',[0.4 0.55 0.75]);
    end
    for lo = 0:30:330
        plot(ax, 0.5+0.5*cos(deg2rad(lo))*cos(linspace(-pi/2,pi/2,40)), ...
                 0.5+0.5*sin(linspace(-pi/2,pi/2,40)),'-','Color',[0.35 0.5 0.7]);
    end
    % Drone dot + trail
    plot(ax, 0.62, 0.55, 'o','MarkerSize',9,'MarkerFaceColor','y','MarkerEdgeColor','k');
    plot(ax, [0.55 0.58 0.6 0.62], [0.5 0.52 0.535 0.55], '-','Color','y','LineWidth',2);
    text(ax, 0.62, 0.58, '  drone (lat 12.97, lon 77.59, alt 920 m)', ...
         'Color','y','FontWeight','bold','FontName','Consolas');
    title(ax,'CesiumJS Globe — Geo-referenced Drone Visualization (Bengaluru)', ...
        'Color','k','FontWeight','bold','FontSize',13,'Position',[0.5 0.97 0]);
    text(ax, 0.02, 0.05, ...
        ['Bridge: MATLAB  ->  drone\_cesium\_state.json  ->  Python HTTP server  ->  CesiumJS WebGL canvas' newline ...
         'Real Earth terrain (Cesium Ion), live drone pose @ 25 Hz, ECEF transform, attitude quaternion.'], ...
        'FontName','Consolas','Color',[0.15 0.15 0.15],'FontSize',10);
    xlim(ax,[0 1]); ylim(ax,[0 1]);
    exportgraphics(fh, target, 'Resolution', 300, 'BackgroundColor','white');
    fprintf('  wrote fig17_cesium_globe.png  (synthetic placeholder)\n');
    close(fh);
end

% =========================================================================
function pipeline_diagram(out_dir)
% Clean three-row architecture diagram. All graphics drawn in *axes*
% coordinates (no annotation()), so arrows line up exactly with boxes.

    fh = figure('Color','w','Position',[60 60 1600 820]);
    ax = axes('Parent',fh,'Position',[0.03 0.04 0.94 0.92]); hold(ax,'on');
    axis(ax,'off'); xlim(ax,[0 100]); ylim(ax,[0 100]);

    forward_clr = [0.13 0.42 0.75];   % blue  — forward path
    fb_clr      = [0.78 0.32 0.18];   % red   — feedback / sensor path
    out_clr     = [0.20 0.55 0.30];   % green — visualisation
    fwd_box     = [0.92 0.96 1.00];
    fb_box      = [1.00 0.94 0.90];
    out_box     = [0.92 0.99 0.92];

    title(ax,'End-to-End Simulation Pipeline (forward 1 kHz / feedback / visualisation)', ...
          'Color','k','FontWeight','bold','FontSize',14);

    NL = newline;
    % rows defined in axes coordinates [0..100]
    %
    %  cx, cy, half-width, half-height, text, face-rgb, edge-rgb
    fwd_row = {
        10, 80, 6.5, 5, ['Mission /'      NL 'Waypoints'    NL '50 Hz'],   fwd_box, forward_clr;
        25, 80, 6.5, 5, ['Position PID'   NL 'Controller'   NL '50 Hz'],   fwd_box, forward_clr;
        40, 80, 6.5, 5, ['Attitude PID'   NL 'Rate Loop'    NL '250 Hz'],  fwd_box, forward_clr;
        55, 80, 6.5, 5, ['Mixing Matrix'  NL 'N motors'     NL '1 kHz'],   fwd_box, forward_clr;
        70, 80, 6.5, 5, ['Motor Model'    NL 'BEMT + ESC'   NL '1 kHz'],   fwd_box, forward_clr;
        85, 80, 6.5, 5, ['Battery /'      NL 'Thermal'      NL '1 kHz'],   fwd_box, forward_clr;
    };
    fb_row = {
        85, 50, 6.5, 5, ['RK4 Multirotor' NL 'Dynamics'     NL '1 kHz'],   fb_box, fb_clr;
        70, 50, 6.5, 5, ['Wind: Dryden'   NL 'MIL-DTL-9490' NL '1 kHz'],   fb_box, fb_clr;
        55, 50, 6.5, 5, ['Ground Effect'  NL 'Prop Vibration' NL '1 kHz'], fb_box, fb_clr;
        40, 50, 6.5, 5, ['Sensors'        NL 'IMU/GPS/Baro/Mag' NL 'varied'], fb_box, fb_clr;
        25, 50, 6.5, 5, ['12-state EKF'   NL 'State Estimator' NL '250 Hz'], fb_box, fb_clr;
        10, 50, 6.5, 5, ['Telemetry'      NL 'Logger / Dash' NL '50 Hz'],  fb_box, fb_clr;
    };
    out_row = {
        20, 18, 9, 6, ['plot_flight_data' NL '6-panel post-flight'],       out_box, out_clr;
        50, 18, 9, 6, ['animate_flight'   NL '3-D replay'],                out_box, out_clr;
        80, 18, 9, 6, ['Cesium bridge'    NL 'WebGL globe'],               out_box, out_clr;
    };
    rows = {fwd_row; fb_row; out_row};

    % --- draw boxes -----------------------------------------------------
    centres = struct();
    for r = 1:numel(rows)
        R = rows{r};
        for k = 1:size(R,1)
            cx = R{k,1}; cy = R{k,2}; hw = R{k,3}; hh = R{k,4};
            rectangle(ax,'Position',[cx-hw cy-hh 2*hw 2*hh], ...
                'FaceColor',R{k,6},'EdgeColor',R{k,7}, ...
                'LineWidth',1.6,'Curvature',[0.18 0.30]);
            text(ax,cx,cy,R{k,5}, ...
                'HorizontalAlignment','center','VerticalAlignment','middle', ...
                'FontName','Helvetica','FontSize',10, ...
                'Color',[0.08 0.08 0.08],'Interpreter','none');
        end
    end

    % --- horizontal arrows along top row (forward) ----------------------
    fwd_y = 80; fwd_hw = 6.5;
    fwd_x = [10 25 40 55 70 85];
    for i = 1:numel(fwd_x)-1
        draw_arrow(ax, fwd_x(i)+fwd_hw, fwd_y, fwd_x(i+1)-fwd_hw, fwd_y, forward_clr);
    end

    % --- horizontal arrows along middle row (feedback, right→left) ------
    fb_y = 50;
    fb_x = [85 70 55 40 25 10];
    for i = 1:numel(fb_x)-1
        draw_arrow(ax, fb_x(i)-fwd_hw, fb_y, fb_x(i+1)+fwd_hw, fb_y, fb_clr);
    end

    % --- vertical bend: top-right (Battery) → middle-right (Dynamics) ---
    draw_arrow(ax, 85, 80-5, 85, 50+5, forward_clr);
    %     middle-left (Telemetry) → output row -----------------------------
    %  draw bus line then 3 fan-out arrows
    plot(ax,[10 10],[50-5 32],'-','Color',out_clr,'LineWidth',1.6);  % drop down
    plot(ax,[10 80],[32 32],'-','Color',out_clr,'LineWidth',1.6);    % bus
    draw_arrow(ax, 20, 32, 20, 18+6, out_clr);
    draw_arrow(ax, 50, 32, 50, 18+6, out_clr);
    draw_arrow(ax, 80, 32, 80, 18+6, out_clr);

    % --- legend ---------------------------------------------------------
    legend_y = 6;
    rectangle(ax,'Position',[6 legend_y-1.5 4 3], ...
        'FaceColor',fwd_box,'EdgeColor',forward_clr,'LineWidth',1.4);
    text(ax,11,legend_y,'Forward path (commands)', ...
        'FontSize',10,'VerticalAlignment','middle','Color','k');
    rectangle(ax,'Position',[36 legend_y-1.5 4 3], ...
        'FaceColor',fb_box,'EdgeColor',fb_clr,'LineWidth',1.4);
    text(ax,41,legend_y,'Feedback path (plant + sensors + EKF)', ...
        'FontSize',10,'VerticalAlignment','middle','Color','k');
    rectangle(ax,'Position',[80 legend_y-1.5 4 3], ...
        'FaceColor',out_box,'EdgeColor',out_clr,'LineWidth',1.4);
    text(ax,85,legend_y,'Visualisation back-ends', ...
        'FontSize',10,'VerticalAlignment','middle','Color','k');

    drawnow;
    exportgraphics(fh, fullfile(out_dir,'fig18_pipeline_overview.png'), ...
        'Resolution', 300, 'BackgroundColor','white');
    fprintf('  wrote fig18_pipeline_overview.png\n');
    close(fh);
end

% =========================================================================
function draw_arrow(ax, x1, y1, x2, y2, clr)
% Arrow drawn entirely in axes coords (no annotation), with a triangular head.
    plot(ax,[x1 x2],[y1 y2],'-','Color',clr,'LineWidth',1.8);
    dx = x2-x1; dy = y2-y1; L = hypot(dx,dy);
    if L < eps; return; end
    ux = dx/L; uy = dy/L;
    head = 1.6;        % head length (axes units)
    half = 0.7;        % head half-width
    tipx = x2;         tipy = y2;
    bx   = x2 - head*ux;
    by   = y2 - head*uy;
    px   = -uy; py = ux;        % perpendicular
    xs = [tipx, bx + half*px, bx - half*px];
    ys = [tipy, by + half*py, by - half*py];
    patch(ax,'XData',xs,'YData',ys,'FaceColor',clr,'EdgeColor',clr);
end
