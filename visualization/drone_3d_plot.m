function h = drone_3d_plot(position, euler, arm_length, ax, motor_layout, motor_speeds, prop_angle)
% DRONE_3D_PLOT  Render a realistic 3D multirotor at given pose.
%
%   h = drone_3d_plot(position, euler, arm_length, ax, motor_layout, motor_speeds, prop_angle)
%
%   Creates solid 3D geometry: body plate, arm tubes, motor cylinders,
%   spinning propeller blades, blur discs, landing gear, and LEDs.
%
%   Inputs:
%     position     - [3x1] position [x; y; z] NED (z positive down)
%     euler        - [3x1] Euler angles [phi; theta; psi] [rad]
%     arm_length   - Motor arm length [m]
%     ax           - Axes handle (optional)
%     motor_layout - (optional) struct from drone_config
%     motor_speeds - (optional) [Nx1] motor speeds for blur opacity
%     prop_angle   - (optional) scalar blade rotation angle [rad]
%
%   Outputs:
%     h - Struct of all patch/line handles

    if nargin < 3; arm_length = 0.23; end
    if nargin < 4; ax = gca; end
    if nargin < 5; motor_layout = []; end
    if nargin < 6; motor_speeds = []; end
    if nargin < 7; prop_angle = 0; end

    L = arm_length;

    % Build default layout if not provided
    if isempty(motor_layout)
        s45 = sqrt(2)/2;
        motor_layout.positions = L * [s45 s45 0; -s45 -s45 0; s45 -s45 0; -s45 s45 0];
        motor_layout.spin_dirs = [-1; -1; 1; 1];
        motor_layout.arm_pairs = [1 2; 3 4];
    end

    n_mot = size(motor_layout.positions, 1);
    prop_r = L * 0.8;

    phi = euler(1); theta = euler(2); psi = euler(3);
    R = euler_to_R(phi, theta, psi);
    pos = position;
    F6 = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];

    axes(ax); hold on;

    %% --- Central body plate ---
    bx = L*0.28; by = L*0.20; bz = L*0.04;
    h.body = patch('Vertices', xfm_v(box_v(bx,by,bz), R, pos), 'Faces', F6, ...
        'FaceColor', [0.18 0.18 0.22], 'EdgeColor', [0.3 0.3 0.35], ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.5, ...
        'DiffuseStrength', 0.7, 'SpecularStrength', 0.3, ...
        'BackFaceLighting', 'reverselit');

    %% --- Electronics top plate (PCB) ---
    tx = L*0.18; ty = L*0.14; tz = L*0.015;
    top_verts = box_v(tx, ty, tz);
    top_verts(:,3) = top_verts(:,3) - bz - tz;
    h.top = patch('Vertices', xfm_v(top_verts, R, pos), 'Faces', F6, ...
        'FaceColor', [0.12 0.42 0.12], 'EdgeColor', [0.18 0.48 0.18], ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.5, ...
        'DiffuseStrength', 0.6);

    %% --- Battery pack (underneath) ---
    btx = L*0.15; bty = L*0.08; btz = L*0.04;
    bat_verts = box_v(btx, bty, btz);
    bat_verts(:,3) = bat_verts(:,3) + bz + btz;
    h.batt = patch('Vertices', xfm_v(bat_verts, R, pos), 'Faces', F6, ...
        'FaceColor', [0.18 0.18 0.55], 'EdgeColor', [0.25 0.25 0.6], ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.5, ...
        'DiffuseStrength', 0.6);

    %% --- Arms (solid tubes) ---
    arm_w = L*0.03; arm_h = L*0.02;
    for i = 1:n_mot
        mp = motor_layout.positions(i,:);
        h.arms(i) = patch('Vertices', xfm_v(tube_v([0 0 0], mp, arm_w, arm_h), R, pos), ...
            'Faces', F6, 'FaceColor', [0.22 0.22 0.25], 'EdgeColor', [0.3 0.3 0.33], ...
            'FaceLighting', 'gouraud', 'AmbientStrength', 0.5, ...
            'DiffuseStrength', 0.7, 'SpecularStrength', 0.2);
    end

    %% --- Motor housings (cylinders) ---
    motor_r = L*0.045; motor_h_val = L*0.065; n_seg = 10;
    for i = 1:n_mot
        mp = motor_layout.positions(i,:);
        [cv, cf] = cyl_v(motor_r, motor_h_val, n_seg, mp);
        if motor_layout.spin_dirs(i) < 0
            clr = [0.55 0.08 0.08];
        else
            clr = [0.08 0.15 0.55];
        end
        h.motors(i) = patch('Vertices', xfm_v(cv, R, pos), 'Faces', cf, ...
            'FaceColor', clr, 'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud', 'AmbientStrength', 0.4, ...
            'DiffuseStrength', 0.8, 'SpecularStrength', 0.6, ...
            'SpecularExponent', 25);
    end

    %% --- Propeller blades (2 per motor, with rotation) ---
    chord_root = prop_r*0.10; chord_tip = prop_r*0.05; hub_r = prop_r*0.10;
    blade_tmpl = [hub_r, chord_root/2, -0.003; hub_r, -chord_root/2, -0.003;
                  prop_r, -chord_tip/2, -0.003; prop_r, chord_tip/2, -0.003];
    for i = 1:n_mot
        mc = motor_layout.positions(i,:);
        spin_a = prop_angle * motor_layout.spin_dirs(i);
        for b = 1:2
            a = spin_a + (b-1)*pi;
            ca = cos(a); sa = sin(a);
            R_z = [ca -sa 0; sa ca 0; 0 0 1];
            bv = (R_z * blade_tmpl')' + mc;
            h.blades(i,b) = patch('Vertices', xfm_v(bv, R, pos), ...
                'Faces', [1 2 3 4], 'FaceColor', [0.25 0.25 0.27], ...
                'EdgeColor', [0.15 0.15 0.17], 'FaceAlpha', 0.9, ...
                'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
        end
    end

    %% --- Prop blur discs (opacity scales with RPM) ---
    n_disc = 24; theta_d = linspace(0, 2*pi, n_disc)';
    for i = 1:n_mot
        mp = motor_layout.positions(i,:);
        dv = [mp(1)+prop_r*cos(theta_d), mp(2)+prop_r*sin(theta_d), ...
              ones(n_disc,1)*(mp(3)-0.004)];
        if ~isempty(motor_speeds) && i <= length(motor_speeds) && max(abs(motor_speeds)) > 0
            rpm_frac = min(1, abs(motor_speeds(i)) / max(abs(motor_speeds)));
        else
            rpm_frac = 0.3;
        end
        if motor_layout.spin_dirs(i) < 0; bc = [0.8 0.25 0.25]; else; bc = [0.25 0.35 0.8]; end
        h.blur(i) = patch('Vertices', xfm_v(dv, R, pos), 'Faces', 1:n_disc, ...
            'FaceColor', bc, 'FaceAlpha', 0.05+0.30*rpm_frac, 'EdgeColor', 'none');
    end

    %% --- Landing gear ---
    leg_drop = L*0.20; leg_spread = L*0.22;
    att = [L*0.15 leg_spread bz; L*0.15 -leg_spread bz;
          -L*0.15 leg_spread bz; -L*0.15 -leg_spread bz];
    feet = [L*0.18 leg_spread*1.3 bz+leg_drop; L*0.18 -leg_spread*1.3 bz+leg_drop;
           -L*0.18 leg_spread*1.3 bz+leg_drop; -L*0.18 -leg_spread*1.3 bz+leg_drop];
    lw = L*0.008; lh = L*0.008;
    for j = 1:4
        h.legs(j) = patch('Vertices', xfm_v(tube_v(att(j,:), feet(j,:), lw, lh), R, pos), ...
            'Faces', F6, 'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
    end
    sw = L*0.01; sh = L*0.006;
    h.skids(1) = patch('Vertices', xfm_v(tube_v(feet(1,:), feet(2,:), sw, sh), R, pos), ...
        'Faces', F6, 'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
    h.skids(2) = patch('Vertices', xfm_v(tube_v(feet(3,:), feet(4,:), sw, sh), R, pos), ...
        'Faces', F6, 'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

    %% --- Forward direction arrow ---
    fwd_pts = xfm_v([0 0 0; L*1.5 0 -0.01], R, pos);
    h.fwd = plot3(fwd_pts(:,1), fwd_pts(:,2), fwd_pts(:,3), ...
        'Color', [0.1 0.85 0.1], 'LineWidth', 2.5);

    %% --- LED indicators (front=white, rear=red) ---
    led_r = L*0.015; led_theta = linspace(0, 2*pi, 8)';
    for i = 1:n_mot
        lp = motor_layout.positions(i,:) * 0.6;
        lv = [lp(1)+led_r*cos(led_theta), lp(2)+led_r*sin(led_theta), ...
              ones(8,1)*(lp(3)-bz-0.002)];
        if motor_layout.positions(i,1) >= 0
            lc = [0.9 0.9 0.9];
        else
            lc = [0.9 0.1 0.1];
        end
        h.leds(i) = patch('Vertices', xfm_v(lv, R, pos), 'Faces', 1:8, ...
            'FaceColor', lc, 'EdgeColor', 'none', 'FaceAlpha', 0.85, ...
            'FaceLighting', 'none');
    end
end


%% ================================================================
%% LOCAL GEOMETRY HELPERS
%% ================================================================
function Vp = xfm_v(Vb, R, pos)
    Vn = (R * Vb')';
    Vp = [Vn(:,1)+pos(1), Vn(:,2)+pos(2), -(Vn(:,3)+pos(3))];
end

function V = box_v(hx, hy, hz)
    V = [hx hy -hz; -hx hy -hz; -hx -hy -hz; hx -hy -hz;
         hx hy  hz; -hx hy  hz; -hx -hy  hz; hx -hy  hz];
end

function V = tube_v(p1, p2, hw, hh)
    d = p2 - p1; len = norm(d);
    if len < 1e-6; V = box_v(hw, hw, hh); return; end
    dx = d/len;
    if abs(dx(3)) < 0.9; up = [0 0 -1]; else; up = [1 0 0]; end
    side = cross(dx, up); side = side/norm(side);
    up2 = cross(side, dx); up2 = up2/norm(up2);
    off = [-hw*side-hh*up2; hw*side-hh*up2; hw*side+hh*up2; -hw*side+hh*up2];
    V = [repmat(p1,4,1)+off; repmat(p2,4,1)+off];
end

function [V, F] = cyl_v(r, h, n, center)
    theta = linspace(0, 2*pi, n+1); theta = theta(1:end-1);
    x = r*cos(theta)+center(1); y = r*sin(theta)+center(2);
    V = [x' y' ones(n,1)*center(3); x' y' ones(n,1)*(center(3)-h)];
    F = zeros(n, 4);
    for i = 1:n; j = mod(i,n)+1; F(i,:) = [i j j+n i+n]; end
end

function R = euler_to_R(phi, theta, psi)
    cphi=cos(phi); sphi=sin(phi); cth=cos(theta); sth=sin(theta);
    cpsi=cos(psi); spsi=sin(psi);
    R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi;
         cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi;
         -sth,     sphi*cth,                cphi*cth               ];
end
