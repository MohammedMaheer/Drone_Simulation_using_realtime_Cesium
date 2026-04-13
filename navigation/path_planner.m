function smoothed_wps = path_planner(waypoints, method, num_points)
% PATH_PLANNER  Generates smooth paths between waypoints.
%
%   smoothed_wps = path_planner(waypoints, method, num_points)
%
%   Inputs:
%     waypoints  - [Nx4] raw waypoints [x, y, z_ned, yaw]
%     method     - 'linear', 'spline', or 'minimum_snap' (default: 'spline')
%     num_points - Number of interpolation points (default: 200)
%
%   Outputs:
%     smoothed_wps - [Mx4] smoothed waypoints [x, y, z_ned, yaw]

    if nargin < 2 || isempty(method)
        method = 'spline';
    end
    if nargin < 3 || isempty(num_points)
        num_points = 200;
    end

    N = size(waypoints, 1);
    if N < 2
        smoothed_wps = waypoints;
        return;
    end

    % Cumulative distance as parameterization
    dists = [0; cumsum(sqrt(sum(diff(waypoints(:,1:3)).^2, 2)))];

    % Normalized parameter
    t_raw = dists / dists(end);
    t_fine = linspace(0, 1, num_points)';

    switch lower(method)
        case 'linear'
            x_interp = interp1(t_raw, waypoints(:,1), t_fine, 'linear');
            y_interp = interp1(t_raw, waypoints(:,2), t_fine, 'linear');
            z_interp = interp1(t_raw, waypoints(:,3), t_fine, 'linear');
            yaw_interp = interp1(t_raw, waypoints(:,4), t_fine, 'linear');

        case 'spline'
            x_interp = interp1(t_raw, waypoints(:,1), t_fine, 'pchip');
            y_interp = interp1(t_raw, waypoints(:,2), t_fine, 'pchip');
            z_interp = interp1(t_raw, waypoints(:,3), t_fine, 'pchip');
            yaw_interp = interp1(t_raw, unwrap(waypoints(:,4)), t_fine, 'pchip');

        case 'minimum_snap'
            % Minimum snap trajectory (polynomial segments)
            [x_interp, y_interp, z_interp] = min_snap_trajectory(...
                waypoints(:,1:3), t_fine, dists(end));
            yaw_interp = interp1(t_raw, unwrap(waypoints(:,4)), t_fine, 'pchip');

        otherwise
            error('Unknown path planning method: %s', method);
    end

    smoothed_wps = [x_interp, y_interp, z_interp, yaw_interp];

end


function [x, y, z] = min_snap_trajectory(positions, t_fine, total_dist)
% Simplified minimum snap: fit 5th-order polynomial per segment
    N = size(positions, 1);
    n_pts = length(t_fine);
    x = zeros(n_pts, 1);
    y = zeros(n_pts, 1);
    z = zeros(n_pts, 1);

    seg_boundaries = linspace(0, 1, N);

    for i = 1:N-1
        % Find points in this segment
        mask = t_fine >= seg_boundaries(i) & t_fine <= seg_boundaries(i+1);
        if i == N-1
            mask = t_fine >= seg_boundaries(i);
        end

        t_seg = (t_fine(mask) - seg_boundaries(i)) / (seg_boundaries(i+1) - seg_boundaries(i));

        % Hermite blending (smooth)
        h00 = 2*t_seg.^3 - 3*t_seg.^2 + 1;
        h01 = -2*t_seg.^3 + 3*t_seg.^2;

        x(mask) = h00 .* positions(i,1) + h01 .* positions(min(i+1,N),1);
        y(mask) = h00 .* positions(i,2) + h01 .* positions(min(i+1,N),2);
        z(mask) = h00 .* positions(i,3) + h01 .* positions(min(i+1,N),3);
    end
end
