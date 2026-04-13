function result = coord_transforms(operation, varargin)
% COORD_TRANSFORMS  Coordinate frame transformation utilities.
%
%   v_body = coord_transforms('ned2body', v_ned, euler)
%   v_ned  = coord_transforms('body2ned', v_body, euler)
%   lla    = coord_transforms('ned2lla', pos_ned, lla_ref)
%   ned    = coord_transforms('lla2ned', lla, lla_ref)

    switch lower(operation)
        case 'ned2body'
            v_ned = varargin{1};
            euler = varargin{2};
            R = euler2dcm(euler(1), euler(2), euler(3));
            result = R' * v_ned;

        case 'body2ned'
            v_body = varargin{1};
            euler  = varargin{2};
            R = euler2dcm(euler(1), euler(2), euler(3));
            result = R * v_body;

        case 'ned2lla'
            pos_ned = varargin{1};
            lla_ref = varargin{2};  % [lat_deg, lon_deg, alt_m]
            lat_ref = deg2rad(lla_ref(1));
            lon_ref = deg2rad(lla_ref(2));
            alt_ref = lla_ref(3);

            R_earth = 6378137;  % WGS-84 equatorial radius [m]
            lat = lat_ref + pos_ned(1) / R_earth;
            lon = lon_ref + pos_ned(2) / (R_earth * cos(lat_ref));
            alt = alt_ref - pos_ned(3);

            result = [rad2deg(lat); rad2deg(lon); alt];

        case 'lla2ned'
            lla     = varargin{1};
            lla_ref = varargin{2};
            R_earth = 6378137;
            lat_ref = deg2rad(lla_ref(1));

            dn = deg2rad(lla(1) - lla_ref(1)) * R_earth;
            de = deg2rad(lla(2) - lla_ref(2)) * R_earth * cos(lat_ref);
            dd = -(lla(3) - lla_ref(3));

            result = [dn; de; dd];

        otherwise
            error('Unknown coordinate transform: %s', operation);
    end
end


function R = euler2dcm(phi, theta, psi)
    cphi=cos(phi); sphi=sin(phi);
    cth=cos(theta); sth=sin(theta);
    cpsi=cos(psi); spsi=sin(psi);
    R = [cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi;
         cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi;
         -sth,     sphi*cth,                 cphi*cth                ];
end
